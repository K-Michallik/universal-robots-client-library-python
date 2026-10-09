"""Reverse-socket control interface."""

from __future__ import annotations

import logging
import threading
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from enum import IntEnum
from typing import TypeVar

from ur_client_library.comm.control_mode import ControlMode
from ur_client_library.exceptions import StreamNotConnectedError, URCLConnectionError
from ur_client_library.types import validate_floats
from ur_client_library.ur.robot_receive_timeout import RobotReceiveTimeout
from ur_client_library.ur.version_information import VersionInformation

from ._serialization import fixed, int32, pack_int32s
from ._server import ClientToken, Server, ServerFactory, socket_server_factory

_LOGGER = logging.getLogger(__name__)

MULT_JOINTSTATE = 1_000_000
MULT_TIME = 1_000_000
DEFAULT_REVERSE_PORT = 50001
DEFAULT_SCRIPT_SENDER_PORT = 50002
DEFAULT_TRAJECTORY_PORT = 50003
DEFAULT_SCRIPT_COMMAND_PORT = 50004

ProgramStateCallback = Callable[[bool], None]
DisconnectionCallback = Callable[[ClientToken], None]
Unsubscribe = Callable[[], None]
CallbackT = TypeVar("CallbackT", bound=Callable[..., None])


class TrajectoryControlMessage(IntEnum):
    CANCEL = -1
    NOOP = 0
    START = 1
    STREAM_START = 2
    STREAM_END = 3


class FreedriveControlMessage(IntEnum):
    STOP = -1
    NOOP = 0
    START = 1


@dataclass(frozen=True, slots=True)
class ReverseInterfaceConfig:
    port: int = DEFAULT_REVERSE_PORT
    host: str = "0.0.0.0"
    handle_program_state: ProgramStateCallback | None = None
    step_time_ms: int = 8
    robot_software_version: VersionInformation = field(default_factory=VersionInformation)

    def __post_init__(self) -> None:
        if isinstance(self.port, bool) or not isinstance(self.port, int):
            raise TypeError("port must be an integer")
        if not 0 <= self.port <= 65535:
            raise ValueError("port must be in range [0, 65535]")
        if isinstance(self.step_time_ms, bool) or not isinstance(self.step_time_ms, int):
            raise TypeError("step_time_ms must be an integer")
        if self.step_time_ms <= 0:
            raise ValueError("step_time_ms must be positive")


class ReverseInterface:
    """Serve the fixed eight-word reverse-control protocol."""

    MULT_JOINTSTATE = MULT_JOINTSTATE
    MULT_TIME = MULT_TIME
    MESSAGE_WORDS = 8

    def __init__(
        self,
        config: ReverseInterfaceConfig | None = None,
        *,
        server: Server | None = None,
        server_factory: ServerFactory = socket_server_factory,
    ) -> None:
        self._config = config or ReverseInterfaceConfig()
        self._server = server or server_factory(self._config.host, self._config.port, 1)
        self._server.set_callbacks(
            on_connect=self._connection_callback,
            on_disconnect=self._disconnection_callback,
            on_message=self._message_callback,
        )
        self._lock = threading.RLock()
        self._client: ClientToken | None = None
        self._program_state_callbacks: dict[int, ProgramStateCallback] = {}
        self._disconnection_callbacks: dict[int, DisconnectionCallback] = {}
        self._next_callback_id = 0
        if self._config.handle_program_state is not None:
            self._program_state_callbacks[self._allocate_callback_id()] = (
                self._config.handle_program_state
            )

    @property
    def port(self) -> int:
        return self._server.port

    @property
    def is_connected(self) -> bool:
        with self._lock:
            return self._client is not None

    @property
    def is_running(self) -> bool:
        return self._server.is_running

    @property
    def robot_software_version(self) -> VersionInformation:
        return self._config.robot_software_version

    def start(self) -> None:
        already_running = self._server.is_running
        try:
            self._server.start()
        except OSError as exc:
            raise URCLConnectionError(
                f"Could not start reverse interface on port {self._config.port}"
            ) from exc
        if not already_running:
            self._notify_program_state(False)

    connect = start

    def close(self) -> None:
        with self._lock:
            had_client = self._client is not None
            self._client = None
        self._server.close()
        if had_client:
            self._notify_program_state(False)

    def __enter__(self) -> ReverseInterface:
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def subscribe_program_state(
        self, callback: ProgramStateCallback, *, notify_current: bool = False
    ) -> Unsubscribe:
        callback_id = self._add_callback(self._program_state_callbacks, callback)
        if notify_current:
            callback(self.is_connected)
        return self._unsubscribe(self._program_state_callbacks, callback_id)

    def subscribe_disconnection(self, callback: DisconnectionCallback) -> Unsubscribe:
        callback_id = self._add_callback(self._disconnection_callbacks, callback)
        return self._unsubscribe(self._disconnection_callbacks, callback_id)

    def write(
        self,
        positions: Sequence[float] | None = None,
        control_mode: ControlMode | int = ControlMode.IDLE,
        robot_receive_timeout: RobotReceiveTimeout | None = None,
    ) -> None:
        if isinstance(control_mode, bool):
            raise TypeError("control_mode must not be a bool")
        try:
            mode = ControlMode(control_mode)
        except ValueError as exc:
            raise ValueError(f"Unknown control mode: {control_mode}") from exc
        timeout = robot_receive_timeout or RobotReceiveTimeout.from_milliseconds()
        read_timeout = (
            100
            if mode is ControlMode.STOPPED
            else timeout.verified(mode, self._config.step_time_ms)
        )
        position_values = (
            (0.0,) * 6
            if positions is None
            else validate_floats(positions, 6, name="positions")
        )
        words = [int32(read_timeout, name="read_timeout")]
        words.extend(
            fixed(value, MULT_JOINTSTATE, name=f"positions[{index}]")
            for index, value in enumerate(position_values)
        )
        words.append(int32(int(mode), name="control_mode"))
        self._write_packet(pack_int32s(words))

    def write_trajectory_control_message(
        self,
        action: TrajectoryControlMessage | int,
        point_number: int = 0,
        robot_receive_timeout: RobotReceiveTimeout | None = None,
    ) -> None:
        if isinstance(action, bool):
            raise TypeError("action must not be a bool")
        try:
            normalized_action = TrajectoryControlMessage(action)
        except ValueError as exc:
            raise ValueError(f"Unknown trajectory control message: {action}") from exc
        timeout = robot_receive_timeout or RobotReceiveTimeout.from_milliseconds(200)
        read_timeout = timeout.verified(ControlMode.FORWARD, self._config.step_time_ms)
        words = [
            int32(read_timeout, name="read_timeout"),
            int(normalized_action),
            int32(point_number, name="point_number"),
            0,
            0,
            0,
            0,
            int(ControlMode.FORWARD),
        ]
        self._write_packet(pack_int32s(words))

    def write_freedrive_control_message(
        self,
        action: FreedriveControlMessage | int,
        robot_receive_timeout: RobotReceiveTimeout | None = None,
    ) -> None:
        if isinstance(action, bool):
            raise TypeError("action must not be a bool")
        try:
            normalized_action = FreedriveControlMessage(action)
        except ValueError as exc:
            raise ValueError(f"Unknown freedrive control message: {action}") from exc
        timeout = robot_receive_timeout or RobotReceiveTimeout.from_milliseconds(200)
        read_timeout = timeout.verified(ControlMode.FREEDRIVE, self._config.step_time_ms)
        words = [
            int32(read_timeout, name="read_timeout"),
            int(normalized_action),
            0,
            0,
            0,
            0,
            0,
            int(ControlMode.FREEDRIVE),
        ]
        self._write_packet(pack_int32s(words))

    def _write_packet(self, packet: bytes) -> None:
        if len(packet) != self.MESSAGE_WORDS * 4:
            raise AssertionError("Reverse packet must contain exactly eight int32 words")
        with self._lock:
            client = self._client
        if client is None:
            raise StreamNotConnectedError("No robot is connected to the reverse interface")
        self._server.write(client, packet)

    def _connection_callback(self, client: ClientToken) -> None:
        with self._lock:
            if self._client is not None:
                _LOGGER.error("Ignoring a second reverse-interface client")
                return
            self._client = client
        _LOGGER.info("Robot connected to reverse interface. Ready to receive control commands.")
        self._notify_program_state(True)

    def _disconnection_callback(self, client: ClientToken) -> None:
        with self._lock:
            if self._client != client:
                return
            self._client = None
            callbacks = tuple(self._disconnection_callbacks.values())
        _LOGGER.info("Connection to reverse interface dropped.")
        self._notify_program_state(False)
        self._invoke_callbacks(callbacks, client)

    def _message_callback(self, _client: ClientToken, data: bytes) -> None:
        _LOGGER.warning(
            "Ignoring %d unexpected bytes received on the reverse interface", len(data)
        )

    def _notify_program_state(self, running: bool) -> None:
        with self._lock:
            callbacks = tuple(self._program_state_callbacks.values())
        self._invoke_callbacks(callbacks, running)

    @staticmethod
    def _invoke_callbacks(callbacks: Sequence[Callable[..., None]], *args: object) -> None:
        for callback in callbacks:
            try:
                callback(*args)
            except Exception:
                _LOGGER.exception("External-control callback raised")

    def _allocate_callback_id(self) -> int:
        callback_id = self._next_callback_id
        self._next_callback_id += 1
        return callback_id

    def _add_callback(self, callbacks: dict[int, CallbackT], callback: CallbackT) -> int:
        if not callable(callback):
            raise TypeError("callback must be callable")
        with self._lock:
            callback_id = self._allocate_callback_id()
            callbacks[callback_id] = callback
        return callback_id

    def _unsubscribe(
        self, callbacks: dict[int, CallbackT], callback_id: int
    ) -> Unsubscribe:
        def unsubscribe() -> None:
            with self._lock:
                callbacks.pop(callback_id, None)

        return unsubscribe
