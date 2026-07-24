"""High-level composition of the URCL protocol clients.

The driver deliberately owns no dashboard client.  Construction only stores
configuration; sockets, servers, and worker threads are created by
:meth:`UrDriver.connect`.
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from importlib import resources
from pathlib import Path
from typing import Protocol, TypeAlias, TypeVar, cast

from ur_client_library.comm.control_mode import ControlMode
from ur_client_library.control._server import ClientToken
from ur_client_library.control.motion_primitives import MotionPrimitive
from ur_client_library.control.reverse_interface import (
    FreedriveControlMessage,
    TrajectoryControlMessage,
)
from ur_client_library.control.script_command_interface import ToolContactResult
from ur_client_library.control.script_reader import ScriptData
from ur_client_library.control.trajectory_point_interface import TrajectoryResult
from ur_client_library.exceptions import (
    IncompatibleRobotVersionError,
    InvalidRangeError,
    LifecycleError,
    MissingArgumentError,
    StreamNotConnectedError,
    URCLConnectionError,
    URCLTimeoutError,
)
from ur_client_library.helpers import wait_for
from ur_client_library.primary.packages import ConfigurationData, ErrorCode
from ur_client_library.rtde.data_package import DataPackage
from ur_client_library.rtde.rtde_writer import RTDEWriter
from ur_client_library.types import Pose, Q, validate_floats, validate_ints
from ur_client_library.ur.robot_receive_timeout import RobotReceiveTimeout
from ur_client_library.ur.tool_communication import ToolCommSetup
from ur_client_library.ur.version_information import VersionInformation

_LOGGER = logging.getLogger(__name__)
Unsubscribe: TypeAlias = Callable[[], None]
ProgramStateCallback: TypeAlias = Callable[[bool], None]
ComponentT = TypeVar("ComponentT")


class RTDEClientProtocol(Protocol):
    @property
    def version(self) -> VersionInformation: ...

    @property
    def target_frequency(self) -> float: ...

    @property
    def output_recipe(self) -> tuple[str, ...]: ...

    @property
    def input_recipe(self) -> tuple[str, ...]: ...

    @property
    def local_ip(self) -> str: ...

    @property
    def writer(self) -> RTDEWriter: ...

    def connect(
        self,
        *,
        max_connection_attempts: int = 0,
        reconnection_timeout: float = 10.0,
        max_initialization_attempts: int = 3,
        initialization_timeout: float = 1.0,
    ) -> None: ...

    def start(self, *, background: bool = True) -> None: ...

    def get_data_package(self, timeout: float | None = None) -> DataPackage | None: ...

    def read_data_package(self) -> DataPackage: ...

    def start_background_read(self) -> None: ...

    def stop_background_read(self) -> None: ...

    def close(self) -> None: ...


class PrimaryClientProtocol(Protocol):
    @property
    def configuration_data(self) -> ConfigurationData | None: ...

    def connect(self) -> None: ...

    def start(self) -> None: ...

    def close(self) -> None: ...

    def pop_error_codes(self) -> tuple[ErrorCode, ...]: ...

    def check_calibration(self, checksum: str, *, timeout: float | None = None) -> bool: ...

    def send_script(self, program: str) -> None: ...

    def send_script_blocking(
        self,
        program: str,
        script_name: str = "",
        *,
        start_timeout: float = 1.0,
        fail_on_warnings: bool = True,
        retry_on_readonly_interface: bool = True,
    ) -> None: ...


class ReverseInterfaceProtocol(Protocol):
    @property
    def is_connected(self) -> bool: ...

    def start(self) -> None: ...

    def close(self) -> None: ...

    def write(
        self,
        positions: Sequence[float] | None = None,
        control_mode: ControlMode | int = ControlMode.IDLE,
        robot_receive_timeout: RobotReceiveTimeout | None = None,
    ) -> None: ...

    def write_trajectory_control_message(
        self,
        action: TrajectoryControlMessage | int,
        point_number: int = 0,
        robot_receive_timeout: RobotReceiveTimeout | None = None,
    ) -> None: ...

    def write_freedrive_control_message(
        self,
        action: FreedriveControlMessage | int,
        robot_receive_timeout: RobotReceiveTimeout | None = None,
    ) -> None: ...


class TrajectoryInterfaceProtocol(Protocol):
    @property
    def is_connected(self) -> bool: ...

    def start(self) -> None: ...

    def close(self) -> None: ...

    def subscribe_trajectory_result(
        self, callback: Callable[[TrajectoryResult], None]
    ) -> Unsubscribe: ...

    def subscribe_disconnection(self, callback: Callable[[ClientToken], None]) -> Unsubscribe: ...

    def write_trajectory_point(
        self,
        positions: Sequence[float],
        *,
        acceleration: float = 1.4,
        velocity: float = 1.05,
        goal_time: float = 0.0,
        blend_radius: float = 0.0,
        cartesian: bool = False,
    ) -> None: ...

    def write_trajectory_spline_point(
        self,
        positions: Sequence[float],
        velocities: Sequence[float],
        accelerations: Sequence[float] | None = None,
        *,
        goal_time: float,
    ) -> None: ...

    def write_motion_primitive(self, primitive: MotionPrimitive) -> None: ...


class ScriptCommandInterfaceProtocol(Protocol):
    @property
    def is_connected(self) -> bool: ...

    def start(self) -> None: ...

    def close(self) -> None: ...

    def subscribe_tool_contact_result(
        self, callback: Callable[[ToolContactResult], None]
    ) -> Unsubscribe: ...


class ScriptReaderProtocol(Protocol):
    def read_script_file(self, file_path: str | Path, data: ScriptData | None = None) -> str: ...


class ScriptSenderProtocol(Protocol):
    def start(self) -> None: ...

    def close(self) -> None: ...


def _validate_port(value: int, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{name} must be an integer")
    if not 0 <= value <= 65535:
        raise ValueError(f"{name} must be in range [0, 65535]")
    return value


def _validate_nonnegative(value: float, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a real number")
    normalized = float(value)
    if normalized < 0:
        raise ValueError(f"{name} must be non-negative")
    return normalized


@dataclass(slots=True)
class UrDriverConfiguration:
    """Configuration used when :class:`UrDriver` is connected.

    Explicit recipe vectors take precedence over recipe files.  ``script_file``
    defaults to the packaged ``external_control.urscript`` resource.
    Durations are expressed in seconds.
    """

    robot_ip: str
    script_file: str | Path | None = None
    output_recipe_file: str | Path | None = None
    input_recipe_file: str | Path | None = None
    output_recipe: Sequence[str] = field(default_factory=tuple)
    input_recipe: Sequence[str] = field(default_factory=tuple)
    handle_program_state: ProgramStateCallback | None = None
    headless_mode: bool = False
    tool_comm_setup: ToolCommSetup | None = None
    reverse_port: int = 50001
    script_sender_port: int = 50002
    trajectory_port: int = 50003
    script_command_port: int = 50004
    reverse_ip: str = ""
    servoj_gain: int = 2000
    servoj_lookahead_time: float = 0.03
    socket_reconnect_attempts: int = 0
    socket_reconnection_timeout: float = 10.0
    rtde_initialization_attempts: int = 3
    rtde_initialization_timeout: float = 5.0
    non_blocking_read: bool = False
    close_timeout: float = 2.0

    def __post_init__(self) -> None:
        if not isinstance(self.robot_ip, str) or not self.robot_ip.strip():
            raise ValueError("robot_ip must be a non-empty string")
        if self.handle_program_state is not None and not callable(self.handle_program_state):
            raise TypeError("handle_program_state must be callable")
        self.output_recipe = tuple(self.output_recipe)
        self.input_recipe = tuple(self.input_recipe)
        if any(not isinstance(name, str) or not name for name in self.output_recipe):
            raise ValueError("output_recipe entries must be non-empty strings")
        if any(not isinstance(name, str) or not name for name in self.input_recipe):
            raise ValueError("input_recipe entries must be non-empty strings")
        for name in (
            "reverse_port",
            "script_sender_port",
            "trajectory_port",
            "script_command_port",
        ):
            setattr(self, name, _validate_port(getattr(self, name), name))
        for name in ("socket_reconnect_attempts", "rtde_initialization_attempts"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError(f"{name} must be an integer")
            if value < 0:
                raise ValueError(f"{name} must be non-negative")
        if self.rtde_initialization_attempts == 0:
            raise ValueError("rtde_initialization_attempts must be greater than zero")
        for name in (
            "socket_reconnection_timeout",
            "rtde_initialization_timeout",
            "close_timeout",
        ):
            setattr(self, name, _validate_nonnegative(getattr(self, name), name))
        if isinstance(self.servoj_gain, bool) or not isinstance(self.servoj_gain, int):
            raise TypeError("servoj_gain must be an integer")
        if not 100 <= self.servoj_gain <= 2000:
            raise ValueError("servoj_gain must be in range [100, 2000]")
        self.servoj_lookahead_time = float(self.servoj_lookahead_time)
        if not 0.03 <= self.servoj_lookahead_time <= 0.2:
            raise ValueError("servoj_lookahead_time must be in range [0.03, 0.2]")


@dataclass(frozen=True, slots=True)
class UrDriverFactories:
    """Typed construction seam for protocol components.

    It is primarily useful for tests and for reconciling independently
    developed subsystem constructor signatures.
    """

    rtde: Callable[
        [str, tuple[str, ...], tuple[str, ...], float, bool],
        RTDEClientProtocol,
    ]
    primary: Callable[[str], PrimaryClientProtocol]
    reverse: Callable[
        [int, ProgramStateCallback, int, VersionInformation],
        ReverseInterfaceProtocol,
    ]
    trajectory: Callable[[int], TrajectoryInterfaceProtocol]
    script_command: Callable[[int, VersionInformation], ScriptCommandInterfaceProtocol]
    script_reader: Callable[[], ScriptReaderProtocol]
    script_sender: Callable[[int, str], ScriptSenderProtocol]

    @classmethod
    def defaults(cls) -> UrDriverFactories:
        def rtde(
            robot_ip: str,
            output_recipe: tuple[str, ...],
            input_recipe: tuple[str, ...],
            target_frequency: float,
            ignore_unavailable_outputs: bool,
        ) -> RTDEClientProtocol:
            from ur_client_library.rtde.rtde_client import RTDEClient

            return RTDEClient(
                robot_ip,
                output_recipe,
                input_recipe,
                target_frequency=target_frequency,
                ignore_unavailable_outputs=ignore_unavailable_outputs,
            )

        def primary(robot_ip: str) -> PrimaryClientProtocol:
            from ur_client_library.primary.client import PrimaryClient

            return PrimaryClient(robot_ip)

        def reverse(
            port: int,
            callback: ProgramStateCallback,
            step_time_ms: int,
            version: VersionInformation,
        ) -> ReverseInterfaceProtocol:
            from ur_client_library.control.reverse_interface import (
                ReverseInterface,
                ReverseInterfaceConfig,
            )

            return ReverseInterface(
                ReverseInterfaceConfig(
                    port=port,
                    handle_program_state=callback,
                    step_time_ms=step_time_ms,
                    robot_software_version=version,
                )
            )

        def trajectory(port: int) -> TrajectoryInterfaceProtocol:
            from ur_client_library.control.trajectory_point_interface import (
                TrajectoryPointInterface,
            )

            return TrajectoryPointInterface(port)

        def script_command(
            port: int, version: VersionInformation
        ) -> ScriptCommandInterfaceProtocol:
            from ur_client_library.control.script_command_interface import (
                ScriptCommandInterface,
            )

            return ScriptCommandInterface(port, robot_software_version=version)

        def script_reader() -> ScriptReaderProtocol:
            from ur_client_library.control.script_reader import ScriptReader

            return ScriptReader()

        def script_sender(port: int, program: str) -> ScriptSenderProtocol:
            from ur_client_library.control.script_sender import ScriptSender

            return ScriptSender(program, port)

        return cls(rtde, primary, reverse, trajectory, script_command, script_reader, script_sender)


class UrDriver:
    """High-level URCL driver without dashboard ownership."""

    PRIMARY_CONFIGURATION_TIMEOUT = 10.0

    def __init__(
        self,
        configuration: UrDriverConfiguration,
        *,
        factories: UrDriverFactories | None = None,
    ) -> None:
        if not isinstance(configuration, UrDriverConfiguration):
            raise TypeError("configuration must be an UrDriverConfiguration")
        self.configuration = configuration
        self._factories = factories
        self._state_lock = threading.RLock()
        self._callback_lock = threading.Lock()
        self._program_callbacks: dict[int, ProgramStateCallback] = {}
        self._next_callback_id = 0
        self._connected = False
        self._closed = False
        self._rtde: RTDEClientProtocol | None = None
        self._primary: PrimaryClientProtocol | None = None
        self._reverse: ReverseInterfaceProtocol | None = None
        self._trajectory: TrajectoryInterfaceProtocol | None = None
        self._script_command: ScriptCommandInterfaceProtocol | None = None
        self._script_reader: ScriptReaderProtocol | None = None
        self._script_sender: ScriptSenderProtocol | None = None
        self._version = VersionInformation()
        self._output_recipe: tuple[str, ...] = ()
        self._input_recipe: tuple[str, ...] = ()
        self._full_robot_program = ""

    @property
    def is_connected(self) -> bool:
        with self._state_lock:
            return self._connected

    @property
    def version(self) -> VersionInformation:
        return self._version

    @property
    def control_frequency(self) -> float:
        return self._require(self._rtde, "RTDE client").target_frequency

    @property
    def output_recipe(self) -> tuple[str, ...]:
        if self._rtde is not None:
            return self._rtde.output_recipe
        return self._output_recipe

    @property
    def input_recipe(self) -> tuple[str, ...]:
        if self._rtde is not None:
            return self._rtde.input_recipe
        return self._input_recipe

    @property
    def rtde_writer(self) -> RTDEWriter:
        return self._require(self._rtde, "RTDE client").writer

    @property
    def primary_client(self) -> PrimaryClientProtocol:
        return self._require(self._primary, "Primary client")

    def connect(self) -> UrDriver:
        with self._state_lock:
            if self._closed:
                raise LifecycleError("A closed UrDriver cannot be connected")
            if self._connected:
                return self
            factories = self._factories or UrDriverFactories.defaults()
            self._factories = factories

        try:
            self._connect_components(factories)
        except BaseException:
            self._cleanup_failed_connect()
            raise
        return self

    def _connect_components(self, factories: UrDriverFactories) -> None:
        output_recipe, input_recipe = self._resolve_configured_recipes()
        primary = factories.primary(self.configuration.robot_ip)
        rtde = factories.rtde(self.configuration.robot_ip, output_recipe, input_recipe, 0.0, False)
        self._primary = primary
        self._rtde = rtde
        self._connect_rtde(rtde)
        self._version = rtde.version
        frequency = rtde.target_frequency
        if frequency <= 0:
            raise LifecycleError("RTDE target frequency must be positive")
        step_time_ms = max(1, int(1000 / frequency))

        reverse = factories.reverse(
            self.configuration.reverse_port,
            self._dispatch_program_state,
            step_time_ms,
            self._version,
        )
        trajectory = factories.trajectory(self.configuration.trajectory_port)
        script_command = factories.script_command(
            self.configuration.script_command_port, self._version
        )
        self._reverse = reverse
        self._trajectory = trajectory
        self._script_command = script_command
        reverse.start()
        trajectory.start()
        script_command.start()

        self.start_primary_client_communication()
        try:
            wait_for(
                lambda: primary.configuration_data is not None,
                self.PRIMARY_CONFIGURATION_TIMEOUT,
            )
        except URCLTimeoutError as exc:
            raise URCLTimeoutError(
                "Could not get ConfigurationData; is the robot reachable?",
                self.PRIMARY_CONFIGURATION_TIMEOUT,
            ) from exc

        local_ip = self.configuration.reverse_ip or rtde.local_ip
        reader = factories.script_reader()
        self._script_reader = reader
        program = self._render_external_control_script(reader, local_ip)
        if self.configuration.headless_mode:
            self._full_robot_program = self._wrap_headless_program(program)
            self.send_robot_program()
        else:
            sender = factories.script_sender(self.configuration.script_sender_port, program)
            self._script_sender = sender
            sender.start()

        self._output_recipe = output_recipe
        self._input_recipe = input_recipe
        with self._state_lock:
            self._connected = True

    def _cleanup_failed_connect(self) -> None:
        self.close()
        self._rtde = None
        self._primary = None
        self._reverse = None
        self._trajectory = None
        self._script_command = None
        self._script_reader = None
        self._script_sender = None
        self._version = VersionInformation()
        self._full_robot_program = ""
        with self._state_lock:
            self._closed = False

    def __enter__(self) -> UrDriver:
        return self.connect()

    def __exit__(
        self,
        _exc_type: type[BaseException] | None,
        _exc_value: BaseException | None,
        _traceback: object,
    ) -> None:
        self.close()

    def close(self) -> None:
        with self._state_lock:
            if self._closed:
                return
            self._closed = True
            self._connected = False
        deadline = time.monotonic() + self.configuration.close_timeout

        # Preserve upstream ownership/destruction order. Each action is bounded
        # independently by the remaining global close budget.
        self._bounded_cleanup(lambda: self.stop_control(), deadline, "stop control")
        cleanup = (
            self._script_sender,
            self._script_command,
            self._trajectory,
            self._reverse,
            self._primary,
            self._rtde,
        )
        for component in cleanup:
            if component is None:
                continue
            self._bounded_cleanup(
                component.close,
                deadline,
                f"close {type(component).__name__}",
            )

    def _bounded_cleanup(
        self, callback: Callable[[], object], deadline: float, description: str
    ) -> None:
        remaining = max(0.0, deadline - time.monotonic())
        failure: list[BaseException] = []

        def run() -> None:
            try:
                callback()
            except BaseException as exc:  # cleanup must continue
                failure.append(exc)

        worker = threading.Thread(target=run, name=f"urcl-{description}", daemon=True)
        worker.start()
        worker.join(remaining)
        if worker.is_alive():
            _LOGGER.warning("%s exceeded the driver close deadline", description)
        elif failure:
            _LOGGER.debug("%s failed during close: %s", description, failure[0])

    @staticmethod
    def _require(component: ComponentT | None, name: str) -> ComponentT:
        if component is None:
            raise LifecycleError(f"{name} is unavailable before UrDriver.connect()")
        return component

    def _resolve_configured_recipes(self) -> tuple[tuple[str, ...], tuple[str, ...]]:
        cfg = self.configuration
        if cfg.output_recipe:
            if cfg.output_recipe_file:
                _LOGGER.warning("Both output recipe vector and file are set; using the vector")
            output = tuple(cfg.output_recipe)
        elif cfg.output_recipe_file:
            output = self._read_recipe(cfg.output_recipe_file)
        else:
            raise LifecycleError("An RTDE output recipe vector or file is required")

        if cfg.input_recipe:
            if cfg.input_recipe_file:
                _LOGGER.warning("Both input recipe vector and file are set; using the vector")
            input_recipe = tuple(cfg.input_recipe)
        elif cfg.input_recipe_file:
            input_recipe = self._read_recipe(cfg.input_recipe_file)
        else:
            input_recipe = ()
        return output, input_recipe

    @staticmethod
    def _read_recipe(path: str | Path) -> tuple[str, ...]:
        recipe_path = Path(path)
        try:
            lines = tuple(recipe_path.read_text(encoding="utf-8").splitlines())
        except OSError as exc:
            raise LifecycleError(f"Could not read RTDE recipe {str(recipe_path)!r}") from exc
        if not lines:
            raise LifecycleError(f"RTDE recipe {str(recipe_path)!r} is empty")
        return lines

    def _connect_rtde(self, rtde: RTDEClientProtocol) -> None:
        rtde.connect(
            max_connection_attempts=self.configuration.socket_reconnect_attempts,
            reconnection_timeout=self.configuration.socket_reconnection_timeout,
            max_initialization_attempts=self.configuration.rtde_initialization_attempts,
            initialization_timeout=self.configuration.rtde_initialization_timeout,
        )

    def _render_external_control_script(self, reader: ScriptReaderProtocol, local_ip: str) -> str:
        substitutions: ScriptData = {
            "BEGIN_REPLACE": self._tool_setup_script(),
            "JOINT_STATE_REPLACE": 1_000_000,
            "TIME_REPLACE": 1_000_000,
            "SERVO_J_REPLACE": (
                f"lookahead_time={self.configuration.servoj_lookahead_time:g}, "
                f"gain={self.configuration.servoj_gain}"
            ),
            "SERVER_IP_REPLACE": local_ip,
            "SERVER_PORT_REPLACE": self.configuration.reverse_port,
            "TRAJECTORY_SERVER_PORT_REPLACE": self.configuration.trajectory_port,
            "SCRIPT_COMMAND_SERVER_PORT_REPLACE": self.configuration.script_command_port,
            "ROBOT_SOFTWARE_VERSION": self._version,
        }
        if self.configuration.script_file is not None:
            return reader.read_script_file(self.configuration.script_file, substitutions)
        script = (
            resources.files("ur_client_library")
            .joinpath("resources")
            .joinpath("external_control.urscript")
        )
        with resources.as_file(script) as script_path:
            return reader.read_script_file(script_path, substitutions)

    def _tool_setup_script(self) -> str:
        setup = self.configuration.tool_comm_setup
        if setup is None:
            return ""
        return setup.to_urscript(self._version)

    @staticmethod
    def _wrap_headless_program(program: str) -> str:
        body = "".join(f"\t{line}\n" for line in program.splitlines())
        return f"stop program\ndef externalControl():\n{body}end\n"

    def _dispatch_program_state(self, running: bool) -> None:
        with self._callback_lock:
            callbacks = tuple(self._program_callbacks.values())
        configured = self.configuration.handle_program_state
        if configured is not None:
            callbacks = (configured, *callbacks)
        for callback in callbacks:
            try:
                callback(running)
            except Exception:
                _LOGGER.exception("Program-state callback raised")

    def subscribe_program_state(self, callback: ProgramStateCallback) -> Unsubscribe:
        if not callable(callback):
            raise TypeError("callback must be callable")
        with self._callback_lock:
            callback_id = self._next_callback_id
            self._next_callback_id += 1
            self._program_callbacks[callback_id] = callback
        lock = threading.Lock()
        active = True

        def unsubscribe() -> None:
            nonlocal active
            with lock:
                if not active:
                    return
                active = False
            with self._callback_lock:
                self._program_callbacks.pop(callback_id, None)

        return unsubscribe

    def get_data_package(self, timeout: float | None = None) -> DataPackage | None:
        rtde = self._require(self._rtde, "RTDE client")
        effective_timeout = (
            (0.0 if self.configuration.non_blocking_read else 0.1) if timeout is None else timeout
        )
        return rtde.get_data_package(effective_timeout)

    def get_data_package_blocking(self) -> DataPackage:
        return self._require(self._rtde, "RTDE client").read_data_package()

    def start_rtde_communication(self, read_in_background: bool = True) -> None:
        self._require(self._rtde, "RTDE client").start(background=read_in_background)

    def set_rtde_background_read_enabled(self, enabled: bool) -> None:
        rtde = self._require(self._rtde, "RTDE client")
        if enabled:
            rtde.start_background_read()
        else:
            rtde.stop_background_read()

    def reset_rtde_client(
        self,
        output_recipe: Sequence[str] | str | Path,
        input_recipe: Sequence[str] | str | Path = (),
        *,
        target_frequency: float = 0.0,
        ignore_unavailable_outputs: bool = False,
    ) -> None:
        factories = self._factories or UrDriverFactories.defaults()
        output = (
            self._read_recipe(output_recipe)
            if isinstance(output_recipe, (str, Path))
            else tuple(output_recipe)
        )
        input_values = (
            self._read_recipe(input_recipe)
            if isinstance(input_recipe, (str, Path))
            else tuple(input_recipe)
        )
        old = self._rtde
        if old is not None:
            old.close()
        rtde = factories.rtde(
            self.configuration.robot_ip,
            output,
            input_values,
            float(target_frequency),
            ignore_unavailable_outputs,
        )
        self._connect_rtde(rtde)
        self._rtde = rtde
        self._output_recipe = output
        self._input_recipe = input_values
        self._version = rtde.version

    def start_primary_client_communication(self) -> None:
        primary = self._require(self._primary, "Primary client")
        attempts = 0
        while True:
            attempts += 1
            try:
                primary.connect()
                break
            except URCLConnectionError:
                limit = self.configuration.socket_reconnect_attempts
                if limit and attempts >= limit:
                    raise
                time.sleep(self.configuration.socket_reconnection_timeout)
        primary.start()

    def stop_primary_client_communication(self) -> None:
        self._require(self._primary, "Primary client").close()

    def pop_error_codes(self) -> tuple[ErrorCode, ...]:
        return self._require(self._primary, "Primary client").pop_error_codes()

    def check_calibration(self, checksum: str) -> bool:
        return self._require(self._primary, "Primary client").check_calibration(checksum)

    def send_script(self, program: str) -> None:
        self._require(self._primary, "Primary client").send_script(program)

    def send_script_blocking(
        self,
        program: str,
        script_name: str = "",
        *,
        start_timeout: float = 1.0,
        fail_on_warnings: bool = True,
        retry_on_readonly_interface: bool = True,
    ) -> None:
        self._require(self._primary, "Primary client").send_script_blocking(
            program,
            script_name,
            start_timeout=start_timeout,
            fail_on_warnings=fail_on_warnings,
            retry_on_readonly_interface=retry_on_readonly_interface,
        )

    def send_robot_program(self) -> None:
        if not self.configuration.headless_mode:
            raise LifecycleError("send_robot_program is only valid in headless mode")
        if not self._full_robot_program:
            raise LifecycleError("The headless robot program has not been rendered")
        self.send_script(self._full_robot_program)

    def write_joint_command(
        self,
        target: Q | Pose | Sequence[float],
        control_mode: ControlMode,
        robot_receive_timeout: RobotReceiveTimeout | None = None,
    ) -> None:
        values = (
            tuple(target)
            if isinstance(target, (Q, Pose))
            else validate_floats(target, 6, name="target")
        )
        timeout = robot_receive_timeout or RobotReceiveTimeout.from_milliseconds()
        self._require(self._reverse, "Reverse interface").write(values, control_mode, timeout)

    def write_keepalive(
        self,
        robot_receive_timeout: RobotReceiveTimeout | None = None,
    ) -> None:
        timeout = robot_receive_timeout or RobotReceiveTimeout.from_milliseconds(1000)
        self._require(self._reverse, "Reverse interface").write(None, ControlMode.IDLE, timeout)

    def stop_control(self) -> None:
        self._require(self._reverse, "Reverse interface").write(None, ControlMode.STOPPED)

    def write_trajectory_control_message(
        self,
        action: TrajectoryControlMessage | int,
        point_number: int = 0,
        robot_receive_timeout: RobotReceiveTimeout | None = None,
    ) -> None:
        timeout = robot_receive_timeout or RobotReceiveTimeout.from_milliseconds(200)
        self._require(self._reverse, "Reverse interface").write_trajectory_control_message(
            action, point_number, timeout
        )

    def write_freedrive_control_message(
        self,
        action: FreedriveControlMessage | int,
        robot_receive_timeout: RobotReceiveTimeout | None = None,
    ) -> None:
        timeout = robot_receive_timeout or RobotReceiveTimeout.from_milliseconds(200)
        self._require(self._reverse, "Reverse interface").write_freedrive_control_message(
            action, timeout
        )

    def write_trajectory_point(
        self,
        target: Q | Pose,
        *,
        acceleration: float | None = None,
        velocity: float | None = None,
        goal_time: float = 0.0,
        blend_radius: float = 0.052,
    ) -> None:
        if not isinstance(target, (Q, Pose)):
            raise TypeError("target must be Q or Pose")
        trajectory = self._require(self._trajectory, "Trajectory interface")
        trajectory.write_trajectory_point(
            tuple(target),
            acceleration=1.4 if acceleration is None else acceleration,
            velocity=1.05 if velocity is None else velocity,
            cartesian=isinstance(target, Pose),
            goal_time=goal_time,
            blend_radius=blend_radius,
        )

    def write_trajectory_spline_point(
        self,
        positions: Q | Pose | Sequence[float],
        velocities: Sequence[float],
        accelerations: Sequence[float] | None = None,
        *,
        goal_time: float = 0.0,
    ) -> None:
        position_values = validate_floats(positions, 6, name="positions")
        velocity_values = validate_floats(velocities, 6, name="velocities")
        acceleration_values = (
            None
            if accelerations is None
            else validate_floats(accelerations, 6, name="accelerations")
        )
        self._require(self._trajectory, "Trajectory interface").write_trajectory_spline_point(
            position_values,
            velocity_values,
            acceleration_values,
            goal_time=goal_time,
        )

    def write_motion_primitive(self, primitive: MotionPrimitive) -> None:
        self._require(self._trajectory, "Trajectory interface").write_motion_primitive(primitive)

    @property
    def is_reverse_interface_connected(self) -> bool:
        return self._require(self._reverse, "Reverse interface").is_connected

    @property
    def is_trajectory_interface_connected(self) -> bool:
        return self._require(self._trajectory, "Trajectory interface").is_connected

    def subscribe_trajectory_done(
        self, callback: Callable[[TrajectoryResult], None]
    ) -> Unsubscribe:
        return self._require(self._trajectory, "Trajectory interface").subscribe_trajectory_result(
            callback
        )

    def subscribe_trajectory_disconnected(
        self, callback: Callable[[ClientToken], None]
    ) -> Unsubscribe:
        return self._require(self._trajectory, "Trajectory interface").subscribe_disconnection(
            callback
        )

    def subscribe_tool_contact_result(
        self, callback: Callable[[ToolContactResult], None]
    ) -> Unsubscribe:
        return self._require(
            self._script_command, "Script command interface"
        ).subscribe_tool_contact_result(callback)

    def _script_command_connected(self) -> bool:
        return self._require(self._script_command, "Script command interface").is_connected

    def _script_command_or_none(self, name: str) -> Callable[..., None] | None:
        if not self._script_command_connected():
            return None
        command = getattr(
            self._require(self._script_command, "Script command interface"),
            name,
            None,
        )
        if command is None or not callable(command):
            raise AttributeError(f"Script command interface does not provide {name!r}")
        return cast(Callable[..., None], command)

    def zero_ft_sensor(self) -> None:
        required = VersionInformation(5, 0)
        if self._version < required:
            raise IncompatibleRobotVersionError(
                "Zeroing the force/torque sensor requires e-Series", required, self._version
            )
        command = self._script_command_or_none("zero_ft_sensor")
        if command is not None:
            command()
            return
        self.send_script("sec tareSetup():\n zero_ftsensor()\nend")

    def set_payload(self, mass: float, cog: Sequence[float]) -> None:
        values = validate_floats(cog, 3, name="cog")
        command = self._script_command_or_none("set_payload")
        if command is not None:
            command(float(mass), values)
            return
        self.send_script(
            f"sec setup():\n set_payload({float(mass):g}, {self._script_vector(values)})\nend"
        )

    def set_target_payload(
        self,
        mass: float,
        cog: Sequence[float],
        inertia: Sequence[float] = (0, 0, 0, 0, 0, 0),
        transition_time: float = 0.0,
    ) -> None:
        cog_values = validate_floats(cog, 3, name="cog")
        inertia_values = validate_floats(inertia, 6, name="inertia")
        command = self._script_command_or_none("set_target_payload")
        if command is not None:
            command(
                float(mass),
                cog_values,
                inertia_values,
                float(transition_time),
            )
            return
        if self._version >= VersionInformation(5, 10):
            body = (
                f" set_target_payload({float(mass):g}, {self._script_vector(cog_values)}, "
                f"{self._script_vector(inertia_values)}, {float(transition_time):g})"
            )
        else:
            body = (
                f" set_payload({float(mass):g}, {self._script_vector(cog_values)})\n"
                ' textmsg("PolyScope < 5.10.0. Inertia and transition_time ignored.")'
            )
        self.send_script(f"sec setup():\n{body}\nend")

    def set_gravity(self, gravity: Sequence[float]) -> None:
        values = validate_floats(gravity, 3, name="gravity")
        command = self._script_command_or_none("set_gravity")
        if command is not None:
            command(values)
            return
        self.send_script(f"sec setup():\n set_gravity({self._script_vector(values)})\nend")

    def set_tool_voltage(self, voltage: int) -> None:
        if voltage not in (0, 12, 24):
            raise InvalidRangeError("tool voltage must be 0, 12, or 24")
        command = self._script_command_or_none("set_tool_voltage")
        if command is not None:
            command(voltage)
            return
        self.send_script(f"sec setup():\n set_tool_voltage({voltage})\nend")

    def start_force_mode(
        self,
        task_frame: Sequence[float],
        selection_vector: Sequence[int],
        wrench: Sequence[float],
        force_type: int,
        limits: Sequence[float],
        *,
        damping: float,
        gain_scaling: float | None = None,
    ) -> None:
        frame = validate_floats(task_frame, 6, name="task_frame")
        selection = validate_ints(
            selection_vector, 6, name="selection_vector", minimum=0, maximum=1
        )
        wrench_values = validate_floats(wrench, 6, name="wrench")
        limit_values = validate_floats(limits, 6, name="limits")
        if force_type not in (1, 2, 3):
            raise InvalidRangeError("force_type must be 1, 2, or 3")
        if not 0 <= damping <= 1:
            raise InvalidRangeError("damping must be in range [0, 1]")
        if self._version.major >= 5:
            if gain_scaling is None:
                raise MissingArgumentError("gain_scaling is required on e-Series controllers")
            if not 0 <= gain_scaling <= 2:
                raise InvalidRangeError("gain_scaling must be in range [0, 2]")
        elif gain_scaling is not None:
            required = VersionInformation(5, 0)
            raise IncompatibleRobotVersionError(
                "Force-mode gain scaling is unavailable on CB3", required, self._version
            )
        command = self._script_command_or_none("start_force_mode")
        if command is None:
            raise StreamNotConnectedError("Script command interface is not connected")
        command(
            frame,
            selection,
            wrench_values,
            force_type,
            limit_values,
            float(damping),
            0.0 if gain_scaling is None else float(gain_scaling),
        )

    def end_force_mode(self) -> None:
        self._connected_script_command("end_force_mode")

    def start_tool_contact(self) -> None:
        self._require_e_series("Tool contact")
        self._connected_script_command("start_tool_contact")

    def end_tool_contact(self) -> None:
        self._require_e_series("Tool contact")
        self._connected_script_command("end_tool_contact")

    def set_tcp_offset(self, offset: Sequence[float]) -> None:
        values = validate_floats(offset, 6, name="offset")
        self._connected_script_command("set_tcp_offset", values)

    def set_friction_scales(
        self, viscous_scale: Sequence[float], coulomb_scale: Sequence[float]
    ) -> None:
        viscous = validate_floats(viscous_scale, 6, name="viscous_scale")
        coulomb = validate_floats(coulomb_scale, 6, name="coulomb_scale")
        self._connected_script_command("set_friction_scales", viscous, coulomb)

    def ft_rtde_input_enable(
        self,
        enabled: bool,
        sensor_mass: float = 0.0,
        sensor_measuring_offset: Sequence[float] = (0, 0, 0),
        sensor_cog: Sequence[float] = (0, 0, 0),
    ) -> None:
        offset = validate_floats(sensor_measuring_offset, 3, name="sensor_measuring_offset")
        cog = validate_floats(sensor_cog, 3, name="sensor_cog")
        self._connected_script_command(
            "ft_rtde_input_enable", enabled, float(sensor_mass), offset, cog
        )

    def _connected_script_command(self, name: str, *args: object) -> None:
        command = self._script_command_or_none(name)
        if command is None:
            raise StreamNotConnectedError("Script command interface is not connected")
        command(*args)

    def _require_e_series(self, feature: str) -> None:
        required = VersionInformation(5, 0)
        if self._version < required:
            raise IncompatibleRobotVersionError(
                f"{feature} requires e-Series", required, self._version
            )

    @staticmethod
    def _script_vector(values: Sequence[float]) -> str:
        return "[" + ", ".join(f"{value:g}" for value in values) + "]"
