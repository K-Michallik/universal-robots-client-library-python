"""Synchronous RTDE client with optional background reading and reconnect."""

from __future__ import annotations

import logging
import math
import os
import threading
import time
from collections.abc import Sequence
from enum import IntEnum
from pathlib import Path
from typing import Protocol, TypeVar, cast

from ur_client_library.comm.pipeline import Notifier, NullNotifier
from ur_client_library.comm.stream import URStream
from ur_client_library.exceptions import (
    LifecycleError,
    ProtocolError,
    RTDEInputConflictError,
    RTDEInvalidKeyError,
    UnexpectedResponseError,
    URCLConnectionError,
    URCLTimeoutError,
    WorkerThreadError,
)
from ur_client_library.rtde.data_package import DataPackage
from ur_client_library.rtde.package_header import PackageHeader
from ur_client_library.rtde.rtde_package import (
    ControlPackagePause,
    ControlPackageSetupInputs,
    ControlPackageSetupOutputs,
    ControlPackageStart,
    GetUrcontrolVersion,
    RequestProtocolVersion,
    RTDEPackage,
    TextMessage,
    serialize_get_urcontrol_version_request,
    serialize_pause_request,
    serialize_protocol_version_request,
    serialize_setup_inputs_request,
    serialize_setup_outputs_request,
    serialize_start_request,
)
from ur_client_library.rtde.rtde_parser import RTDEParser
from ur_client_library.rtde.rtde_writer import RTDEWriter
from ur_client_library.ur.version_information import VersionInformation

LOGGER = logging.getLogger(__name__)
RTDE_PORT = 30004
MAX_REQUEST_RETRIES = 5
P = TypeVar("P", bound=RTDEPackage)


class RTDEStream(Protocol):
    @property
    def local_ip(self) -> str: ...

    @property
    def is_connected(self) -> bool: ...

    def connect(
        self,
        *,
        max_attempts: int = 0,
        retry_interval: float = 10.0,
        cancel_event: threading.Event | None = None,
    ) -> None: ...

    def read(self) -> bytes: ...

    def write(self, data: bytes | bytearray | memoryview) -> int: ...

    def close(self) -> None: ...


class ClientState(IntEnum):
    UNINITIALIZED = 0
    INITIALIZING = 1
    INITIALIZED = 2
    RUNNING = 3
    PAUSED = 4
    CONNECTION_LOST = 5


class RobotStatusBit(IntEnum):
    IS_POWER_ON = 0
    IS_PROGRAM_RUNNING = 1
    IS_TEACH_BUTTON_PRESSED = 2
    IS_POWER_BUTTON_PRESSED = 3


class SafetyStatusBit(IntEnum):
    IS_NORMAL_MODE = 0
    IS_REDUCED_MODE = 1
    IS_PROTECTIVE_STOPPED = 2
    IS_RECOVERY_MODE = 3
    IS_SAFEGUARD_STOPPED = 4
    IS_SYSTEM_EMERGENCY_STOPPED = 5
    IS_ROBOT_EMERGENCY_STOPPED = 6
    IS_EMERGENCY_STOPPED = 7
    IS_VIOLATION = 8
    IS_FAULT = 9
    IS_STOPPED_DUE_TO_SAFETY = 10
    IS_3PE_INPUT_ACTIVE = 11


class _RetryHandshake(Exception):
    pass


class RTDEClient:
    """Manage RTDE negotiation, recipes, data flow, and reconnection."""

    CB3_MAX_FREQUENCY = 125.0
    E_SERIES_MAX_FREQUENCY = 500.0

    def __init__(
        self,
        host: str,
        output_recipe: Sequence[str] | str,
        input_recipe: Sequence[str] | str = (),
        *,
        target_frequency: float = 0.0,
        ignore_unavailable_outputs: bool = False,
        port: int = RTDE_PORT,
        notifier: Notifier | None = None,
        stream: RTDEStream | None = None,
    ) -> None:
        if not isinstance(host, str):
            raise TypeError("host must be a string")
        if isinstance(port, bool) or not isinstance(port, int):
            raise TypeError("port must be an integer")
        if not 0 <= port <= 65535:
            raise ValueError("port must be in [0, 65535]")
        if not isinstance(ignore_unavailable_outputs, bool):
            raise TypeError("ignore_unavailable_outputs must be a bool")
        target_frequency = self._validate_interval(target_frequency, "target_frequency")
        output = (
            self.read_recipe(output_recipe)
            if isinstance(output_recipe, str)
            else tuple(output_recipe)
        )
        inputs = (
            self.read_recipe(input_recipe) if isinstance(input_recipe, str) else tuple(input_recipe)
        )
        if not output:
            raise ValueError("RTDE output recipe cannot be empty")
        self._output_recipe = self._ensure_timestamp(tuple(output))
        self._input_recipe = tuple(inputs)
        # Validate every locally known key without performing network I/O.
        DataPackage(self._output_recipe)
        if self._input_recipe:
            DataPackage(self._input_recipe)
        self._host = host
        self._port = port
        self._ignore_unavailable_outputs = ignore_unavailable_outputs
        self._requested_frequency = float(target_frequency)
        self._target_frequency = float(target_frequency)
        self._max_frequency = self.E_SERIES_MAX_FREQUENCY
        self._version = VersionInformation()
        self._protocol_version = 1
        self._state = ClientState.UNINITIALIZED
        self._state_lock = threading.RLock()
        self._stream = stream or URStream(
            host,
            port,
            length_size=2,
            length_decoder=PackageHeader.get_package_length,
            max_packet_size=0xFFFF,
            receive_timeout=1.0,
        )
        self._parser = RTDEParser(self._output_recipe)
        self._writer = RTDEWriter(self._stream, self._input_recipe)
        self._notifier = notifier or NullNotifier()
        self._io_lock = threading.Lock()
        self._latest_condition = threading.Condition()
        self._latest: DataPackage | None = None
        self._new_latest = False
        self._background_running = threading.Event()
        self._background_thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._reconnect_lock = threading.Lock()
        self._reconnect_thread: threading.Thread | None = None
        self._reconnecting = threading.Event()
        self._worker_error: WorkerThreadError | None = None
        self._worker_error_lock = threading.Lock()
        self._prefetched: DataPackage | None = None
        self._max_connection_attempts = 0
        self._reconnection_timeout = 10.0
        self._max_initialization_attempts = 3
        self._initialization_timeout = 1.0

    @staticmethod
    def read_recipe(recipe_file: str | Path) -> tuple[str, ...]:
        path = Path(recipe_file)
        try:
            lines = tuple(path.read_text(encoding="utf-8").splitlines())
        except OSError as exc:
            raise URCLConnectionError(f"Could not read RTDE recipe {str(path)!r}") from exc
        if not lines:
            raise ValueError(f"RTDE recipe {str(path)!r} is empty")
        return lines

    @staticmethod
    def _ensure_timestamp(recipe: tuple[str, ...]) -> tuple[str, ...]:
        return recipe if "timestamp" in recipe else (*recipe, "timestamp")

    @property
    def state(self) -> ClientState:
        with self._state_lock:
            return self._state

    @property
    def is_connected(self) -> bool:
        return self.state >= ClientState.INITIALIZED and self.state != ClientState.CONNECTION_LOST

    @property
    def protocol_version(self) -> int:
        return self._protocol_version

    @property
    def version(self) -> VersionInformation:
        return self._version

    @property
    def max_frequency(self) -> float:
        return self._max_frequency

    @property
    def target_frequency(self) -> float:
        return self._target_frequency

    @property
    def output_recipe(self) -> tuple[str, ...]:
        return self._output_recipe

    @property
    def input_recipe(self) -> tuple[str, ...]:
        return self._input_recipe

    @property
    def local_ip(self) -> str:
        self._raise_worker_error()
        return self._stream.local_ip

    @property
    def writer(self) -> RTDEWriter:
        self._raise_worker_error()
        return self._writer

    def connect(
        self,
        *,
        max_connection_attempts: int = 0,
        reconnection_timeout: float = 10.0,
        max_initialization_attempts: int = 3,
        initialization_timeout: float = 1.0,
    ) -> None:
        self._raise_worker_error()
        if isinstance(max_initialization_attempts, bool) or not isinstance(
            max_initialization_attempts, int
        ):
            raise TypeError("max_initialization_attempts must be an integer")
        if max_initialization_attempts <= 0:
            raise ValueError("max_initialization_attempts must be greater than zero")
        if isinstance(max_connection_attempts, bool) or not isinstance(
            max_connection_attempts, int
        ):
            raise TypeError("max_connection_attempts must be an integer")
        if max_connection_attempts < 0:
            raise ValueError("max_connection_attempts cannot be negative")
        reconnection_timeout = self._validate_interval(
            reconnection_timeout, "reconnection_timeout"
        )
        initialization_timeout = self._validate_interval(
            initialization_timeout, "initialization_timeout"
        )
        with self._state_lock:
            if self._state in (
                ClientState.INITIALIZED,
                ClientState.RUNNING,
                ClientState.PAUSED,
            ):
                return
            if self._state is ClientState.CONNECTION_LOST:
                raise LifecycleError("RTDE client is currently reconnecting")
        self._max_connection_attempts = max_connection_attempts
        self._reconnection_timeout = reconnection_timeout
        self._max_initialization_attempts = max_initialization_attempts
        self._initialization_timeout = initialization_timeout
        self._stop.clear()
        last_error: BaseException | None = None
        for attempt in range(max_initialization_attempts):
            try:
                self._setup_communication()
                with self._state_lock:
                    self._state = ClientState.INITIALIZED
                return
            except (
                URCLConnectionError,
                ProtocolError,
                RTDEInputConflictError,
                _RetryHandshake,
            ) as exc:
                last_error = exc
                self._disconnect_transport()
                if attempt + 1 < max_initialization_attempts and self._stop.wait(
                    initialization_timeout
                ):
                    break
        with self._state_lock:
            self._state = ClientState.UNINITIALIZED
        raise URCLConnectionError(
            f"Failed to initialize RTDE client after {max_initialization_attempts} attempts"
        ) from last_error

    init = connect

    def _setup_communication(self) -> None:
        with self._io_lock:
            with self._state_lock:
                self._state = ClientState.INITIALIZING
            self._stream.connect(
                max_attempts=self._max_connection_attempts,
                retry_interval=self._reconnection_timeout,
                cancel_event=self._stop,
            )
            self._protocol_version = self._negotiate_protocol_version()
            self._parser.protocol_version = self._protocol_version
            version_package = self._request_expected(
                serialize_get_urcontrol_version_request(),
                GetUrcontrolVersion,
                "URControl version",
            )
            self._version = version_package.version
            self._set_target_frequency()
            self._setup_outputs()
            self._check_boot_gate()
            if self._input_recipe:
                self._setup_inputs()

    def _negotiate_protocol_version(self) -> int:
        for version in (2, 1):
            self._parser.protocol_version = 1
            response = self._request_expected(
                serialize_protocol_version_request(version),
                RequestProtocolVersion,
                f"RTDE protocol version {version}",
            )
            if response.accepted:
                return version
        raise ProtocolError("Robot rejected RTDE protocol versions 2 and 1")

    def _set_target_frequency(self) -> None:
        self._max_frequency = (
            self.CB3_MAX_FREQUENCY if self._version.major < 5 else self.E_SERIES_MAX_FREQUENCY
        )
        requested = self._requested_frequency
        if requested == 0:
            self._target_frequency = self._max_frequency
        elif requested <= 0 or requested > self._max_frequency:
            raise ProtocolError(
                f"RTDE target frequency {requested:g} is outside (0, {self._max_frequency:g}]"
            )
        else:
            self._target_frequency = requested

    def _setup_outputs(self) -> None:
        if self._protocol_version == 1 and self._target_frequency != self._max_frequency:
            LOGGER.warning(
                "RTDE protocol version 1 cannot set a target frequency; "
                "the controller will use its maximum frequency"
            )
        response = self._request_expected(
            serialize_setup_outputs_request(
                self._output_recipe,
                protocol_version=self._protocol_version,
                frequency=self._target_frequency,
            ),
            ControlPackageSetupOutputs,
            "RTDE output recipe",
            resend_on_unexpected=True,
        )
        if len(response.variable_types) != len(self._output_recipe):
            raise ProtocolError(
                "RTDE output setup returned a different number of field types than requested"
            )
        unavailable = tuple(
            key
            for key, field_type in zip(
                self._output_recipe,
                response.variable_types,
                strict=True,
            )
            if field_type == "NOT_FOUND"
        )
        if not unavailable:
            return
        if not self._ignore_unavailable_outputs:
            raise ProtocolError(
                "Robot does not provide RTDE outputs: "
                + ", ".join(repr(key) for key in unavailable)
            )
        available = tuple(key for key in self._output_recipe if key not in unavailable)
        if "timestamp" not in available:
            raise ProtocolError("Robot does not provide required RTDE timestamp output")
        LOGGER.warning("Removing unavailable RTDE outputs: %s", ", ".join(unavailable))
        self._output_recipe = available
        self._parser = RTDEParser(available, self._protocol_version)
        raise _RetryHandshake

    def _setup_inputs(self) -> None:
        response = self._request_expected(
            serialize_setup_inputs_request(self._input_recipe),
            ControlPackageSetupInputs,
            "RTDE input recipe",
        )
        if len(response.variable_types) != len(self._input_recipe):
            raise ProtocolError(
                "RTDE input setup returned a different number of field types than requested"
            )
        for key, field_type in zip(
            self._input_recipe,
            response.variable_types,
            strict=True,
        ):
            if field_type == "NOT_FOUND":
                raise RTDEInvalidKeyError(f"Robot does not recognize RTDE input {key!r}")
            if field_type == "IN_USE":
                raise RTDEInputConflictError(key)
        self._writer.close()
        self._writer.start(response.recipe_id)

    def _check_boot_gate(self) -> None:
        self._send_start()
        timestamp = 0.0
        reads = 0
        maximum_reads = max(1, math.ceil(self._target_frequency * 2))
        if self._prefetched is not None:
            timestamp = cast(float, self._prefetched["timestamp"])
            reads = 1
            self._prefetched = None
        while timestamp < 40.0 and reads < maximum_reads:
            package = self._read_until_data()
            timestamp = cast(float, package["timestamp"])
            reads += 1
        self._send_pause()

    def start(self, *, background: bool = True) -> None:
        self._raise_worker_error()
        with self._state_lock:
            if self._state is ClientState.RUNNING:
                if background and not self._background_running.is_set():
                    self.start_background_read()
                return
            if self._state not in (ClientState.INITIALIZED, ClientState.PAUSED):
                raise LifecycleError("RTDE client must be connected before it can be started")
        with self._io_lock:
            self._send_start()
        with self._state_lock:
            self._state = ClientState.RUNNING
        if background:
            self.start_background_read()
        self._notifier.started("RTDE communication")

    def pause(self) -> None:
        self._raise_worker_error()
        with self._state_lock:
            if self._state is ClientState.PAUSED:
                return
            if self._state is not ClientState.RUNNING:
                raise LifecycleError("RTDE client can only be paused while running")
        self.stop_background_read()
        with self._io_lock:
            self._send_pause()
        with self._state_lock:
            self._state = ClientState.PAUSED

    def _send_start(self) -> None:
        self._stream.write(serialize_start_request())
        for _ in range(MAX_REQUEST_RETRIES):
            try:
                package = self._read_one()
            except URCLTimeoutError:
                continue
            if isinstance(package, ControlPackageStart):
                if not package.accepted:
                    raise UnexpectedResponseError("Robot rejected RTDE start request")
                return
            if isinstance(package, DataPackage) and "timestamp" in package:
                self._prefetched = package
                return
        raise UnexpectedResponseError("Robot did not answer RTDE start request")

    def _send_pause(self) -> None:
        self._stream.write(serialize_pause_request())
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            try:
                package = self._read_one()
            except URCLTimeoutError:
                continue
            if isinstance(package, ControlPackagePause):
                if not package.accepted:
                    raise UnexpectedResponseError("Robot rejected RTDE pause request")
                return
        raise URCLTimeoutError("Robot did not answer RTDE pause request", 5.0)

    def _request_expected(
        self,
        request: bytes,
        expected: type[P],
        description: str,
        *,
        resend_on_unexpected: bool = False,
    ) -> P:
        self._stream.write(request)
        unexpected: RTDEPackage | None = None
        for attempt in range(MAX_REQUEST_RETRIES):
            try:
                package = self._read_one()
            except URCLTimeoutError:
                continue
            if isinstance(package, expected):
                return package
            unexpected = package
            if isinstance(package, TextMessage):
                LOGGER.warning("RTDE text while waiting for %s: %s", description, package.message)
            if resend_on_unexpected and attempt + 1 < MAX_REQUEST_RETRIES:
                self._stream.write(request)
        detail = "" if unexpected is None else f"; last package type {unexpected.package_type!r}"
        raise UnexpectedResponseError(f"Robot did not answer {description}{detail}")

    def _read_one(self) -> RTDEPackage:
        return self._parser.parse(self._stream.read())

    def _read_until_data(self) -> DataPackage:
        while True:
            package = self._read_one()
            if isinstance(package, DataPackage):
                return package
            if isinstance(package, TextMessage):
                LOGGER.info("RTDE text message: %s", package.message)

    def get_data_package(self, timeout: float | None = None) -> DataPackage | None:
        """Return the latest background package, or ``None`` on polling timeout."""
        self._raise_worker_error()
        if self._reconnecting.is_set():
            return None
        if not self._background_running.is_set():
            raise LifecycleError("RTDE background reading is not running")
        with self._latest_condition:
            if not self._new_latest:
                self._latest_condition.wait_for(
                    lambda: (
                        self._new_latest
                        or not self._background_running.is_set()
                        or self._worker_error is not None
                    ),
                    timeout=timeout,
                )
            self._raise_worker_error()
            if not self._new_latest or self._latest is None:
                return None
            result = self._latest.copy()
            self._new_latest = False
            return result

    get_latest = get_data_package

    def read_data_package(self) -> DataPackage:
        """Block until the next data package when background reading is disabled."""
        self._raise_worker_error()
        if self._background_running.is_set():
            raise LifecycleError("Blocking and background RTDE reads are mutually exclusive")
        if self.state is not ClientState.RUNNING:
            raise LifecycleError("RTDE client is not running")
        try:
            while True:
                try:
                    with self._io_lock:
                        return self._read_until_data()
                except URCLTimeoutError as exc:
                    self._raise_worker_error()
                    if self._stop.is_set():
                        raise URCLConnectionError(
                            "RTDE client was closed while waiting for data"
                        ) from exc
        except URCLConnectionError:
            self._schedule_reconnect(background_was_running=False)
            raise

    get_data_package_blocking = read_data_package

    def start_background_read(self) -> None:
        self._raise_worker_error()
        if self.state is not ClientState.RUNNING:
            raise LifecycleError("RTDE client must be running before background reading starts")
        if self._background_running.is_set():
            return
        self._background_running.set()
        self._background_thread = threading.Thread(
            target=self._background_loop,
            name="rtde-reader",
            daemon=True,
        )
        self._background_thread.start()

    def stop_background_read(self) -> None:
        self._background_running.clear()
        with self._latest_condition:
            self._latest_condition.notify_all()
        thread = self._background_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)

    def _background_loop(self) -> None:
        self._try_fifo()
        try:
            while self._background_running.is_set() and not self._stop.is_set():
                try:
                    with self._io_lock:
                        package = self._read_one()
                except URCLTimeoutError:
                    continue
                if isinstance(package, DataPackage):
                    with self._latest_condition:
                        self._latest = package
                        self._new_latest = True
                        self._latest_condition.notify_all()
                elif isinstance(package, TextMessage):
                    LOGGER.info("RTDE text message: %s", package.message)
        except URCLConnectionError:
            if not self._stop.is_set():
                self._schedule_reconnect(background_was_running=True)
        except BaseException as exc:
            self._store_worker_error("RTDE reader", exc)
        finally:
            self._background_running.clear()
            with self._latest_condition:
                self._latest_condition.notify_all()

    def _schedule_reconnect(self, *, background_was_running: bool) -> None:
        if self._stop.is_set() or self._reconnecting.is_set():
            return
        self._reconnecting.set()
        self._reconnect_thread = threading.Thread(
            target=self._reconnect,
            args=(background_was_running,),
            name="rtde-reconnect",
            daemon=True,
        )
        self._reconnect_thread.start()

    def _reconnect(self, background_was_running: bool) -> None:
        with self._reconnect_lock:
            prior_state = self.state
            with self._state_lock:
                self._state = ClientState.CONNECTION_LOST
            self._background_running.clear()
            with self._latest_condition:
                self._latest = None
                self._new_latest = False
                self._latest_condition.notify_all()
            self._disconnect_transport()
            last_error: BaseException | None = None
            try:
                initialization_failures = 0
                while initialization_failures < self._max_initialization_attempts:
                    if self._stop.is_set():
                        return
                    try:
                        self._setup_communication()
                        with self._state_lock:
                            self._state = ClientState.INITIALIZED
                        if prior_state is ClientState.RUNNING:
                            with self._io_lock:
                                self._send_start()
                            with self._state_lock:
                                self._state = ClientState.RUNNING
                            if background_was_running:
                                self.start_background_read()
                        elif prior_state is ClientState.PAUSED:
                            with self._state_lock:
                                self._state = ClientState.PAUSED
                        return
                    except (
                        URCLConnectionError,
                        ProtocolError,
                        RTDEInputConflictError,
                        _RetryHandshake,
                    ) as exc:
                        last_error = exc
                        transport_was_connected = self._stream.is_connected
                        self._disconnect_transport()
                        with self._state_lock:
                            self._state = ClientState.CONNECTION_LOST
                        if transport_was_connected:
                            initialization_failures += 1
                        if (
                            initialization_failures < self._max_initialization_attempts
                            and self._stop.wait(self._initialization_timeout)
                        ):
                            return
                raise URCLConnectionError(
                    "RTDE reconnect attempts were exhausted"
                ) from last_error
            except BaseException as exc:
                if not self._stop.is_set():
                    self._store_worker_error("RTDE reconnect", exc)
                with self._state_lock:
                    self._state = ClientState.UNINITIALIZED
            finally:
                self._reconnecting.clear()

    def _disconnect_transport(self) -> None:
        self._writer.close()
        self._stream.close()

    def close(self) -> None:
        self._stop.set()
        self.stop_background_read()
        self._writer.close()
        self._stream.close()
        thread = self._reconnect_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        with self._state_lock:
            was_active = self._state is not ClientState.UNINITIALIZED
            self._state = ClientState.UNINITIALIZED
        if was_active:
            self._notifier.stopped("RTDE communication")

    disconnect = close

    def _store_worker_error(self, worker: str, cause: BaseException) -> None:
        with self._worker_error_lock:
            if self._worker_error is None:
                self._worker_error = WorkerThreadError(worker, cause)
        with self._latest_condition:
            self._latest_condition.notify_all()

    def _raise_worker_error(self) -> None:
        with self._worker_error_lock:
            error = self._worker_error
        if error is not None:
            raise error from error.cause

    @staticmethod
    def _validate_interval(value: float, name: str) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise TypeError(f"{name} must be a real number")
        normalized = float(value)
        if not math.isfinite(normalized) or normalized < 0:
            raise ValueError(f"{name} must be finite and non-negative")
        return normalized

    @staticmethod
    def _try_fifo() -> None:
        try:
            priority = os.sched_get_priority_max(os.SCHED_FIFO)
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(priority))
        except (AttributeError, OSError, PermissionError) as exc:
            LOGGER.warning("Could not enable FIFO scheduling for RTDE reader: %s", exc)

    def __enter__(self) -> RTDEClient:
        self.connect()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
