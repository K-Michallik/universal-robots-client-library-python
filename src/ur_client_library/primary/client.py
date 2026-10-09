"""Stateful Primary-interface client with injectable transport composition."""

from __future__ import annotations

import re
import socket
import threading
import time
from collections.abc import Callable
from contextlib import suppress
from typing import Protocol, cast

from ur_client_library.exceptions import (
    LifecycleError,
    ReadOnlyInterfaceError,
    RobotErrorCodeError,
    RobotModeError,
    RobotRuntimeError,
    SafetyModeError,
    ScriptSyntaxError,
    StreamNotConnectedError,
    URCLConnectionError,
    URCLTimeoutError,
    WorkerThreadError,
)
from ur_client_library.helpers import robot_series_from_type_and_version
from ur_client_library.primary.consumer import PrimaryConsumer
from ur_client_library.primary.packages import (
    UR_PRIMARY_PORT,
    ConfigurationData,
    ErrorCode,
    KeyMessage,
    KinematicsInfo,
    MasterboardData,
    PrimaryPackage,
    RobotModeData,
    RuntimeExceptionMessage,
    SafetyModeMessage,
    ScriptInfo,
    ScriptType,
)
from ur_client_library.primary.parser import PrimaryParser
from ur_client_library.ur.datatypes import (
    ReportLevel,
    RobotMode,
    RobotSeries,
    RobotType,
    SafetyMode,
)
from ur_client_library.ur.version_information import VersionInformation

_SCRIPT_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_ALLOWED_SAFETY_MODES = (
    SafetyMode.NORMAL,
    SafetyMode.REDUCED,
    SafetyMode.RECOVERY,
    SafetyMode.UNDEFINED_SAFETY_MODE,
)
_FATAL_LEVELS = (ReportLevel.VIOLATION, ReportLevel.FAULT, ReportLevel.CRITICAL_FAULT)


class PrimaryTransport(Protocol):
    """Minimum transport surface used by :class:`PrimaryClient`."""

    def connect(self) -> object: ...

    def close(self) -> object: ...

    def read(self, size: int) -> bytes: ...

    def write(self, data: bytes) -> object: ...


class _SocketTransport:
    def __init__(self, host: str, port: int, timeout: float) -> None:
        self._host = host
        self._port = port
        self._timeout = timeout
        self._socket: socket.socket | None = None
        self._write_lock = threading.Lock()

    def connect(self) -> None:
        self.close()
        stream = socket.create_connection((self._host, self._port), self._timeout)
        stream.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        stream.settimeout(0.2)
        self._socket = stream

    def read(self, size: int) -> bytes:
        if self._socket is None:
            raise OSError("socket is not connected")
        return self._socket.recv(size)

    def write(self, data: bytes) -> None:
        if self._socket is None:
            raise OSError("socket is not connected")
        with self._write_lock:
            self._socket.sendall(data)

    def close(self) -> None:
        stream, self._socket = self._socket, None
        if stream is not None:
            with suppress(OSError):
                stream.shutdown(socket.SHUT_RDWR)
            stream.close()


class PrimaryClient:
    """Client for Primary state, messages, commands, and script feedback.

    Construction performs no network I/O. ``connect()`` opens the transport and
    ``start()`` launches the reader. A custom transport can be injected for
    testing or integration with the package's shared communication layer.
    """

    def __init__(
        self,
        robot_ip: str,
        *,
        port: int = UR_PRIMARY_PORT,
        strict: bool = False,
        transport: PrimaryTransport | object | None = None,
        transport_factory: Callable[[], PrimaryTransport | object] | None = None,
        connect_timeout: float = 5.0,
    ) -> None:
        if transport is not None and transport_factory is not None:
            raise ValueError("Specify either transport or transport_factory, not both")
        if not robot_ip:
            raise ValueError("robot_ip must not be empty")
        if not 0 < port <= 65535:
            raise ValueError("port must be between 1 and 65535")
        if connect_timeout <= 0:
            raise ValueError("connect_timeout must be positive")
        self._robot_ip = robot_ip
        self._port = port
        self._parser = PrimaryParser(strict=strict)
        self._consumer = PrimaryConsumer()
        self._transport = transport
        self._transport_factory = transport_factory or (
            lambda: _SocketTransport(robot_ip, port, connect_timeout)
        )
        self._state_lock = threading.RLock()
        self._write_lock = threading.Lock()
        self._execution_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._reader: threading.Thread | None = None
        self._connected = False
        self._running = False
        self._worker_failure: WorkerThreadError | None = None

    def connect(self) -> None:
        with self._state_lock:
            if self._connected:
                return
            if self._transport is None:
                self._transport = self._transport_factory()
            try:
                result = self._call_transport("connect")
            except Exception as exc:
                raise URCLConnectionError(
                    f"Could not connect to Primary interface at {self._robot_ip}:{self._port}"
                ) from exc
            if result is False:
                raise URCLConnectionError(
                    f"Could not connect to Primary interface at {self._robot_ip}:{self._port}"
                )
            self._stop_event.clear()
            self._worker_failure = None
            self._connected = True

    def start(self) -> None:
        with self._state_lock:
            if not self._connected:
                raise LifecycleError("PrimaryClient must be connected before it is started")
            if self._running:
                return
            self._stop_event.clear()
            self._worker_failure = None
            self._reader = threading.Thread(
                target=self._reader_loop,
                name="urcl-primary-reader",
                daemon=True,
            )
            self._running = True
            self._reader.start()

    def close(self) -> None:
        with self._state_lock:
            if not self._connected and not self._running:
                return
            self._stop_event.set()
            reader = self._reader
            transport = self._transport
            self._running = False
            self._connected = False
        if transport is not None:
            try:
                close = getattr(transport, "close", None)
                if callable(close):
                    close()
            except Exception:
                pass
        if reader is not None and reader is not threading.current_thread():
            reader.join(timeout=2.0)
        with self._state_lock:
            if self._reader is reader:
                self._reader = None

    def __enter__(self) -> PrimaryClient:
        self.connect()
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    @property
    def is_connected(self) -> bool:
        with self._state_lock:
            return self._connected

    @property
    def is_running(self) -> bool:
        with self._state_lock:
            return self._running

    @property
    def robot_mode_data(self) -> RobotModeData | None:
        return self._consumer.robot_mode_data

    @property
    def robot_mode(self) -> RobotMode:
        data = self.robot_mode_data
        if data is None or not isinstance(data.robot_mode, RobotMode):
            return RobotMode.UNKNOWN
        return data.robot_mode

    @property
    def configuration_data(self) -> ConfigurationData | None:
        return self._consumer.configuration_data

    @property
    def kinematics_info(self) -> KinematicsInfo | None:
        return self._consumer.kinematics_info

    @property
    def masterboard_data(self) -> MasterboardData | None:
        return self._consumer.masterboard_data

    @property
    def safety_mode_message(self) -> SafetyModeMessage | None:
        return self._consumer.safety_mode_message

    @property
    def safety_mode(self) -> SafetyMode:
        message = self.safety_mode_message
        if message is None or not isinstance(message.safety_mode, SafetyMode):
            return SafetyMode.UNDEFINED_SAFETY_MODE
        return message.safety_mode

    @property
    def version(self) -> VersionInformation | None:
        return self._consumer.version

    @property
    def robot_type(self) -> RobotType:
        data = self.configuration_data
        if data is None:
            return RobotType.UNDEFINED
        try:
            return RobotType(data.robot_type)
        except ValueError:
            return RobotType.UNDEFINED

    @property
    def robot_series(self) -> RobotSeries:
        version = self.version
        robot_type = self.robot_type
        if version is None or robot_type == RobotType.UNDEFINED:
            return RobotSeries.UNDEFINED
        return robot_series_from_type_and_version(robot_type, version)

    @property
    def is_protective_stopped(self) -> bool:
        data = self.robot_mode_data
        if data is None:
            raise LifecycleError("Robot mode data has not been received yet")
        return data.is_protective_stopped

    def wait_for_version(self, timeout: float = 2.0) -> VersionInformation:
        if not self._consumer.wait_for(lambda: self.version is not None, timeout):
            raise URCLTimeoutError("No VersionMessage received within timeout", timeout)
        version = self.version
        assert version is not None
        return version

    def check_calibration(self, checksum: str, *, timeout: float | None = None) -> bool:
        if not self._consumer.wait_for(lambda: self.kinematics_info is not None, timeout):
            raise URCLTimeoutError("No KinematicsInfo received within timeout", timeout)
        info = self.kinematics_info
        assert info is not None
        return info.calibration_hash == checksum

    def safety_mode_allows_execution(self) -> bool:
        return self.safety_mode in _ALLOWED_SAFETY_MODES

    def process_package(self, package: PrimaryPackage) -> None:
        """Consume an already parsed package (primarily for composed pipelines)."""
        self._consumer.consume(package)

    def subscribe(
        self, callback: Callable[[PrimaryPackage], object]
    ) -> Callable[[], None]:
        return self._consumer.subscribe(callback)

    def add_primary_consumer(self, consumer: object) -> Callable[[], None]:
        callback: Callable[[PrimaryPackage], object]
        if callable(consumer):
            callback = cast(Callable[[PrimaryPackage], object], consumer)
        else:
            method = getattr(consumer, "consume", None)
            if method is None or not callable(method):
                raise TypeError("consumer must be callable or provide consume(package)")
            callback = cast(Callable[[PrimaryPackage], object], method)
        return self.subscribe(callback)

    def subscribe_errors(self, callback: Callable[[ErrorCode], object]) -> Callable[[], None]:
        return self._consumer.subscribe_errors(callback)

    def subscribe_key_messages(
        self, callback: Callable[[KeyMessage], object]
    ) -> Callable[[], None]:
        return self._consumer.subscribe_key_messages(callback)

    def subscribe_runtime_exceptions(
        self, callback: Callable[[RuntimeExceptionMessage], object]
    ) -> Callable[[], None]:
        return self._consumer.subscribe_runtime_exceptions(callback)

    def pop_error_codes(self) -> tuple[ErrorCode, ...]:
        self._raise_worker_failure()
        return self._consumer.pop_error_codes()

    def pop_key_messages(self) -> tuple[KeyMessage, ...]:
        self._raise_worker_failure()
        return self._consumer.pop_key_messages()

    def pop_runtime_exceptions(self) -> tuple[RuntimeExceptionMessage, ...]:
        self._raise_worker_failure()
        return self._consumer.pop_runtime_exceptions()

    def send_script(self, program: str) -> None:
        """Upload URScript, appending the newline required by the controller."""
        if not isinstance(program, str):
            raise TypeError("program must be a string")
        self._require_connected()
        self._raise_worker_failure()
        data = (program + "\n").encode()
        failure: BaseException | None = None
        for attempt in range(2):
            try:
                with self._write_lock:
                    result = self._write_transport(data)
                if result is not False and not (
                    isinstance(result, int)
                    and not isinstance(result, bool)
                    and result != len(data)
                ):
                    return
                failure = StreamNotConnectedError(
                    f"Could not send complete program to the robot ({result!r})"
                )
            except Exception as exc:
                failure = exc
            if attempt == 0:
                try:
                    self._restart()
                except Exception as exc:
                    failure = exc
                    break
        raise StreamNotConnectedError("Could not send program to the robot") from failure

    def send_script_blocking(
        self,
        program: str,
        script_name: str = "",
        *,
        start_timeout: float = 1.0,
        fail_on_warnings: bool = True,
        retry_on_readonly_interface: bool = True,
    ) -> None:
        if start_timeout < 0:
            raise ValueError("start_timeout must be non-negative")
        script_info = self.prepare_script(program, script_name)
        with self._execution_lock:
            if not self._consumer.wait_for(
                lambda: self.robot_mode != RobotMode.UNKNOWN, 1.0
            ):
                raise URCLTimeoutError("Robot mode not received within timeout", 1.0)
            if self.robot_mode != RobotMode.RUNNING:
                raise RobotModeError(
                    "Script execution via primary interface",
                    RobotMode.RUNNING,
                    self.robot_mode,
                )
            if not self.safety_mode_allows_execution():
                raise SafetyModeError(
                    "Script execution via primary interface",
                    _ALLOWED_SAFETY_MODES,
                    self.safety_mode,
                )
            try:
                self._send_script_monitor_execution(
                    script_info,
                    start_timeout=start_timeout,
                    fail_on_warnings=fail_on_warnings,
                )
            except ReadOnlyInterfaceError:
                if not retry_on_readonly_interface:
                    raise
                self._restart()
                self._send_script_monitor_execution(
                    script_info,
                    start_timeout=start_timeout,
                    fail_on_warnings=fail_on_warnings,
                )

    @staticmethod
    def prepare_script(script: str, script_name: str = "") -> ScriptInfo:
        if not isinstance(script, str) or not isinstance(script_name, str):
            raise TypeError("script and script_name must be strings")
        stripped: list[str] = []
        for line in script.split("\n"):
            content = line.lstrip()
            if content and not content.startswith("#"):
                stripped.append(line)
        if not stripped:
            raise ScriptSyntaxError("Script is empty after stripping comments and whitespace")

        actual_name = script_name or f"script_{int(time.time() * 1000)}"
        script_type = ScriptType.DEF
        first = stripped[0]
        is_definition = first.startswith("def ") or first.startswith("sec ")
        if not is_definition:
            actual_name = actual_name[:31]
            stripped = [
                f"def {actual_name}():",
                *(f"  {line}" for line in stripped),
                "end",
            ]
        else:
            name_end = first.find("(")
            if name_end < 0:
                raise ScriptSyntaxError(
                    "Function definition detected, but no '(' was found"
                )
            name_in_script = first[4:name_end]
            script_type = ScriptType.DEF if first.startswith("def ") else ScriptType.SEC
            actual_name = name_in_script[:31]
            if actual_name != name_in_script:
                stripped[0] = first[:4] + actual_name + first[name_end:]

        if not _SCRIPT_NAME.fullmatch(actual_name):
            raise ScriptSyntaxError(
                f"Invalid script name {actual_name!r}; use letters, numbers, and underscores, "
                "starting with a letter or underscore"
            )
        if not stripped[-1].startswith("end"):
            raise ScriptSyntaxError("Defined script has no final 'end' term")
        return ScriptInfo(actual_name, "".join(f"{line}\n" for line in stripped), script_type)

    def command_power_on(self, *, validate: bool = True, timeout: float = 30.0) -> None:
        self.send_script("power on")
        if validate:
            self._wait_for_state(
                lambda: self.robot_mode in (RobotMode.IDLE, RobotMode.RUNNING),
                timeout,
                "Robot did not power on within the given timeout",
            )

    def command_power_off(self, *, validate: bool = True, timeout: float = 30.0) -> None:
        self.send_script("power off")
        if validate:
            self._wait_for_state(
                lambda: self.robot_mode == RobotMode.POWER_OFF,
                timeout,
                "Robot did not power off within the given timeout",
            )

    def command_brake_release(
        self, *, validate: bool = True, timeout: float = 30.0
    ) -> None:
        self.send_script("set robotmode run")
        if validate:
            self._wait_for_state(
                lambda: self.robot_mode == RobotMode.RUNNING,
                timeout,
                "Robot did not release the brakes within the given timeout",
            )

    def command_unlock_protective_stop(
        self, *, validate: bool = True, timeout: float = 5.0
    ) -> None:
        self.send_script("set unlock protective stop")
        if validate:
            self._wait_for_state(
                lambda: self.robot_mode_data is not None
                and not self.robot_mode_data.is_protective_stopped,
                timeout,
                "Robot did not unlock the protective stop within the given timeout",
            )

    def command_stop(self, *, validate: bool = True, timeout: float = 2.0) -> None:
        if self.robot_mode_data is None:
            raise LifecycleError("Cannot stop a program before robot mode data is received")
        self.send_script("stop program")
        if validate:
            self._wait_for_state(
                lambda: self.robot_mode_data is not None
                and not self.robot_mode_data.is_program_running
                and not self.robot_mode_data.is_program_paused,
                timeout,
                "Robot did not stop the program within the given timeout",
            )

    def _send_script_monitor_execution(
        self,
        script_info: ScriptInfo,
        *,
        start_timeout: float,
        fail_on_warnings: bool,
    ) -> None:
        self._consumer.clear_script_feedback()
        self.send_script(script_info.script_code)
        if script_info.script_type == ScriptType.SEC:
            return

        sent_at = time.monotonic()
        script_started = start_timeout == 0
        stopped_at: float | None = None
        while True:
            self._raise_worker_failure()
            runtime_exceptions = self._consumer.pop_runtime_exceptions()
            if runtime_exceptions:
                raise RobotRuntimeError(
                    self._format_runtime_error(script_info, runtime_exceptions[-1])
                )

            errors = self._consumer.pop_error_codes()
            if errors:
                read_only = any(error.message_code == 210 for error in errors)
                fatal = any(error.report_level in _FATAL_LEVELS for error in errors)
                warning = any(error.report_level == ReportLevel.WARNING for error in errors)
                reportable = [
                    error
                    for error in errors
                    if error.report_level in _FATAL_LEVELS
                    or (fail_on_warnings and error.report_level == ReportLevel.WARNING)
                    or error.message_code == 210
                ]
                details = "\n".join(self._format_error_code(error) for error in reportable)
                if read_only:
                    raise ReadOnlyInterfaceError(
                        "Script cannot execute because the Primary interface is read-only"
                    )
                if fatal:
                    self.command_stop()
                    raise RobotErrorCodeError(
                        f"Robot error codes received during script execution:\n{details}"
                    )
                if warning and fail_on_warnings:
                    raise RobotErrorCodeError(
                        f"Robot warnings received during script execution:\n{details}"
                    )

            for message in self._consumer.pop_key_messages():
                if (
                    message.title == "PROGRAM_XXX_STOPPED"
                    and message.text == script_info.script_name
                ):
                    if stopped_at is None:
                        stopped_at = time.monotonic()
                        script_started = True
                elif (
                    not script_started
                    and message.title == "PROGRAM_XXX_STARTED"
                    and message.text == script_info.script_name
                ):
                    script_started = True

            now = time.monotonic()
            if stopped_at is not None and now - stopped_at >= 0.1:
                return
            if stopped_at is None and not script_started and now - sent_at > start_timeout:
                raise URCLTimeoutError(
                    f"Script {script_info.script_name!r} not started within timeout",
                    start_timeout,
                )
            time.sleep(0.01)

    @staticmethod
    def _format_runtime_error(
        script_info: ScriptInfo, exception: RuntimeExceptionMessage
    ) -> str:
        lines = [
            "Runtime exception occurred during script execution.",
            f"Runtime exception type: {exception.text}",
            f"Exception occurred at line {exception.line_number}, "
            f"column {exception.column_number}",
        ]
        script_lines = script_info.script_code.splitlines()
        width = len(str(len(script_lines)))
        for index, line in enumerate(script_lines, 1):
            if line:
                lines.append(f"{index:>{width}}: {line}")
            if index == exception.line_number and exception.column_number > 0:
                lines.append(
                    " " * (width + 2 + exception.column_number - 1) + "^<--- here"
                )
        return "\n".join(lines)

    @classmethod
    def _format_error_code(cls, error: ErrorCode) -> str:
        controller_text = f" [controller: {error.text}]" if error.text else ""
        return (
            f"{error.description}{controller_text} "
            f"(severity {cls._level_name(error.report_level)})"
        )

    @staticmethod
    def _level_name(level: ReportLevel | int) -> str:
        return level.name if isinstance(level, ReportLevel) else str(level)

    def _wait_for_state(
        self, predicate: Callable[[], bool], timeout: float, message: str
    ) -> None:
        if timeout < 0:
            raise ValueError("timeout must be non-negative")
        if not self._consumer.wait_for(predicate, timeout):
            raise URCLTimeoutError(message, timeout)

    def _restart(self) -> None:
        was_running = self.is_running
        self.close()
        self.connect()
        if was_running:
            self.start()

    def _reader_loop(self) -> None:
        try:
            while not self._stop_event.is_set():
                header = self._read_exact(4)
                package_size = int.from_bytes(header, "big", signed=True)
                if package_size < 5:
                    raise ValueError(f"Invalid Primary package size {package_size}")
                frame = header + self._read_exact(package_size - 4)
                for package in self._parser.parse(frame):
                    self._consumer.consume(package)
        except Exception as exc:
            if not self._stop_event.is_set():
                failure = (
                    exc
                    if isinstance(exc, WorkerThreadError)
                    else WorkerThreadError("Primary reader", exc)
                )
                with self._state_lock:
                    self._worker_failure = failure
                    self._running = False
        finally:
            with self._state_lock:
                if threading.current_thread() is self._reader:
                    self._running = False

    def _read_exact(self, size: int) -> bytes:
        result = bytearray()
        while len(result) < size:
            if self._stop_event.is_set():
                raise URCLConnectionError("Primary reader stopped")
            try:
                chunk = self._read_transport(size - len(result))
            except TimeoutError:
                continue
            if not chunk:
                raise URCLConnectionError("Primary connection closed while reading")
            result.extend(chunk)
        return bytes(result)

    def _read_transport(self, size: int) -> bytes:
        transport = self._transport
        if transport is None:
            raise StreamNotConnectedError("Primary transport is not configured")
        read_exact = getattr(transport, "read_exact", None)
        if callable(read_exact):
            return cast(bytes, read_exact(size))
        read = getattr(transport, "read", None)
        if callable(read):
            return cast(bytes, read(size))
        recv = getattr(transport, "recv", None)
        if callable(recv):
            return cast(bytes, recv(size))
        raise TypeError("Primary transport must provide read(), read_exact(), or recv()")

    def _write_transport(self, data: bytes) -> object:
        transport = self._transport
        if transport is None:
            raise StreamNotConnectedError("Primary transport is not configured")
        write = getattr(transport, "write", None)
        if callable(write):
            return write(data)
        sendall = getattr(transport, "sendall", None)
        if callable(sendall):
            return sendall(data)
        send = getattr(transport, "send", None)
        if callable(send):
            return send(data)
        raise TypeError("Primary transport must provide write(), sendall(), or send()")

    def _call_transport(self, method_name: str) -> object:
        transport = self._transport
        if transport is None:
            raise StreamNotConnectedError("Primary transport is not configured")
        method = getattr(transport, method_name, None)
        if not callable(method):
            raise TypeError(f"Primary transport must provide {method_name}()")
        return method()

    def _require_connected(self) -> None:
        if not self.is_connected:
            raise LifecycleError("PrimaryClient is not connected")

    def _raise_worker_failure(self) -> None:
        with self._state_lock:
            failure = self._worker_failure
        if failure is not None:
            raise failure
