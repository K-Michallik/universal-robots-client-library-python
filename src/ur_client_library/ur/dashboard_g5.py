"""Persistent TCP dashboard client for CB3 and PolyScope 5 controllers."""

from __future__ import annotations

import errno
import re
import socket
import threading
import time
from collections.abc import Callable
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from ur_client_library.exceptions import (
    DashboardCommandError,
    LifecycleError,
    ProtocolError,
    StreamNotConnectedError,
    URCLConnectionError,
    URCLTimeoutError,
    VersionMismatchError,
)
from ur_client_library.ur.dashboard_client import (
    DashboardResponse,
    ProgramSaveState,
    ProgramState,
)
from ur_client_library.ur.version_information import VersionInformation


@dataclass(frozen=True, slots=True)
class _DashboardCommand:
    command: str
    response_pattern: str
    minimum_cb3: VersionInformation
    minimum_e_series: VersionInformation


def _version(value: str) -> VersionInformation:
    return VersionInformation.from_string(value)


_COMMANDS: dict[str, _DashboardCommand] = {
    "load": _DashboardCommand(
        "load <program.urp>",
        r"(?:Loading program: ).*(?: <program.urp> ).*",
        _version("1.4"),
        _version("5.0.0"),
    ),
    "play": _DashboardCommand("play", "Starting program", _version("1.4"), _version("5.0.0")),
    "stop": _DashboardCommand("stop", "Stopped", _version("1.4"), _version("5.0.0")),
    "pause": _DashboardCommand("pause", "Pausing program", _version("1.4"), _version("5.0.0")),
    "quit": _DashboardCommand("quit", "Disconnected", _version("1.4"), _version("5.0.0")),
    "shutdown": _DashboardCommand("shutdown", "Shutting down", _version("1.4"), _version("5.0.0")),
    "unlock protective stop": _DashboardCommand(
        "unlock protective stop", "Protective stop releasing", _version("3.1"), _version("5.0.0")
    ),
    "running": _DashboardCommand(
        "running", r"(?:Program running: ).*", _version("1.6"), _version("5.0.0")
    ),
    "robotmode": _DashboardCommand(
        "robotmode", r"(?:Robotmode: ).*", _version("1.6"), _version("5.0.0")
    ),
    "get loaded program": _DashboardCommand(
        "get loaded program", r"(?:Loaded program: ).*", _version("1.6"), _version("5.0.0")
    ),
    "popup": _DashboardCommand(
        "popup <popup_text>", "showing popup", _version("1.6"), _version("5.0.0")
    ),
    "close popup": _DashboardCommand(
        "close popup", "closing popup", _version("1.6"), _version("5.0.0")
    ),
    "addToLog": _DashboardCommand(
        "addToLog <log_text>", "Added log message", _version("1.8"), _version("5.0.0")
    ),
    "isProgramSaved": _DashboardCommand(
        "isProgramSaved", r"(?:true|false).*", _version("1.8"), _version("5.0.0")
    ),
    "programState": _DashboardCommand(
        "programState", r"(STOPPED|PLAYING|PAUSED) .*", _version("1.8"), _version("5.0.0")
    ),
    "PolyscopeVersion": _DashboardCommand(
        "PolyscopeVersion", r"(?:URSoftware ).*", _version("1.8"), _version("5.0.0")
    ),
    "version": _DashboardCommand(
        "version", r"(\d\.\d\.\d\.\d)", _version("5.0.0"), _version("5.0.0")
    ),
    "setUserRole": _DashboardCommand(
        "setUserRole <role>",
        "Setting user role: <role>",
        _version("3.1.0"),
        _version("5.99.0"),
    ),
    "getUserRole": _DashboardCommand(
        "getUserRole",
        r"(PROGRAMMER|OPERATOR|NONE|LOCKED|RESTRICTED)",
        _version("3.1.0"),
        _version("5.99.0"),
    ),
    "set operational mode": _DashboardCommand(
        "set operational mode <operational_mode>",
        r"Operational mode (MANUAL|AUTOMATIC) is set",
        _version("3.99.0"),
        _version("5.0.0"),
    ),
    "get operational mode": _DashboardCommand(
        "get operational mode",
        r"(NONE|MANUAL|AUTOMATIC)",
        _version("5.6.0"),
        _version("5.6.0"),
    ),
    "clear operational mode": _DashboardCommand(
        "clear operational mode",
        r"(?:No longer controlling the operational mode.).*",
        _version("5.0.0"),
        _version("5.0.0"),
    ),
    "power on": _DashboardCommand(
        "power on", "Powering on", _version("3.0.0"), _version("5.0.0")
    ),
    "power off": _DashboardCommand(
        "power off", "Powering off", _version("3.0.0"), _version("5.0.0")
    ),
    "brake release": _DashboardCommand(
        "brake release", "Brake releasing", _version("3.0.0"), _version("5.0.0")
    ),
    "safetymode": _DashboardCommand(
        "safetymode", r"(?:Safetymode: ).*", _version("3.0"), _version("5.0.0")
    ),
    "safetystatus": _DashboardCommand(
        "safetystatus", r"(?:Safetystatus: ).*", _version("5.4.0"), _version("5.4.0")
    ),
    "close safety popup": _DashboardCommand(
        "close safety popup", "closing safety popup", _version("3.1"), _version("5.0.0")
    ),
    "load installation": _DashboardCommand(
        "load installation <default.installation>",
        r"(?:Loading installation: ).*(?: <default.installation> ).*",
        _version("3.2"),
        _version("5.0.0"),
    ),
    "restart safety": _DashboardCommand(
        "restart safety", "Restarting safety", _version("3.7"), _version("5.1.0")
    ),
    "is in remote control": _DashboardCommand(
        "is in remote control", r"(?:true|false)", _version("5.6.0"), _version("5.6.0")
    ),
    "get serial number": _DashboardCommand(
        "get serial number", r"(?:20).*", _version("3.12"), _version("5.6.0")
    ),
    "get robot model": _DashboardCommand(
        "get robot model", r"(?:UR).*", _version("3.12"), _version("5.6.0")
    ),
    "generate flight report": _DashboardCommand(
        "generate flight report <report_type>",
        r"(?:Generating flight report: ).*(?: <report_type> ).*",
        _version("3.13"),
        _version("5.8.0"),
    ),
    "generate support file": _DashboardCommand(
        "generate support file <dir_path>",
        r"(?:Generating support file: ).*(?: <dir_path> ).*",
        _version("3.13"),
        _version("5.8.0"),
    ),
    "saveLog": _DashboardCommand(
        "saveLog", "Log saved to disk", _version("3.13"), _version("5.8.0")
    ),
}


def _default_socket_factory() -> socket.socket:
    return socket.socket(socket.AF_INET, socket.SOCK_STREAM)


def _validate_timeout(value: float, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a number")
    result = float(value)
    if result <= 0:
        raise ValueError(f"{name} must be greater than zero")
    return result


class DashboardClientG5:
    """Newline-delimited dashboard protocol on TCP port 29999."""

    DASHBOARD_SERVER_PORT = 29999

    def __init__(
        self,
        host: str,
        *,
        port: int = DASHBOARD_SERVER_PORT,
        receive_timeout: float = 1.0,
        socket_factory: Callable[[], Any] | None = None,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        if not isinstance(host, str) or not host.strip():
            raise ValueError("host must be a non-empty string")
        if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
            raise ValueError("port must be an integer from 1 through 65535")
        self._host = host
        self._port = port
        self._receive_timeout = _validate_timeout(receive_timeout, "receive_timeout")
        self._socket_factory = socket_factory or _default_socket_factory
        self._sleep = sleep
        self._socket: Any | None = None
        self._connected = False
        self._polyscope_version = VersionInformation()
        self._request_lock = threading.Lock()

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def polyscope_version(self) -> VersionInformation:
        if self._polyscope_version.is_empty:
            raise LifecycleError("PolyScope version is unavailable before connect() succeeds")
        return self._polyscope_version

    @property
    def version(self) -> VersionInformation:
        return self.polyscope_version

    @property
    def receive_timeout(self) -> float:
        return self._receive_timeout

    @receive_timeout.setter
    def receive_timeout(self, timeout: float) -> None:
        self._receive_timeout = _validate_timeout(timeout, "receive_timeout")
        if self._socket is not None and self._connected:
            self._socket.settimeout(self._receive_timeout)

    def set_receive_timeout(self, timeout: float) -> None:
        self.receive_timeout = timeout

    def get_configured_receive_timeout(self) -> float:
        return self.receive_timeout

    def connect(
        self,
        *,
        max_attempts: int = 0,
        retry_interval: float = 10.0,
    ) -> DashboardClientG5:
        if self._connected:
            raise LifecycleError("dashboard client is already connected")
        if isinstance(max_attempts, bool) or not isinstance(max_attempts, int) or max_attempts < 0:
            raise ValueError("max_attempts must be a non-negative integer")
        if isinstance(retry_interval, bool) or not isinstance(retry_interval, (int, float)):
            raise TypeError("retry_interval must be a number")
        if retry_interval < 0:
            raise ValueError("retry_interval must be non-negative")

        attempts = 0
        last_error: BaseException | None = None
        while max_attempts == 0 or attempts < max_attempts:
            attempts += 1
            candidate = self._socket_factory()
            try:
                candidate.settimeout(10.0)
                candidate.connect((self._host, self._port))
            except OSError as exc:
                last_error = exc
                with suppress(OSError):
                    candidate.close()
                if max_attempts != 0 and attempts >= max_attempts:
                    break
                self._sleep(float(retry_interval))
                continue

            self._socket = candidate
            self._connected = True
            try:
                self._read_line(timeout=10.0)
            except URCLTimeoutError as exc:
                # The dashboard can accept TCP before its greeting is ready.
                # Upstream treats this as a transient startup condition.
                last_error = exc
                self.close()
                if max_attempts != 0 and attempts >= max_attempts:
                    break
                self._sleep(float(retry_interval))
                continue
            finally:
                if self._connected:
                    candidate.settimeout(self._receive_timeout)
            try:
                self._polyscope_version = self._query_polyscope_version()
            except BaseException:
                self.close()
                raise
            return self

        message = (
            f"failed to connect to dashboard server at {self._host}:{self._port} "
            f"after {attempts} attempt{'s' if attempts != 1 else ''}"
        )
        raise URCLConnectionError(message) from last_error

    def close(self) -> None:
        sock, self._socket = self._socket, None
        self._connected = False
        if sock is not None:
            with suppress(OSError):
                sock.close()

    disconnect = close

    def __enter__(self) -> DashboardClientG5:
        return self.connect()

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: object,
    ) -> None:
        self.close()

    def _require_socket(self) -> Any:
        if not self._connected or self._socket is None:
            raise StreamNotConnectedError("dashboard client is not connected")
        return self._socket

    def _read_line(self, *, timeout: float | None = None) -> str:
        sock = self._require_socket()
        configured_timeout = self._receive_timeout if timeout is None else timeout
        result = bytearray()
        while True:
            try:
                chunk = sock.recv(1)
            except TimeoutError as exc:
                raise URCLTimeoutError(
                    "did not receive an answer from the dashboard server in time",
                    configured_timeout,
                ) from exc
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    raise URCLTimeoutError(
                        "did not receive an answer from the dashboard server in time",
                        configured_timeout,
                    ) from exc
                self._connected = False
                raise URCLConnectionError("failed to read from dashboard server") from exc
            if not chunk:
                self._connected = False
                raise URCLConnectionError("dashboard server closed the connection")
            result.extend(chunk)
            if chunk == b"\n":
                break
        try:
            return result.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise ProtocolError("dashboard response is not valid UTF-8") from exc

    def send_and_receive(self, text: str) -> str:
        if not isinstance(text, str):
            raise TypeError("text must be a string")
        if not text:
            raise ValueError("text must not be empty")
        sock = self._require_socket()
        command = text if text.endswith("\n") else f"{text}\n"
        with self._request_lock:
            try:
                sock.sendall(command.encode("utf-8"))
            except TimeoutError as exc:
                raise URCLTimeoutError(
                    "timed out sending a dashboard request", self._receive_timeout
                ) from exc
            except OSError as exc:
                raise URCLConnectionError(
                    "failed to send request to dashboard server"
                ) from exc
            return self._read_line().rstrip()

    @staticmethod
    def _replace_payload(value: str, payload: str) -> str:
        return re.sub(r"<.*>", lambda _match: payload, value)

    def _assert_has_command(self, command: str) -> _DashboardCommand:
        if self._polyscope_version.is_empty:
            raise LifecycleError("PolyScope version is not set; call connect() first")
        descriptor = _COMMANDS.get(command)
        if descriptor is None:
            raise NotImplementedError(f"dashboard command {command!r} is not implemented")
        minimum = (
            descriptor.minimum_e_series
            if self._polyscope_version.is_e_series
            else descriptor.minimum_cb3
        )
        if self._polyscope_version < minimum:
            raise VersionMismatchError(
                f"dashboard command {command!r} is not supported by this controller",
                minimum,
                self._polyscope_version,
            )
        return descriptor

    def send_request(
        self,
        command: str,
        expected: str = "",
        payload: str = "",
    ) -> str:
        descriptor = self._assert_has_command(command)
        wire_command = self._replace_payload(descriptor.command, payload)
        pattern = self._replace_payload(expected or descriptor.response_pattern, payload)
        response = self.send_and_receive(wire_command)
        if re.fullmatch(pattern, response) is None:
            raise DashboardCommandError(
                wire_command,
                f"expected response matching {pattern!r}, received {response!r}",
            )
        return response

    def wait_for_reply(
        self,
        command: str,
        expected: str,
        timeout: float = 30.0,
        *,
        poll_period: float = 0.1,
    ) -> bool:
        if timeout < 0:
            raise ValueError("timeout must be non-negative")
        poll_period = _validate_timeout(poll_period, "poll_period")
        elapsed = 0.0
        while elapsed < timeout:
            response = self.send_and_receive(command)
            if re.fullmatch(expected, response) is not None:
                return True
            self._sleep(poll_period)
            elapsed += poll_period
        return False

    def retry_command(
        self,
        request_command: str,
        request_expected_response: str,
        wait_request: str,
        wait_expected_response: str,
        timeout: float,
        *,
        retry_period: float = 1.0,
    ) -> str:
        retry_period = _validate_timeout(retry_period, "retry_period")
        elapsed = 0.0
        last_response = ""
        while True:
            last_response = self.send_request(
                request_command, request_expected_response
            )
            elapsed += retry_period
            if self.wait_for_reply(
                wait_request,
                wait_expected_response,
                retry_period,
            ):
                return last_response
            if elapsed >= timeout:
                raise DashboardCommandError(
                    request_command,
                    f"expected state was not reached within {timeout:g} seconds; "
                    f"last command response was {last_response!r}",
                )

    @staticmethod
    def _response(message: str, **data: object) -> DashboardResponse:
        return DashboardResponse(message=message, data=data)

    def power_off(self) -> DashboardResponse:
        message = self.send_request("power off")
        if not self.wait_for_reply("robotmode", "Robotmode: POWER_OFF"):
            raise DashboardCommandError(
                "power off", "robot mode did not become POWER_OFF within 30 seconds"
            )
        return self._response(message)

    def power_on(self, timeout: float = 300.0) -> DashboardResponse:
        message = self.retry_command(
            "power on",
            "Powering on",
            "robotmode",
            r"Robotmode: (IDLE|RUNNING)",
            timeout,
        )
        return self._response(message)

    def brake_release(self) -> DashboardResponse:
        message = self.send_request("brake release")
        if not self.wait_for_reply("robotmode", "Robotmode: RUNNING"):
            raise DashboardCommandError(
                "brake release", "robot mode did not become RUNNING within 30 seconds"
            )
        return self._response(message)

    def load_program(self, program_file_name: str) -> DashboardResponse:
        expected = rf"(?:Loading program: ).*(?:{program_file_name}).*"
        message = self.send_request("load", expected, program_file_name)
        expected_state = f"STOPPED {Path(program_file_name).name}"
        if not self.wait_for_reply("programState", expected_state):
            raise DashboardCommandError(
                "load",
                f"program state did not become {expected_state!r} within 30 seconds",
            )
        return self._response(message, program_name=program_file_name)

    def load_installation(self, installation_file_name: str) -> DashboardResponse:
        expected = rf"(?:Loading installation: ).*(?:{installation_file_name}).*"
        message = self.send_request(
            "load installation", expected, installation_file_name
        )
        return self._response(message, installation_name=installation_file_name)

    def play(self) -> DashboardResponse:
        message = self.send_request("play")
        if not self.wait_for_reply("programState", r"(?:PLAYING ).*"):
            raise DashboardCommandError(
                "play", "program state did not become PLAYING within 30 seconds"
            )
        return self._response(message)

    def pause(self) -> DashboardResponse:
        message = self.send_request("pause")
        if not self.wait_for_reply("programState", r"(?:PAUSED ).*"):
            raise DashboardCommandError(
                "pause", "program state did not become PAUSED within 30 seconds"
            )
        return self._response(message)

    def resume(self) -> DashboardResponse:
        raise NotImplementedError(
            "resume is not available on G5; use play() to resume a paused program"
        )

    def stop(self) -> DashboardResponse:
        message = self.send_request("stop")
        if not self.wait_for_reply("programState", r"(?:STOPPED ).*"):
            raise DashboardCommandError(
                "stop", "program state did not become STOPPED within 30 seconds"
            )
        return self._response(message)

    def close_popup(self) -> DashboardResponse:
        return self._response(self.send_request("close popup"))

    def close_safety_popup(self) -> DashboardResponse:
        return self._response(self.send_request("close safety popup"))

    def restart_safety(self) -> DashboardResponse:
        message = self.send_request("restart safety")
        if not self.wait_for_reply("robotmode", "Robotmode: POWER_OFF"):
            raise DashboardCommandError(
                "restart safety", "robot mode did not become POWER_OFF within 30 seconds"
            )
        return self._response(message)

    def unlock_protective_stop(self) -> DashboardResponse:
        message = self.send_request("unlock protective stop")
        if not self.wait_for_reply("robotmode", "Robotmode: RUNNING"):
            raise DashboardCommandError(
                "unlock protective stop",
                "robot mode did not become RUNNING within 30 seconds",
            )
        return self._response(message)

    def shutdown(self) -> DashboardResponse:
        return self._response(self.send_request("shutdown"))

    def quit(self) -> DashboardResponse:
        message = self.send_request("quit")
        self.close()
        return self._response(message)

    def running(self) -> bool:
        response = self.send_request("running")
        match = re.fullmatch(r"Program running: (true|false)", response)
        if match is None:
            raise ProtocolError(f"could not parse running state from {response!r}")
        return match.group(1) == "true"

    def is_program_saved(self) -> ProgramSaveState:
        response = self.send_request("isProgramSaved")
        match = re.search(r"(false|true) (.*\.urp)", response)
        if match is None:
            raise ProtocolError(f"could not parse program save state from {response!r}")
        return ProgramSaveState(match.group(1) == "true", match.group(2))

    def is_in_remote_control(self) -> bool:
        return self.send_request("is in remote control") == "true"

    def popup(self, popup_text: str, popup_title: str = "") -> DashboardResponse:
        # The G5 dashboard has no popup title; it exists for PolyScope X.
        return self._response(self.send_request("popup", payload=popup_text))

    def add_to_log(self, log_text: str) -> DashboardResponse:
        return self._response(self.send_request("addToLog", payload=log_text))

    @staticmethod
    def _parse_version_response(response: str) -> VersionInformation:
        match = re.fullmatch(r"URSoftware ([^ ]+) \(.*", response)
        if match is None:
            raise ProtocolError(f"could not parse PolyScope version from {response!r}")
        return VersionInformation.from_string(match.group(1))

    def _query_polyscope_version(self) -> VersionInformation:
        response = self.send_and_receive("PolyscopeVersion")
        return self._parse_version_response(response)

    def get_polyscope_version(self) -> VersionInformation:
        response = self.send_request("PolyscopeVersion")
        self._polyscope_version = self._parse_version_response(response)
        return self._polyscope_version

    def get_robot_model(self) -> str:
        return self.send_request("get robot model")

    def get_serial_number(self) -> str:
        return self.send_request("get serial number")

    def _prefixed_value(self, command: str, prefix: str) -> str:
        response = self.send_request(command)
        match = re.fullmatch(rf"{re.escape(prefix)}(.*)", response)
        if match is None:
            raise ProtocolError(f"could not parse {command!r} response {response!r}")
        return match.group(1)

    def robot_mode(self) -> str:
        return self._prefixed_value("robotmode", "Robotmode: ")

    def get_loaded_program(self) -> str:
        return self._prefixed_value("get loaded program", "Loaded program: ")

    def safety_mode(self) -> str:
        return self._prefixed_value("safetymode", "Safetymode: ")

    def safety_status(self) -> str:
        return self._prefixed_value("safetystatus", "Safetystatus: ")

    def program_state(self) -> ProgramState:
        response = self.send_request("programState")
        match = re.fullmatch(r"(STOPPED|PLAYING|PAUSED) (.*)", response)
        if match is None:
            raise ProtocolError(f"could not parse program state from {response!r}")
        return ProgramState(match.group(1), match.group(2))

    def get_operational_mode(self) -> str:
        return self.send_request("get operational mode")

    def set_operational_mode(self, operational_mode: str) -> DashboardResponse:
        expected = rf"(?:Operational mode ).*(?:{operational_mode}).*"
        return self._response(
            self.send_request(
                "set operational mode", expected, operational_mode
            )
        )

    def clear_operational_mode(self) -> DashboardResponse:
        return self._response(self.send_request("clear operational mode"))

    def set_user_role(self, user_role: str) -> DashboardResponse:
        return self._response(self.send_request("setUserRole", payload=user_role))

    def get_user_role(self) -> str:
        return self.send_request("getUserRole")

    def _request_with_temporary_timeout(
        self,
        timeout: float,
        command: str,
        expected: str,
        payload: str,
    ) -> DashboardResponse:
        configured = self._receive_timeout
        self.receive_timeout = timeout
        try:
            return self._response(self.send_request(command, expected, payload))
        finally:
            self.receive_timeout = configured

    def generate_flight_report(self, report_type: str = "") -> DashboardResponse:
        return self._request_with_temporary_timeout(
            180.0,
            "generate flight report",
            r"(?:Flight Report generated with id:).*",
            report_type,
        )

    def generate_support_file(self, directory_path: str) -> DashboardResponse:
        return self._request_with_temporary_timeout(
            600.0,
            "generate support file",
            r"(?:Completed successfully:).*",
            directory_path,
        )

    def download_support_files(self, save_path: str | Path) -> DashboardResponse:
        raise NotImplementedError(
            "support-file download is only available on PolyScope X"
        )

    def save_log(self) -> DashboardResponse:
        return self._response(self.send_request("saveLog"))

    def assert_version(
        self,
        e_series_minimum: str,
        cb3_minimum: str,
        required_call: str,
    ) -> None:
        actual = self.polyscope_version
        minimum_text = e_series_minimum if actual.is_e_series else cb3_minimum
        if minimum_text == "-":
            series = "e-Series" if actual.is_e_series else "CB3"
            raise VersionMismatchError(
                f"dashboard call {required_call!r} is unavailable on {series}",
                VersionInformation(),
                actual,
            )
        minimum = VersionInformation.from_string(minimum_text)
        if actual < minimum:
            raise VersionMismatchError(
                f"PolyScope is too old for dashboard call {required_call!r}",
                minimum,
                actual,
            )

    def get_program_list(self) -> tuple[object, ...]:
        raise NotImplementedError(
            f"program listing is unavailable on G5 PolyScope {self.polyscope_version}"
        )

    def upload_program(self, file_path: str | Path) -> DashboardResponse:
        raise NotImplementedError(
            f"program upload is unavailable on G5 PolyScope {self.polyscope_version}"
        )

    def update_program(self, file_path: str | Path) -> DashboardResponse:
        raise NotImplementedError(
            f"program update is unavailable on G5 PolyScope {self.polyscope_version}"
        )

    def download_program(
        self, program_name: str, save_path: str | Path
    ) -> DashboardResponse:
        raise NotImplementedError(
            f"program download is unavailable on G5 PolyScope {self.polyscope_version}"
        )

    # Direct snake_case translations of the C++ command names.
    command_power_off = power_off
    command_power_on = power_on
    command_brake_release = brake_release
    command_load_program = load_program
    command_load_installation = load_installation
    command_play = play
    command_pause = pause
    command_resume = resume
    command_stop = stop
    command_close_popup = close_popup
    command_close_safety_popup = close_safety_popup
    command_restart_safety = restart_safety
    command_unlock_protective_stop = unlock_protective_stop
    command_shutdown = shutdown
    command_quit = quit
    command_running = running
    command_is_program_saved = is_program_saved
    command_is_in_remote_control = is_in_remote_control
    command_popup = popup
    command_add_to_log = add_to_log
    command_polyscope_version = get_polyscope_version
    command_get_robot_model = get_robot_model
    command_get_serial_number = get_serial_number
    command_robot_mode = robot_mode
    command_get_loaded_program = get_loaded_program
    command_safety_mode = safety_mode
    command_safety_status = safety_status
    command_program_state = program_state
    command_get_operational_mode = get_operational_mode
    command_set_operational_mode = set_operational_mode
    command_clear_operational_mode = clear_operational_mode
    command_set_user_role = set_user_role
    command_get_user_role = get_user_role
    command_generate_flight_report = generate_flight_report
    command_generate_support_file = generate_support_file
    command_download_support_files = download_support_files
    command_save_log = save_log
    command_get_program_list = get_program_list
    command_upload_program = upload_program
    command_update_program = update_program
    command_download_program = download_program


G5DashboardClient = DashboardClientG5
DashboardClientImplG5 = DashboardClientG5

__all__ = ["DashboardClientG5", "DashboardClientImplG5", "G5DashboardClient"]
