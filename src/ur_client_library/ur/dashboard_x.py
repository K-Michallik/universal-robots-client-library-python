"""Synchronous PolyScope X Robot API dashboard backend."""

from __future__ import annotations

import json
import threading
from collections.abc import Callable
from pathlib import Path
from typing import Any, cast
from urllib.parse import urlsplit, urlunsplit

import httpx

from ur_client_library.exceptions import (
    DashboardCommandError,
    LifecycleError,
    ProtocolError,
    URCLConnectionError,
    URCLTimeoutError,
)
from ur_client_library.ur.dashboard_client import (
    DashboardResponse,
    ProgramInformation,
    ProgramSaveState,
    ProgramState,
    _download_result,
)
from ur_client_library.ur.version_information import VersionInformation

_PROGRAM_API_MINIMUM = VersionInformation.from_string("3.1.4")


def _validate_timeout(value: float, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a number")
    result = float(value)
    if result <= 0:
        raise ValueError(f"{name} must be greater than zero")
    return result


def _origin(host: str) -> str:
    if not isinstance(host, str) or not host.strip():
        raise ValueError("host must be a non-empty string")
    candidate = host.strip()
    if "://" not in candidate:
        candidate = f"http://{candidate}"
    parsed = urlsplit(candidate)
    if parsed.scheme not in {"http", "https"} or not parsed.netloc:
        raise ValueError(f"host {host!r} is not a valid HTTP(S) host")
    if parsed.path not in {"", "/"} or parsed.query or parsed.fragment:
        raise ValueError("host must not contain a path, query, or fragment")
    return urlunsplit((parsed.scheme, parsed.netloc, "", "", ""))


class DashboardClientX:
    """Robot API implementation used by PolyScope X."""

    BASE_PATH = "/universal-robots/robot-api"

    def __init__(
        self,
        host: str,
        *,
        connect_timeout: float = 5.0,
        receive_timeout: float = 10.0,
        send_timeout: float = 10.0,
        transport: httpx.BaseTransport | None = None,
        client_factory: Callable[..., httpx.Client] | None = None,
    ) -> None:
        self._origin = _origin(host)
        self._connect_timeout = _validate_timeout(connect_timeout, "connect_timeout")
        self._receive_timeout = _validate_timeout(receive_timeout, "receive_timeout")
        self._send_timeout = _validate_timeout(send_timeout, "send_timeout")
        self._transport = transport
        self._client_factory = client_factory or httpx.Client
        self._client: httpx.Client | None = None
        self._robot_api_version = VersionInformation()
        self._timeout_lock = threading.RLock()

    @property
    def is_connected(self) -> bool:
        return self._client is not None and not self._robot_api_version.is_empty

    @property
    def robot_api_version(self) -> VersionInformation:
        if self._robot_api_version.is_empty:
            raise LifecycleError("Robot API version is unavailable before connect() succeeds")
        return self._robot_api_version

    api_version = robot_api_version

    @property
    def version(self) -> VersionInformation:
        return self.robot_api_version

    @property
    def polyscope_version(self) -> VersionInformation:
        raise NotImplementedError(
            "the legacy PolyScope-version query is not implemented for PolyScope X"
        )

    @property
    def connect_timeout(self) -> float:
        return self._connect_timeout

    @connect_timeout.setter
    def connect_timeout(self, timeout: float) -> None:
        self._connect_timeout = _validate_timeout(timeout, "connect_timeout")
        self._apply_client_timeout()

    @property
    def receive_timeout(self) -> float:
        return self._receive_timeout

    @receive_timeout.setter
    def receive_timeout(self, timeout: float) -> None:
        self._receive_timeout = _validate_timeout(timeout, "receive_timeout")
        self._apply_client_timeout()

    @property
    def send_timeout(self) -> float:
        return self._send_timeout

    @send_timeout.setter
    def send_timeout(self, timeout: float) -> None:
        self._send_timeout = _validate_timeout(timeout, "send_timeout")
        self._apply_client_timeout()

    def set_receive_timeout(self, timeout: float) -> None:
        self.receive_timeout = timeout

    def get_configured_receive_timeout(self) -> float:
        return self.receive_timeout

    def set_send_timeout(self, timeout: float) -> None:
        self.send_timeout = timeout

    def get_configured_send_timeout(self) -> float:
        return self.send_timeout

    def _timeout(self) -> httpx.Timeout:
        return httpx.Timeout(
            connect=self._connect_timeout,
            read=self._receive_timeout,
            write=self._send_timeout,
            pool=self._connect_timeout,
        )

    def _apply_client_timeout(self) -> None:
        if self._client is not None:
            self._client.timeout = self._timeout()

    def connect(
        self,
        *,
        max_attempts: int = 0,
        retry_interval: float = 10.0,
    ) -> DashboardClientX:
        # The C++ X backend accepts the generic retry arguments but performs
        # one discovery call. Preserve that distinction from persistent G5.
        if isinstance(max_attempts, bool) or not isinstance(max_attempts, int) or max_attempts < 0:
            raise ValueError("max_attempts must be a non-negative integer")
        if isinstance(retry_interval, bool) or not isinstance(retry_interval, (int, float)):
            raise TypeError("retry_interval must be a number")
        if retry_interval < 0:
            raise ValueError("retry_interval must be non-negative")
        if self.is_connected or self._client is not None:
            raise LifecycleError("dashboard client is already connected")

        if self._transport is None:
            self._client = self._client_factory(
                base_url=self._origin,
                follow_redirects=True,
                timeout=self._timeout(),
            )
        else:
            self._client = self._client_factory(
                base_url=self._origin,
                follow_redirects=True,
                timeout=self._timeout(),
                transport=self._transport,
            )
        try:
            response = self._perform_request("GET", "/openapi.json", connecting=True)
            if response.status_code != 200:
                raise URCLConnectionError(
                    "Robot API discovery returned "
                    f"HTTP {response.status_code}: {response.text}"
                )
            payload = self._decode_json(response.text, "Robot API OpenAPI document")
            info = payload.get("info")
            if not isinstance(info, dict):
                raise ProtocolError(
                    "Robot API OpenAPI document has no string info.version"
                )
            version = info.get("version")
            if not isinstance(version, str):
                raise ProtocolError(
                    "Robot API OpenAPI document has no string info.version"
                )
            self._robot_api_version = VersionInformation.from_string(version)
            return self
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        client, self._client = self._client, None
        self._robot_api_version = VersionInformation()
        if client is not None:
            client.close()

    disconnect = close

    def __enter__(self) -> DashboardClientX:
        return self.connect()

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: object,
    ) -> None:
        self.close()

    def _require_client(self) -> httpx.Client:
        if self._client is None or self._robot_api_version.is_empty:
            raise LifecycleError("dashboard client is not connected")
        return self._client

    def _perform_request(
        self,
        method: str,
        endpoint: str,
        *,
        connecting: bool = False,
        json_data: dict[str, object] | None = None,
        files: dict[str, tuple[str, bytes, str]] | None = None,
    ) -> httpx.Response:
        client = self._client
        if client is None or (not connecting and self._robot_api_version.is_empty):
            raise LifecycleError("dashboard client is not connected")
        try:
            return client.request(
                method,
                f"{self.BASE_PATH}{endpoint}",
                json=json_data,
                files=files,
                timeout=self._timeout(),
            )
        except httpx.ConnectTimeout as exc:
            raise URCLTimeoutError(
                f"Robot API {method} {endpoint} timed out while connecting",
                self._connect_timeout,
            ) from exc
        except httpx.WriteTimeout as exc:
            raise URCLTimeoutError(
                f"Robot API {method} {endpoint} timed out while writing",
                self._send_timeout,
            ) from exc
        except httpx.ReadTimeout as exc:
            raise URCLTimeoutError(
                f"Robot API {method} {endpoint} timed out while reading",
                self._receive_timeout,
            ) from exc
        except httpx.PoolTimeout as exc:
            raise URCLTimeoutError(
                f"Robot API {method} {endpoint} timed out waiting for a connection",
                self._connect_timeout,
            ) from exc
        except httpx.HTTPError as exc:
            raise URCLConnectionError(
                f"Robot API {method} {endpoint} failed: {exc}"
            ) from exc

    @staticmethod
    def _decode_json(message: str, source: str) -> dict[str, Any]:
        try:
            payload = json.loads(message)
        except json.JSONDecodeError as exc:
            raise ProtocolError(f"{source} did not contain valid JSON") from exc
        if not isinstance(payload, dict):
            raise ProtocolError(f"{source} must contain a JSON object")
        return cast(dict[str, Any], payload)

    def _request(
        self,
        method: str,
        endpoint: str,
        *,
        json_data: dict[str, object] | None = None,
        files: dict[str, tuple[str, bytes, str]] | None = None,
    ) -> DashboardResponse:
        response = self._perform_request(
            method,
            endpoint,
            json_data=json_data,
            files=files,
        )
        if response.status_code != 200:
            raise DashboardCommandError(
                f"{method} {endpoint}",
                response.text,
                status=response.status_code,
            )
        data: dict[str, object] = {}
        if response.text:
            try:
                decoded = response.json()
            except json.JSONDecodeError:
                decoded = None
            if isinstance(decoded, dict):
                data.update(decoded)
        data["status_code"] = response.status_code
        return DashboardResponse(
            message=response.text,
            data=data,
            status_code=response.status_code,
        )

    def _json_request(
        self,
        method: str,
        endpoint: str,
        *,
        json_data: dict[str, object] | None = None,
    ) -> tuple[DashboardResponse, dict[str, Any]]:
        response = self._request(method, endpoint, json_data=json_data)
        return response, self._decode_json(response.message, f"{method} {endpoint} response")

    @staticmethod
    def _string_field(payload: dict[str, Any], field: str, source: str) -> str:
        value = payload.get(field)
        if not isinstance(value, str):
            raise ProtocolError(f"{source} has no string {field!r} field")
        return value

    def _require_program_api(self, operation: str) -> None:
        if self.robot_api_version < _PROGRAM_API_MINIMUM:
            raise NotImplementedError(
                f"{operation} requires Robot API 3.1.4 (PolyScope 10.12.0) or newer; "
                f"connected API is {self.robot_api_version}"
            )

    def power_off(self) -> DashboardResponse:
        return self._request(
            "PUT", "/robotstate/v1/state", json_data={"action": "POWER_OFF"}
        )

    def power_on(self, timeout: float = 300.0) -> DashboardResponse:
        temporary_timeout = _validate_timeout(timeout, "timeout")
        with self._timeout_lock:
            configured = self._receive_timeout
            self.receive_timeout = temporary_timeout
            try:
                return self._request(
                    "PUT",
                    "/robotstate/v1/state",
                    json_data={"action": "POWER_ON"},
                )
            finally:
                self.receive_timeout = configured

    def brake_release(self) -> DashboardResponse:
        return self._request(
            "PUT",
            "/robotstate/v1/state",
            json_data={"action": "BRAKE_RELEASE"},
        )

    def load_program(self, program_file_name: str) -> DashboardResponse:
        if self.robot_api_version < _PROGRAM_API_MINIMUM:
            return self._request(
                "PUT",
                "/program/v1/load",
                json_data={"programName": program_file_name},
            )
        return self._request(
            "PUT",
            "/program/v1/loaded",
            json_data={"name": program_file_name},
        )

    def load_installation(self, installation_file_name: str) -> DashboardResponse:
        raise NotImplementedError("installation loading is not implemented for PolyScope X")

    def play(self) -> DashboardResponse:
        return self._request(
            "PUT", "/program/v1/state", json_data={"action": "play"}
        )

    def pause(self) -> DashboardResponse:
        return self._request(
            "PUT", "/program/v1/state", json_data={"action": "pause"}
        )

    def resume(self) -> DashboardResponse:
        return self._request(
            "PUT", "/program/v1/state", json_data={"action": "resume"}
        )

    def stop(self) -> DashboardResponse:
        return self._request(
            "PUT", "/program/v1/state", json_data={"action": "stop"}
        )

    def close_popup(self) -> DashboardResponse:
        raise NotImplementedError("closing popups is not implemented for PolyScope X")

    def close_safety_popup(self) -> DashboardResponse:
        raise NotImplementedError("closing safety popups is not implemented for PolyScope X")

    def restart_safety(self) -> DashboardResponse:
        return self._request(
            "PUT",
            "/robotstate/v1/state",
            json_data={"action": "RESTART_SAFETY"},
        )

    def unlock_protective_stop(self) -> DashboardResponse:
        return self._request(
            "PUT",
            "/robotstate/v1/state",
            json_data={"action": "UNLOCK_PROTECTIVE_STOP"},
        )

    def shutdown(self) -> DashboardResponse:
        raise NotImplementedError("shutdown is not implemented for PolyScope X")

    def quit(self) -> DashboardResponse:
        self._require_client()
        return DashboardResponse(message="Nothing to quit; the Robot API has no session.")

    def running(self) -> bool:
        raise NotImplementedError("running is not implemented for PolyScope X")

    def is_program_saved(self) -> ProgramSaveState:
        raise NotImplementedError("program save status is not implemented for PolyScope X")

    def is_in_remote_control(self) -> bool:
        self._require_program_api("remote-control query")
        _response, payload = self._json_request("GET", "/system/v1/controlmode")
        return self._string_field(payload, "mode", "control-mode response") == "REMOTE"

    def popup(self, popup_text: str) -> DashboardResponse:
        raise NotImplementedError("popups are not implemented for PolyScope X")

    def add_to_log(self, log_text: str) -> DashboardResponse:
        raise NotImplementedError("adding log messages is not implemented for PolyScope X")

    def get_polyscope_version(self) -> VersionInformation:
        raise NotImplementedError(
            "the legacy PolyScope-version query is not implemented for PolyScope X"
        )

    def get_robot_model(self) -> str:
        raise NotImplementedError("robot-model query is not implemented for PolyScope X")

    def get_serial_number(self) -> str:
        raise NotImplementedError("serial-number query is not implemented for PolyScope X")

    def robot_mode(self) -> str:
        self._require_program_api("robot-mode query")
        _response, payload = self._json_request("GET", "/robotstate/v1/robotmode")
        return self._string_field(payload, "mode", "robot-mode response")

    def get_loaded_program(self) -> str:
        self._require_program_api("loaded-program query")
        _response, payload = self._json_request("GET", "/program/v1/loaded")
        return self._string_field(payload, "name", "loaded-program response")

    def safety_mode(self) -> str:
        self._require_program_api("safety-mode query")
        _response, payload = self._json_request("GET", "/robotstate/v1/safetymode")
        return self._string_field(payload, "mode", "safety-mode response")

    def safety_status(self) -> str:
        raise NotImplementedError("safety-status query is not implemented for PolyScope X")

    def program_state(self) -> ProgramState:
        _response, payload = self._json_request("GET", "/program/v1/state")
        state = self._string_field(payload, "state", "program-state response")
        return ProgramState(state)

    def get_operational_mode(self) -> str:
        self._require_program_api("operational-mode query")
        _response, payload = self._json_request("GET", "/system/v1/operationalmode")
        return self._string_field(payload, "mode", "operational-mode response")

    def set_operational_mode(self, operational_mode: str) -> DashboardResponse:
        raise NotImplementedError("setting operational mode is not implemented for PolyScope X")

    def clear_operational_mode(self) -> DashboardResponse:
        raise NotImplementedError("clearing operational mode is not implemented for PolyScope X")

    def set_user_role(self, user_role: str) -> DashboardResponse:
        raise NotImplementedError("user roles are not implemented for PolyScope X")

    def get_user_role(self) -> str:
        raise NotImplementedError("user roles are not implemented for PolyScope X")

    def generate_flight_report(self, report_type: str) -> DashboardResponse:
        raise NotImplementedError("flight reports are not implemented for PolyScope X")

    def generate_support_file(self, directory_path: str) -> DashboardResponse:
        raise NotImplementedError("support files are not implemented for PolyScope X")

    def save_log(self) -> DashboardResponse:
        raise NotImplementedError("saving logs is not implemented for PolyScope X")

    def assert_version(
        self,
        e_series_minimum: str,
        cb3_minimum: str,
        required_call: str,
    ) -> None:
        raise NotImplementedError(
            "legacy PolyScope version assertions are unavailable on PolyScope X"
        )

    @staticmethod
    def _required_int(
        payload: dict[str, Any],
        field: str,
        *,
        nullable: bool = False,
    ) -> int:
        value = payload.get(field)
        if value is None and nullable:
            return 0
        if isinstance(value, bool) or not isinstance(value, int):
            raise ProtocolError(f"program metadata has no integer {field!r} field")
        return value

    def get_program_list(self) -> tuple[ProgramInformation, ...]:
        self._require_program_api("program listing")
        _response, payload = self._json_request("GET", "/programs/v1/")
        raw_programs = payload.get("programs")
        if not isinstance(raw_programs, list):
            raise ProtocolError("program-list response has no array 'programs' field")
        result: list[ProgramInformation] = []
        for raw_program in raw_programs:
            if not isinstance(raw_program, dict):
                raise ProtocolError("program-list entry must be an object")
            result.append(
                ProgramInformation(
                    created_date=self._required_int(raw_program, "createdDate"),
                    description=self._string_field(
                        raw_program, "description", "program metadata"
                    ),
                    last_modified_date=self._required_int(
                        raw_program, "lastModifiedDate", nullable=True
                    ),
                    last_saved_date=self._required_int(
                        raw_program, "lastSavedDate", nullable=True
                    ),
                    name=self._string_field(raw_program, "name", "program metadata"),
                    program_state=self._string_field(
                        raw_program, "programState", "program metadata"
                    ),
                )
            )
        return tuple(result)

    def _program_upload(self, file_path: str | Path, method: str) -> DashboardResponse:
        self._require_program_api("program upload")
        path = Path(file_path)
        try:
            content = path.read_bytes()
        except FileNotFoundError:
            raise
        except OSError as exc:
            raise OSError(f"failed to read program file {path}") from exc
        response = self._request(
            method,
            "/programs/v1/",
            files={"file": (path.name, content, "text/plain")},
        )
        program_name = response.data.get("programName")
        if program_name is not None and not isinstance(program_name, str):
            raise ProtocolError("program-upload response programName must be a string")
        data = dict(response.data)
        if isinstance(program_name, str):
            data["program_name"] = program_name
        return DashboardResponse(
            message=response.message,
            data=data,
            status_code=response.status_code,
        )

    def upload_program(self, file_path: str | Path) -> DashboardResponse:
        return self._program_upload(file_path, "POST")

    def update_program(self, file_path: str | Path) -> DashboardResponse:
        return self._program_upload(file_path, "PUT")

    def download_program(
        self, program_name: str, save_path: str | Path
    ) -> DashboardResponse:
        self._require_program_api("program download")
        if not program_name:
            raise ValueError("program_name must not be empty")
        if not str(save_path):
            raise ValueError("save_path must not be empty")
        path = Path(save_path)
        response = self._request("GET", f"/programs/v1/{program_name}")
        try:
            path.write_text(response.message, encoding="utf-8")
        except OSError as exc:
            raise OSError(f"failed to save downloaded program to {path}") from exc
        return _download_result(
            path,
            DashboardResponse(
                message=f"Downloaded program to {path}",
                data=response.data,
                status_code=response.status_code,
            ),
        )

    def send_and_receive(self, text: str) -> str:
        raise NotImplementedError("raw dashboard TCP commands are unavailable on PolyScope X")

    def send_request(self, command: str, expected: str = "", payload: str = "") -> str:
        raise NotImplementedError("regex dashboard requests are unavailable on PolyScope X")

    def wait_for_reply(
        self,
        command: str,
        expected: str,
        timeout: float = 30.0,
        *,
        poll_period: float = 0.1,
    ) -> bool:
        raise NotImplementedError("dashboard polling commands are unavailable on PolyScope X")

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
        raise NotImplementedError("dashboard retry commands are unavailable on PolyScope X")

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
    command_save_log = save_log
    command_get_program_list = get_program_list
    command_upload_program = upload_program
    command_update_program = update_program
    command_download_program = download_program


PolyScopeXDashboardClient = DashboardClientX
DashboardClientImplX = DashboardClientX

__all__ = [
    "DashboardClientImplX",
    "DashboardClientX",
    "PolyScopeXDashboardClient",
]
