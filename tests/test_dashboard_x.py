from __future__ import annotations

import json
import logging
from collections.abc import Callable
from pathlib import Path

import httpx
import pytest

from ur_client_library.exceptions import (
    DashboardCommandError,
    LifecycleError,
    ProtocolError,
    URCLConnectionError,
    URCLTimeoutError,
)
from ur_client_library.ur.dashboard_client import (
    DashboardClient,
    DashboardClientPolicy,
    ProgramInformation,
    ProgramState,
)
from ur_client_library.ur.dashboard_x import DashboardClientX
from ur_client_library.ur.version_information import VersionInformation

OPENAPI_PATH = "/universal-robots/robot-api/openapi.json"
BASE = "/universal-robots/robot-api"


def response(status: int, payload: object) -> httpx.Response:
    return httpx.Response(status, json=payload)


def client_for(
    handler: httpx.MockTransport | object,
) -> DashboardClientX:
    transport = (
        handler
        if isinstance(handler, httpx.MockTransport)
        else httpx.MockTransport(handler)
    )
    return DashboardClientX("robot.example", transport=transport)


def test_constructor_is_side_effect_free_and_connect_discovers_api_version() -> None:
    requests: list[httpx.Request] = []

    def handler(request: httpx.Request) -> httpx.Response:
        requests.append(request)
        assert request.url.path == OPENAPI_PATH
        return response(200, {"info": {"version": "3.1.4"}})

    client = client_for(handler)
    assert requests == []
    assert client.connect_timeout == 5.0
    assert client.receive_timeout == 10.0
    assert client.send_timeout == 10.0

    assert client.connect() is client
    assert str(client.robot_api_version) == "3.1.4.0"
    assert client.version == client.robot_api_version
    with pytest.raises(NotImplementedError):
        _ = client.polyscope_version
    assert len(requests) == 1


def test_context_manager_closes_and_close_is_idempotent() -> None:
    def handler(_request: httpx.Request) -> httpx.Response:
        return response(200, {"info": {"version": "3.1.4"}})

    client = client_for(handler)
    with client as connected:
        assert connected.is_connected
    assert not client.is_connected
    client.close()


def test_operation_before_connect_is_lifecycle_error() -> None:
    client = client_for(lambda _request: response(200, {}))
    with pytest.raises(LifecycleError):
        client.play()


@pytest.mark.parametrize(
    ("version", "expected_path", "expected_json"),
    [
        ("3.1.3", f"{BASE}/program/v1/load", {"programName": "demo"}),
        ("3.1.4", f"{BASE}/program/v1/loaded", {"name": "demo"}),
    ],
)
def test_load_program_branches_at_api_3_1_4(
    version: str,
    expected_path: str,
    expected_json: dict[str, str],
) -> None:
    seen: list[httpx.Request] = []

    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path == OPENAPI_PATH:
            return response(200, {"info": {"version": version}})
        seen.append(request)
        return response(200, {"message": "loaded"})

    client = client_for(handler)
    client.connect()
    client.load_program("demo")
    assert seen[0].method == "PUT"
    assert seen[0].url.path == expected_path
    assert json.loads(seen[0].content) == expected_json


def test_redirects_are_followed_for_non_get_requests() -> None:
    paths: list[str] = []

    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path == OPENAPI_PATH:
            return response(200, {"info": {"version": "3.1.4"}})
        paths.append(request.url.path)
        if request.url.path.endswith("/program/v1/state"):
            return httpx.Response(
                307,
                headers={"Location": f"{BASE}/program/v1/state/"},
            )
        assert request.method == "PUT"
        return response(200, {"state": "PLAYING"})

    client = client_for(handler)
    client.connect()
    client.play()
    assert paths == [f"{BASE}/program/v1/state", f"{BASE}/program/v1/state/"]


def test_query_values_are_typed() -> None:
    payloads = {
        f"{BASE}/program/v1/state": {"state": "PAUSED"},
        f"{BASE}/robotstate/v1/robotmode": {"mode": "IDLE"},
        f"{BASE}/robotstate/v1/safetymode": {"mode": "NORMAL"},
        f"{BASE}/system/v1/controlmode": {"mode": "REMOTE"},
        f"{BASE}/system/v1/operationalmode": {"mode": "AUTOMATIC"},
        f"{BASE}/program/v1/loaded": {"name": "demo"},
    }

    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path == OPENAPI_PATH:
            return response(200, {"info": {"version": "3.1.4"}})
        return response(200, payloads[request.url.path])

    client = client_for(handler)
    client.connect()
    assert client.program_state() == ProgramState("PAUSED")
    assert client.robot_mode() == "IDLE"
    assert client.safety_mode() == "NORMAL"
    assert client.is_in_remote_control() is True
    assert client.get_operational_mode() == "AUTOMATIC"
    assert client.get_loaded_program() == "demo"


def test_program_list_maps_nullable_dates() -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path == OPENAPI_PATH:
            return response(200, {"info": {"version": "3.1.4"}})
        assert request.url.path == f"{BASE}/programs/v1/"
        return response(
            200,
            {
                "programs": [
                    {
                        "createdDate": 123,
                        "description": "fixture",
                        "lastModifiedDate": None,
                        "lastSavedDate": 789,
                        "name": "demo",
                        "programState": "FINAL",
                    }
                ]
            },
        )

    client = client_for(handler)
    client.connect()
    assert client.get_program_list() == (
        ProgramInformation(123, "fixture", 0, 789, "demo", "FINAL"),
    )


def test_http_rejection_raises_dashboard_command_error() -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path == OPENAPI_PATH:
            return response(200, {"info": {"version": "3.1.4"}})
        return response(409, {"message": "remote control required"})

    client = client_for(handler)
    client.connect()
    with pytest.raises(DashboardCommandError) as raised:
        client.power_off()
    assert raised.value.status == 409


def test_power_on_temporarily_overrides_and_restores_read_timeout() -> None:
    read_timeouts: list[float] = []

    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path == OPENAPI_PATH:
            return response(200, {"info": {"version": "3.1.4"}})
        read_timeouts.append(request.extensions["timeout"]["read"])
        return response(503, {"message": "not ready"})

    client = client_for(handler)
    client.connect()
    client.receive_timeout = 7.25
    with pytest.raises(DashboardCommandError):
        client.power_on(timeout=42.5)
    assert read_timeouts == [42.5]
    assert client.receive_timeout == 7.25
    assert client._client is not None
    assert client._client.timeout.read == 7.25


def test_multipart_upload_and_update(tmp_path: Path) -> None:
    program = tmp_path / "fixture.urpx"
    program.write_text('{"name": "fixture"}', encoding="utf-8")
    requests: list[httpx.Request] = []

    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path == OPENAPI_PATH:
            return response(200, {"info": {"version": "3.1.4"}})
        requests.append(request)
        return response(200, {"programName": "fixture"})

    client = client_for(handler)
    client.connect()
    uploaded = client.upload_program(program)
    updated = client.update_program(program)

    assert [request.method for request in requests] == ["POST", "PUT"]
    for request in requests:
        assert request.url.path == f"{BASE}/programs/v1/"
        assert request.headers["content-type"].startswith("multipart/form-data; boundary=")
        assert b'name="file"; filename="fixture.urpx"' in request.content
        assert b'{"name": "fixture"}' in request.content
    assert uploaded.data["program_name"] == "fixture"
    assert updated.data["program_name"] == "fixture"


def test_download_writes_response_and_returns_path(tmp_path: Path) -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path == OPENAPI_PATH:
            return response(200, {"info": {"version": "3.1.4"}})
        return httpx.Response(200, text='{"program": "demo"}')

    target = tmp_path / "demo.urpx"
    client = client_for(handler)
    client.connect()
    result = client.download_program("demo", target)
    assert target.read_text(encoding="utf-8") == '{"program": "demo"}'
    assert result.data["path"] == target


@pytest.mark.parametrize(
    "operation",
    [
        lambda client: client.close_popup(),
        lambda client: client.shutdown(),
        lambda client: client.running(),
        lambda client: client.get_robot_model(),
        lambda client: client.safety_status(),
        lambda client: client.set_operational_mode("AUTOMATIC"),
        lambda client: client.save_log(),
    ],
)
def test_unsupported_matrix_raises_not_implemented(
    operation: Callable[[DashboardClientX], object],
) -> None:
    client = client_for(
        lambda request: response(200, {"info": {"version": "3.1.4"}})
    )
    client.connect()
    with pytest.raises(NotImplementedError):
        operation(client)


def test_facade_selects_x_without_probing_in_constructor() -> None:
    calls = 0

    def handler(_request: httpx.Request) -> httpx.Response:
        nonlocal calls
        calls += 1
        return response(200, {"info": {"version": "3.1.4"}})

    facade = DashboardClient(
        "robot",
        DashboardClientPolicy.POLYSCOPE_X,
        transport=httpx.MockTransport(handler),
    )
    assert calls == 0
    facade.connect()
    assert calls == 1
    assert str(facade.version) == "3.1.4.0"


# --- Robot API 5.0.107 / PolyScope 10.14.0 commands (upstream #549) ---

API_10_14 = "5.0.107"
SUPPORTFILES = f"{BASE}/supportfiles/v1"
FORBIDDEN = {
    "message": "Forbidden",
    "details": "Not authorized to perform this operation under the current "
    "robot control mode.",
}

Handler = Callable[[httpx.Request], httpx.Response]


def connected_client(
    handler: Handler,
    *,
    version: str = API_10_14,
    seen: list[httpx.Request] | None = None,
) -> DashboardClientX:
    def dispatch(request: httpx.Request) -> httpx.Response:
        if request.url.path == OPENAPI_PATH:
            return response(200, {"info": {"version": version}})
        if seen is not None:
            seen.append(request)
        return handler(request)

    client = client_for(dispatch)
    client.connect()
    return client


def unexpected(request: httpx.Request) -> httpx.Response:
    raise AssertionError(f"unexpected request {request.method} {request.url.path}")


NEW_COMMANDS: list[tuple[str, Callable[[DashboardClientX], object]]] = [
    ("close_popup", lambda client: client.close_popup()),
    ("close_safety_popup", lambda client: client.close_safety_popup()),
    ("shutdown", lambda client: client.shutdown()),
    ("popup", lambda client: client.popup("text", "title")),
    ("add_to_log", lambda client: client.add_to_log("text")),
    ("PolyscopeVersion", lambda client: client.get_polyscope_version()),
    ("PolyscopeVersion", lambda client: client.polyscope_version),
    ("get_robot_model", lambda client: client.get_robot_model()),
    ("get_serial_number", lambda client: client.get_serial_number()),
    ("generate_flight_report", lambda client: client.generate_flight_report()),
    ("download_support_files", lambda client: client.download_support_files("x.zip")),
]


@pytest.mark.parametrize(("command", "operation"), NEW_COMMANDS)
@pytest.mark.parametrize("version", ["3.1.4", "5.0.106"])
def test_new_commands_require_robot_api_5_0_107(
    version: str,
    command: str,
    operation: Callable[[DashboardClientX], object],
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.chdir(tmp_path)
    seen: list[httpx.Request] = []
    client = connected_client(unexpected, version=version, seen=seen)
    with pytest.raises(NotImplementedError) as raised:
        operation(client)
    message = str(raised.value)
    assert f"'{command}' requires Robot API version 5.0.107.0" in message
    assert "upgrade the robot to PolyScope 10.14.0.0" in message
    assert seen == []
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize(("command", "operation"), NEW_COMMANDS)
def test_new_commands_before_connect_are_lifecycle_errors(
    command: str,
    operation: Callable[[DashboardClientX], object],
) -> None:
    client = client_for(unexpected)
    with pytest.raises(LifecycleError):
        operation(client)


@pytest.mark.parametrize(
    "operation",
    [
        lambda client: client.get_program_list(),
        lambda client: client.robot_mode(),
        lambda client: client.is_in_remote_control(),
        lambda client: client.download_program("demo", "demo.urpx"),
    ],
)
def test_program_api_gate_keeps_3_1_4_and_10_12_0(
    operation: Callable[[DashboardClientX], object],
) -> None:
    client = connected_client(unexpected, version="3.1.3")
    with pytest.raises(NotImplementedError) as raised:
        operation(client)
    assert "Robot API version 3.1.4.0" in str(raised.value)
    assert "PolyScope 10.12.0.0" in str(raised.value)


@pytest.mark.parametrize(
    ("operation", "method", "path", "payload", "expected_body"),
    [
        (lambda c: c.close_popup(), "DELETE", "/popup/v1", {"closed": True}, None),
        (
            lambda c: c.close_safety_popup(),
            "DELETE",
            "/popup/v1/safety",
            {"closed": True},
            None,
        ),
        (
            lambda c: c.popup("Hello", "Greeting"),
            "POST",
            "/popup/v1",
            {"message": "Popup opened successfully"},
            {"title": "Greeting", "message": "Hello"},
        ),
        (
            lambda c: c.command_popup("Hello"),
            "POST",
            "/popup/v1",
            {"message": "Popup opened successfully"},
            {"title": "", "message": "Hello"},
        ),
        (
            lambda c: c.add_to_log("Log line"),
            "POST",
            "/system/v1/log",
            {"message": "Log entry added"},
            {"message": "Log line"},
        ),
    ],
)
def test_popup_and_log_commands_send_robot_api_requests(
    operation: Callable[[DashboardClientX], object],
    method: str,
    path: str,
    payload: dict[str, object],
    expected_body: dict[str, str] | None,
) -> None:
    seen: list[httpx.Request] = []
    client = connected_client(lambda _request: response(200, payload), seen=seen)
    result = operation(client)
    assert result.ok  # type: ignore[attr-defined]
    assert [(request.method, request.url.path) for request in seen] == [
        (method, f"{BASE}{path}")
    ]
    if expected_body is None:
        assert seen[0].content == b""
    else:
        assert json.loads(seen[0].content) == expected_body


def test_popup_rejection_raises_dashboard_command_error() -> None:
    client = connected_client(lambda _request: response(403, FORBIDDEN))
    with pytest.raises(DashboardCommandError) as raised:
        client.close_popup()
    assert raised.value.status == 403
    assert "Forbidden" in raised.value.response


def test_information_queries_are_typed() -> None:
    payloads = {
        f"{BASE}/versions/v1": {
            "marketingVersion": "10.14.0",
            "baselineVersion": "10.14.0.12345",
        },
        f"{BASE}/system/v1/information": {
            "robotType": "UR7",
            "serialNumber": "20265000001",
        },
    }
    seen: list[httpx.Request] = []
    client = connected_client(
        lambda request: response(200, payloads[request.url.path]), seen=seen
    )
    assert client.get_polyscope_version() == VersionInformation(10, 14, 0)
    assert client.command_polyscope_version() == VersionInformation(10, 14, 0)
    assert client.polyscope_version == VersionInformation(10, 14, 0)
    assert client.get_robot_model() == "UR7"
    assert client.get_serial_number() == "20265000001"
    assert {request.method for request in seen} == {"GET"}


def test_polyscope_version_property_through_facade() -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path == OPENAPI_PATH:
            return response(200, {"info": {"version": API_10_14}})
        return response(200, {"marketingVersion": "10.14.1"})

    facade = DashboardClient(
        "robot",
        DashboardClientPolicy.POLYSCOPE_X,
        transport=httpx.MockTransport(handler),
    )
    with facade:
        assert facade.polyscope_version == VersionInformation(10, 14, 1)


def test_information_query_without_field_is_protocol_error() -> None:
    client = connected_client(lambda _request: response(200, {"robotType": None}))
    with pytest.raises(ProtocolError):
        client.get_robot_model()


@pytest.mark.parametrize(
    ("status", "payload"),
    [
        (202, {"message": None}),
        (403, FORBIDDEN),
        (404, {"detail": "Not Found"}),
    ],
)
def test_shutdown_puts_empty_json_body(status: int, payload: object) -> None:
    seen: list[httpx.Request] = []
    client = connected_client(lambda _request: response(status, payload), seen=seen)
    if status == 202:
        result = client.command_shutdown()
        assert result.ok
        assert result.status_code == 202
        assert result.data["status_code"] == 202
    else:
        with pytest.raises(DashboardCommandError) as raised:
            client.shutdown()
        assert raised.value.status == status
    request = seen[0]
    assert (request.method, request.url.path) == ("PUT", f"{BASE}/system/v1/shutdown")
    assert request.content == b""
    assert request.headers["content-type"] == "application/json"
    assert request.headers["content-length"] == "0"


@pytest.mark.parametrize("status", [200, 408, 507])
def test_flight_report_posts_with_30_second_read_timeout(status: int) -> None:
    seen: list[httpx.Request] = []
    body = {"message": "Flight report generated successfully.", "details": None}
    client = connected_client(lambda _request: response(status, body), seen=seen)
    client.receive_timeout = 7.25
    if status == 200:
        result = client.generate_flight_report()
        assert result.status_code == 200
    else:
        with pytest.raises(DashboardCommandError) as raised:
            client.generate_flight_report()
        assert raised.value.status == status
    request = seen[0]
    assert (request.method, request.url.path) == ("POST", SUPPORTFILES)
    assert request.content == b""
    assert request.headers["content-type"] == "application/json"
    assert request.extensions["timeout"]["read"] == 30.0
    assert client.receive_timeout == 7.25
    assert client._client is not None
    assert client._client.timeout.read == 7.25


def test_flight_report_restores_timeout_after_transport_error() -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        raise httpx.ReadTimeout("slow", request=request)

    client = connected_client(handler)
    with pytest.raises(URCLTimeoutError) as raised:
        client.generate_flight_report()
    assert raised.value.timeout == 30.0
    assert client.receive_timeout == 10.0


def test_flight_report_ignores_report_type_with_warning(
    caplog: pytest.LogCaptureFixture,
) -> None:
    seen: list[httpx.Request] = []
    client = connected_client(
        lambda _request: response(200, {"message": "ok"}), seen=seen
    )
    with caplog.at_level(logging.WARNING, logger="ur_client_library.ur.dashboard_x"):
        client.command_generate_flight_report("blackbox")
    assert seen[0].content == b""
    assert "report_type parameter is not used" in caplog.text


def leftovers(directory: Path, *, keep: Path | None = None) -> list[Path]:
    return sorted(path for path in directory.iterdir() if path != keep)


def test_download_support_files_streams_archive(tmp_path: Path) -> None:
    archive = b"PK\x03\x04fake_zip_data" * 10_000
    seen: list[httpx.Request] = []
    client = connected_client(
        lambda _request: httpx.Response(
            200, content=archive, headers={"Content-Type": "application/zip"}
        ),
        seen=seen,
    )
    target = tmp_path / "support.zip"
    result = client.download_support_files(target)
    assert target.read_bytes() == archive
    assert result.data["path"] == target
    assert result.status_code == 200
    assert (seen[0].method, seen[0].url.path) == ("GET", SUPPORTFILES)
    assert leftovers(tmp_path, keep=target) == []


def test_download_support_files_204_creates_empty_file(tmp_path: Path) -> None:
    client = connected_client(lambda _request: httpx.Response(204))
    target = tmp_path / "support.zip"
    result = client.command_download_support_files(str(target))
    assert target.read_bytes() == b""
    assert result.status_code == 204
    assert leftovers(tmp_path, keep=target) == []


def test_download_support_files_overwrites_existing_file(tmp_path: Path) -> None:
    contents = [b"version1_data", b"version2_longer_data"]
    client = connected_client(
        lambda _request: httpx.Response(200, content=contents[0])
    )
    target = tmp_path / "support.zip"
    target.write_bytes(b"stale content that is longer than both downloads")
    client.download_support_files(target)
    assert target.read_bytes() == b"version1_data"
    contents.pop(0)
    client.download_support_files(target)
    assert target.read_bytes() == b"version2_longer_data"
    assert leftovers(tmp_path, keep=target) == []


@pytest.mark.parametrize(
    ("status", "message"),
    [
        (404, "Flight recorder service not found."),
        (500, "Internal server error."),
    ],
)
def test_download_support_files_http_error(
    status: int, message: str, tmp_path: Path
) -> None:
    body = {"message": message, "details": ""}
    client = connected_client(lambda _request: response(status, body))
    target = tmp_path / "support.zip"
    with pytest.raises(DashboardCommandError) as raised:
        client.download_support_files(target)
    assert raised.value.status == status
    assert json.loads(raised.value.response) == body
    assert leftovers(tmp_path) == []


def test_download_support_files_invalid_path_sends_nothing(tmp_path: Path) -> None:
    seen: list[httpx.Request] = []
    client = connected_client(unexpected, seen=seen)
    with pytest.raises(OSError, match="temporary file"):
        client.download_support_files(tmp_path / "missing" / "support.zip")
    assert seen == []
    assert leftovers(tmp_path) == []


def test_download_support_files_connection_error(tmp_path: Path) -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        raise httpx.ConnectError("refused", request=request)

    client = connected_client(handler)
    with pytest.raises(URCLConnectionError):
        client.download_support_files(tmp_path / "support.zip")
    assert leftovers(tmp_path) == []


def test_download_support_files_rename_failure_removes_temp(tmp_path: Path) -> None:
    client = connected_client(lambda _request: httpx.Response(200, content=b"zip"))
    target = tmp_path / "support.zip"
    target.mkdir()
    with pytest.raises(OSError, match="failed to move"):
        client.download_support_files(target)
    assert target.is_dir()
    assert list(target.iterdir()) == []
    assert leftovers(tmp_path, keep=target) == []


def test_download_support_files_empty_path_is_value_error() -> None:
    client = connected_client(unexpected)
    with pytest.raises(ValueError):
        client.download_support_files("")


PROGRAM = {
    "createdDate": 123,
    "description": "fixture",
    "lastModifiedDate": 456,
    "lastSavedDate": 789,
    "name": "demo",
    "programState": "FINAL",
}


def program_list_client(programs: list[dict[str, object]]) -> DashboardClientX:
    return connected_client(
        lambda _request: response(200, {"programs": programs}), version="3.1.4"
    )


@pytest.mark.parametrize(
    ("field", "expected"),
    [
        ("createdDate", ProgramInformation(0, "fixture", 456, 789, "demo", "FINAL")),
        ("description", ProgramInformation(123, "", 456, 789, "demo", "FINAL")),
        ("lastModifiedDate", ProgramInformation(123, "fixture", 0, 789, "demo", "FINAL")),
        ("lastSavedDate", ProgramInformation(123, "fixture", 456, 0, "demo", "FINAL")),
        ("programState", ProgramInformation(123, "fixture", 456, 789, "demo", "")),
    ],
)
def test_program_list_tolerates_each_nullable_field(
    field: str, expected: ProgramInformation
) -> None:
    other = dict(PROGRAM, name="other")
    client = program_list_client([dict(PROGRAM, **{field: None}), other])
    programs = client.get_program_list()
    assert programs[0] == expected
    assert programs[1].name == "other"


def test_program_list_tolerates_all_nullable_fields_at_once() -> None:
    nulls = {key: None for key in PROGRAM if key != "name"}
    client = program_list_client([dict(PROGRAM, **nulls)])
    assert client.get_program_list() == (ProgramInformation(0, "", 0, 0, "demo", ""),)


def test_program_list_still_requires_name() -> None:
    client = program_list_client([dict(PROGRAM, name=None)])
    with pytest.raises(ProtocolError, match="'name'"):
        client.get_program_list()
