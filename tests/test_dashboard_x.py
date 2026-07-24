from __future__ import annotations

import json
from collections.abc import Callable
from pathlib import Path

import httpx
import pytest

from ur_client_library.exceptions import DashboardCommandError, LifecycleError
from ur_client_library.ur.dashboard_client import (
    DashboardClient,
    DashboardClientPolicy,
    ProgramInformation,
    ProgramState,
)
from ur_client_library.ur.dashboard_x import DashboardClientX

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
