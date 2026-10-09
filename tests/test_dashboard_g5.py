from __future__ import annotations

from collections import defaultdict, deque
from collections.abc import Callable
from pathlib import Path

import pytest

from ur_client_library.exceptions import (
    DashboardCommandError,
    LifecycleError,
    URCLTimeoutError,
    VersionMismatchError,
)
from ur_client_library.ur.dashboard_client import (
    DashboardClient,
    DashboardClientPolicy,
    ProgramSaveState,
    ProgramState,
)
from ur_client_library.ur.dashboard_g5 import DashboardClientG5


class FakeSocket:
    def __init__(
        self,
        responses: dict[str, list[str]],
        *,
        greeting: str = "Connected: Universal Robots Dashboard Server\n",
    ) -> None:
        self._incoming = bytearray(greeting.encode())
        self._responses = defaultdict(deque)
        for command, values in responses.items():
            self._responses[command].extend(values)
        self.sent: list[str] = []
        self.timeouts: list[float] = []
        self.address: tuple[str, int] | None = None
        self.closed = False
        self.next_read_error: BaseException | None = None

    def settimeout(self, timeout: float) -> None:
        self.timeouts.append(timeout)

    def connect(self, address: tuple[str, int]) -> None:
        self.address = address

    def sendall(self, data: bytes) -> None:
        command = data.decode()
        self.sent.append(command)
        values = self._responses[command]
        if not values:
            raise AssertionError(f"no fake response configured for {command!r}")
        self._incoming.extend(values.popleft().encode())

    def recv(self, size: int) -> bytes:
        if self.next_read_error is not None:
            error, self.next_read_error = self.next_read_error, None
            raise error
        if not self._incoming:
            raise AssertionError("fake socket has no response bytes")
        result = bytes(self._incoming[:size])
        del self._incoming[:size]
        return result

    def close(self) -> None:
        self.closed = True


def make_client(
    extra_responses: dict[str, list[str]] | None = None,
    *,
    version: str = "5.12.0.1101319",
    sleep: Callable[[float], None] = lambda _seconds: None,
) -> tuple[DashboardClientG5, FakeSocket]:
    responses = {
        "PolyscopeVersion\n": [f"URSoftware {version} (Jun 01 2026)\n"],
    }
    responses.update(extra_responses or {})
    fake = FakeSocket(responses)
    client = DashboardClientG5(
        "192.0.2.10",
        receive_timeout=2.5,
        socket_factory=lambda: fake,
        sleep=sleep,
    )
    return client, fake


def test_constructor_is_side_effect_free_and_connect_reads_banner() -> None:
    calls = 0
    fake = FakeSocket(
        {"PolyscopeVersion\n": ["URSoftware 5.12.0.1101319 (build)\n"]}
    )

    def factory() -> FakeSocket:
        nonlocal calls
        calls += 1
        return fake

    client = DashboardClientG5("robot", socket_factory=factory)
    assert calls == 0

    assert client.connect() is client
    assert calls == 1
    assert fake.address == ("robot", 29999)
    assert fake.sent == ["PolyscopeVersion\n"]
    assert fake.timeouts == [10.0, 1.0]
    assert str(client.polyscope_version) == "5.12.0.1101319"


def test_connect_retries_when_startup_greeting_times_out() -> None:
    first = FakeSocket({}, greeting="")
    first.next_read_error = TimeoutError()
    second = FakeSocket(
        {"PolyscopeVersion\n": ["URSoftware 5.12.0.1101319 (build)\n"]}
    )
    candidates = iter((first, second))
    sleeps: list[float] = []
    client = DashboardClientG5(
        "robot",
        socket_factory=lambda: next(candidates),
        sleep=sleeps.append,
    )

    client.connect(max_attempts=2, retry_interval=0.25)

    assert first.closed
    assert client.is_connected
    assert sleeps == [0.25]


def test_context_manager_and_close_are_idempotent() -> None:
    client, fake = make_client()
    with client as connected:
        assert connected.is_connected
    assert fake.closed
    assert not client.is_connected
    client.close()


def test_connecting_twice_is_a_lifecycle_error() -> None:
    client, _fake = make_client()
    client.connect()
    with pytest.raises(LifecycleError):
        client.connect()


def test_raw_request_is_newline_delimited_and_trimmed() -> None:
    client, fake = make_client({"robotmode\n": ["Robotmode: IDLE \r\n"]})
    client.connect()
    assert client.send_and_receive("robotmode") == "Robotmode: IDLE"
    assert fake.sent[-1] == "robotmode\n"


def test_read_timeout_does_not_disconnect_socket() -> None:
    client, fake = make_client({"robotmode\n": ["unused\n"]})
    client.connect()
    fake.next_read_error = TimeoutError()

    with pytest.raises(URCLTimeoutError):
        client.send_and_receive("robotmode")

    assert client.is_connected
    assert not fake.closed


def test_rejected_response_raises_dashboard_command_error() -> None:
    client, _fake = make_client({"power off\n": ["Cannot power off\n"]})
    client.connect()
    with pytest.raises(DashboardCommandError, match="power off"):
        client.send_request("power off")


def test_power_off_polls_until_expected_mode() -> None:
    client, fake = make_client(
        {
            "power off\n": ["Powering off\n"],
            "robotmode\n": ["Robotmode: POWER_ON\n", "Robotmode: POWER_OFF\n"],
        }
    )
    client.connect()
    response = client.power_off()
    assert response.message == "Powering off"
    assert fake.sent[-2:] == ["robotmode\n", "robotmode\n"]


def test_queries_return_typed_values() -> None:
    client, _fake = make_client(
        {
            "running\n": ["Program running: true\n"],
            "isProgramSaved\n": ["true folder/program.urp\n"],
            "programState\n": ["PAUSED wait_program.urp\n"],
        }
    )
    client.connect()
    assert client.running() is True
    assert client.is_program_saved() == ProgramSaveState(
        saved=True, program_name="folder/program.urp"
    )
    assert client.program_state() == ProgramState(
        state="PAUSED", program_name="wait_program.urp"
    )


def test_version_gate_matches_g5_command_table() -> None:
    client, _fake = make_client(version="5.3.0")
    client.connect()
    with pytest.raises(VersionMismatchError):
        client.safety_status()


def test_assert_version_uses_controller_series() -> None:
    client, _fake = make_client(version="5.12.0")
    client.connect()
    client.assert_version("5.10.0", "3.15.0", "fixture")
    with pytest.raises(VersionMismatchError):
        client.assert_version("5.13.0", "3.15.0", "fixture")
    with pytest.raises(VersionMismatchError):
        client.assert_version("-", "3.15.0", "fixture")


def test_facade_requires_explicit_backend_without_constructing_transport() -> None:
    calls = 0
    fake = FakeSocket(
        {"PolyscopeVersion\n": ["URSoftware 5.12.0.1101319 (build)\n"]}
    )

    def factory() -> FakeSocket:
        nonlocal calls
        calls += 1
        return fake

    client = DashboardClient(
        "robot",
        DashboardClientPolicy.G5,
        socket_factory=factory,
    )
    assert calls == 0
    assert client.policy is DashboardClientPolicy.G5
    client.connect()
    assert calls == 1


def test_popup_title_is_accepted_and_ignored() -> None:
    client, fake = make_client({"popup hello\n": ["showing popup\n"]})
    client.connect()
    response = client.popup("hello", "Ignored title")
    assert response.message == "showing popup"
    assert fake.sent[-1] == "popup hello\n"


def test_download_support_files_is_polyscope_x_only(tmp_path: Path) -> None:
    client, fake = make_client()
    client.connect()
    for operation in (
        client.download_support_files,
        client.command_download_support_files,
    ):
        with pytest.raises(NotImplementedError):
            operation(tmp_path / "support.zip")
    assert fake.sent == ["PolyscopeVersion\n"]
    assert list(tmp_path.iterdir()) == []
