from __future__ import annotations

from dataclasses import replace

import pytest

from ur_client_library.example_robot_wrapper import (
    ExampleRobotWrapper,
    ExampleRobotWrapperFactories,
)
from ur_client_library.exceptions import DashboardCommandError, URCLTimeoutError
from ur_client_library.ur.dashboard_client import DashboardClientPolicy
from ur_client_library.ur.ur_driver import UrDriverConfiguration
from ur_client_library.ur.version_information import VersionInformation


class FakePrimary:
    robot_mode_data = object()

    def __init__(
        self,
        version: VersionInformation,
        events: list[str],
        *,
        protective_stopped: bool = False,
    ) -> None:
        self.version = version
        self.events = events
        self.is_protective_stopped = protective_stopped
        self.unlock_attempts = 0

    def connect(self) -> None:
        self.events.append("primary-connect")

    def start(self) -> None:
        self.events.append("primary-start")

    def wait_for_version(self, _timeout: float) -> VersionInformation:
        return self.version

    def command_stop(self) -> None:
        self.events.append("primary-stop-program")

    def command_brake_release(self) -> None:
        self.events.append("primary-brake-release")

    def command_unlock_protective_stop(self) -> None:
        self.unlock_attempts += 1
        if self.unlock_attempts == 1:
            raise URCLTimeoutError("cooldown", 5.0)
        self.is_protective_stopped = False
        self.events.append("primary-unlock")

    def close(self) -> None:
        self.events.append("primary-close")


class FakeDashboard:
    def __init__(self, policy: DashboardClientPolicy | str, events: list[str]) -> None:
        self.policy = policy
        self.events = events
        self.receive_timeout = 0.0

    def connect(self) -> None:
        self.events.append("dashboard-connect")

    def set_receive_timeout(self, timeout: float) -> None:
        self.receive_timeout = timeout

    def close_popup(self) -> None:
        self.events.append("close-popup")

    def close_safety_popup(self) -> None:
        self.events.append("close-safety-popup")

    def load_program(self, program: str) -> None:
        self.events.append(f"load:{program}")

    def play(self) -> None:
        self.events.append("play")

    def close(self) -> None:
        self.events.append("dashboard-close")


class FakeDriver:
    def __init__(self, config: UrDriverConfiguration, events: list[str]) -> None:
        self.config = config
        self.events = events

    def connect(self) -> FakeDriver:
        self.events.append("driver-connect")
        self.config.handle_program_state(True)
        return self

    def start_rtde_communication(self, consume: bool) -> None:
        self.events.append(f"rtde:{consume}")

    def send_robot_program(self) -> None:
        self.events.append("resend")

    def close(self) -> None:
        self.events.append("driver-close")


def make_factories(
    version: VersionInformation,
    events: list[str],
    captured: dict[str, object],
    *,
    protective_stopped: bool = False,
) -> ExampleRobotWrapperFactories:
    def primary(_ip: str) -> FakePrimary:
        instance = FakePrimary(version, events, protective_stopped=protective_stopped)
        captured["primary"] = instance
        return instance

    def dashboard(_ip: str, policy: DashboardClientPolicy | str) -> FakeDashboard:
        instance = FakeDashboard(policy, events)
        captured["dashboard"] = instance
        return instance

    def driver(config: UrDriverConfiguration) -> FakeDriver:
        instance = FakeDriver(config, events)
        captured["driver"] = instance
        return instance

    return ExampleRobotWrapperFactories(primary, dashboard, driver)


@pytest.mark.parametrize(
    ("version", "expected_policy"),
    [
        (VersionInformation(5, 15), DashboardClientPolicy.G5),
        (VersionInformation(10, 5), None),
        (VersionInformation(10, 11), DashboardClientPolicy.POLYSCOPE_X),
    ],
)
def test_construction_is_deferred_and_dashboard_policy_is_explicit(
    version: VersionInformation,
    expected_policy: DashboardClientPolicy | None,
) -> None:
    events: list[str] = []
    captured: dict[str, object] = {}
    wrapper = ExampleRobotWrapper(
        "192.0.2.10",
        ("timestamp",),
        headless_mode=True,
        factories=make_factories(version, events, captured),
    )
    assert events == []

    wrapper.connect()

    if expected_policy is None:
        assert wrapper.dashboard_client is None
    else:
        dashboard = captured["dashboard"]
        assert isinstance(dashboard, FakeDashboard)
        assert dashboard.policy is expected_policy
        assert dashboard.receive_timeout == 10.0
    assert wrapper.is_healthy
    wrapper.close()
    assert events[-2:] == ["driver-close", "primary-close"] or events[-3:] == [
        "driver-close",
        "dashboard-close",
        "primary-close",
    ]


def test_protective_stop_retries_once_and_autostarts() -> None:
    events: list[str] = []
    sleeps: list[float] = []
    captured: dict[str, object] = {}
    wrapper = ExampleRobotWrapper(
        "192.0.2.11",
        ("timestamp",),
        headless_mode=False,
        autostart_program="external_control.urp",
        factories=make_factories(
            VersionInformation(5, 15),
            events,
            captured,
            protective_stopped=True,
        ),
        sleep=sleeps.append,
    )

    wrapper.connect()

    primary = captured["primary"]
    assert isinstance(primary, FakePrimary)
    assert primary.unlock_attempts == 2
    assert sleeps == [5.0]
    assert events.index("primary-unlock") < events.index("driver-connect")
    assert events[-2:] == ["load:external_control.urp", "play"]
    assert wrapper.wait_for_program_running(0.0)


class RejectingPopupDashboard(FakeDashboard):
    def close_popup(self) -> None:
        self.events.append("close-popup-rejected")
        raise DashboardCommandError("DELETE /popup/v1", '{"closed":false}', status=400)

    def close_safety_popup(self) -> None:
        self.events.append("close-safety-popup-unsupported")
        raise NotImplementedError("too old")


def test_protective_stop_ignores_rejected_and_unsupported_popup_closing() -> None:
    events: list[str] = []
    captured: dict[str, object] = {}
    factories = make_factories(
        VersionInformation(10, 14), events, captured, protective_stopped=True
    )

    def dashboard(_ip: str, policy: DashboardClientPolicy | str) -> FakeDashboard:
        return RejectingPopupDashboard(policy, events)

    wrapper = ExampleRobotWrapper(
        "192.0.2.13",
        ("timestamp",),
        factories=replace(factories, dashboard=dashboard),
        sleep=lambda _seconds: None,
    )

    wrapper.connect()

    assert events.index("close-popup-rejected") < events.index(
        "close-safety-popup-unsupported"
    )
    assert events.index("close-safety-popup-unsupported") < events.index(
        "primary-unlock"
    )


def test_rtde_start_and_resend_are_explicit_and_idempotent() -> None:
    events: list[str] = []
    captured: dict[str, object] = {}
    wrapper = ExampleRobotWrapper(
        "192.0.2.12",
        ("timestamp",),
        factories=make_factories(VersionInformation(10, 5), events, captured),
    ).connect()

    wrapper.start_rtde_communication(True)
    wrapper.start_rtde_communication(False)
    wrapper.resend_robot_program()

    assert events.count("rtde:True") == 1
    assert "rtde:False" not in events
    assert events[-1] == "resend"


def test_failed_connect_cleans_owned_clients_and_allows_retry() -> None:
    events: list[str] = []
    captured: dict[str, object] = {}
    base = make_factories(VersionInformation(10, 5), events, captured)
    attempts = 0

    def driver(config: UrDriverConfiguration) -> FakeDriver:
        nonlocal attempts
        attempts += 1
        instance = FakeDriver(config, events)
        if attempts == 1:
            instance.connect = lambda: (_ for _ in ()).throw(RuntimeError("driver failed"))
        return instance

    wrapper = ExampleRobotWrapper(
        "192.0.2.13",
        ("timestamp",),
        factories=ExampleRobotWrapperFactories(
            primary=base.primary,
            dashboard=base.dashboard,
            driver=driver,
        ),
    )

    with pytest.raises(RuntimeError, match="driver failed"):
        wrapper.connect()
    assert "primary-close" in events
    assert not wrapper.is_connected

    wrapper.connect()
    assert wrapper.is_connected
    wrapper.close()
