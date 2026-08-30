"""Regression tests for controller state restored by the script-command example."""

from __future__ import annotations

import argparse
import time
from types import TracebackType

import pytest
from examples import script_command_interface

from ur_client_library.ur.version_information import VersionInformation

ORIGINAL_TCP_OFFSET = (0.0, 0.0, 0.05, 0.0, 0.0, 0.0)
LEAKED_TCP_OFFSET = (0.0, 0.0, 0.1, 0.0, 0.0, 0.0)


class FakeDriver:
    """Records the controller-wide settings an example changes."""

    control_frequency = 500.0

    def __init__(self) -> None:
        self.version = VersionInformation.from_string("5.26.0")
        self.tcp_offset: tuple[float, ...] = ORIGINAL_TCP_OFFSET
        self.payload: tuple[float, tuple[float, ...]] | None = None
        self.tool_voltage: int | None = None

    def get_data_package_blocking(self) -> dict[str, tuple[float, ...]]:
        return {"tcp_offset": self.tcp_offset}

    def set_tcp_offset(self, offset) -> None:
        self.tcp_offset = tuple(offset)

    def set_target_payload(self, mass, cog, inertia, radius) -> None:
        self.payload = (mass, tuple(cog))

    def set_payload(self, mass, cog) -> None:
        self.payload = (mass, tuple(cog))

    def set_tool_voltage(self, voltage: int) -> None:
        self.tool_voltage = voltage

    def set_friction_scales(self, viscous, coulomb) -> None:
        pass

    def start_tool_contact(self) -> None:
        pass

    def end_tool_contact(self) -> None:
        pass

    def zero_ft_sensor(self) -> None:
        pass

    def write_keepalive(self) -> None:
        pass

    def stop_control(self) -> None:
        pass


class FakeRobot:
    def __init__(self, driver: FakeDriver) -> None:
        self.driver = driver
        self.is_healthy = True

    def __enter__(self) -> FakeRobot:
        return self

    def __exit__(self, exc_type, exc, tb: TracebackType | None) -> None:
        return None

    def start_rtde_communication(self, consume_data: bool = False) -> None:
        pass


@pytest.fixture()
def driver(monkeypatch: pytest.MonkeyPatch) -> FakeDriver:
    fake = FakeDriver()
    monkeypatch.setattr(
        script_command_interface,
        "ExampleRobotWrapper",
        lambda **_kwargs: FakeRobot(fake),
    )
    # Drive the example's command thread through a full pass with no delay, so
    # it really applies the TCP offset and payload the cleanup must undo.
    monkeypatch.setattr(script_command_interface, "_COMMAND_DELAY", 0.0)

    def run_one_pass(_duration, action, **_kwargs) -> None:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            action()
            if fake.tcp_offset == LEAKED_TCP_OFFSET and fake.payload is not None:
                return
            time.sleep(0.01)
        raise AssertionError("the example never applied the settings under test")

    monkeypatch.setattr(script_command_interface, "run_until", run_one_pass)
    return fake


def _args() -> argparse.Namespace:
    return argparse.Namespace(
        robot_ip="192.168.56.101",
        duration=0.0,
        headless=True,
    )


def test_run_restores_the_original_tcp_offset(driver: FakeDriver) -> None:
    """A leaked tool offset shifts every later Cartesian target out of reach."""
    script_command_interface.run(_args())

    assert driver.tcp_offset == ORIGINAL_TCP_OFFSET


def test_run_clears_the_payload_it_set(driver: FakeDriver) -> None:
    script_command_interface.run(_args())

    assert driver.payload == (0.0, (0.0, 0.0, 0.0))
