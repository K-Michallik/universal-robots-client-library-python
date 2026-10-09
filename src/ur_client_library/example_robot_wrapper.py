"""Developer-oriented convenience wrapper used by examples and robot tests.

This helper intentionally encodes opinionated startup behavior and is not
intended as a stable production abstraction.
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable, Sequence
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path
from typing import TypeVar

from ur_client_library.exceptions import LifecycleError, URCLTimeoutError
from ur_client_library.helpers import wait_for
from ur_client_library.primary.client import PrimaryClient
from ur_client_library.ur.dashboard_client import (
    DashboardClient,
    DashboardClientPolicy,
)
from ur_client_library.ur.ur_driver import UrDriver, UrDriverConfiguration
from ur_client_library.ur.version_information import VersionInformation

ComponentT = TypeVar("ComponentT")
_LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True, slots=True)
class ExampleRobotWrapperFactories:
    """Construction seam for the wrapper's separately owned clients."""

    primary: Callable[[str], PrimaryClient]
    dashboard: Callable[[str, DashboardClientPolicy | str], DashboardClient]
    driver: Callable[[UrDriverConfiguration], UrDriver]

    @classmethod
    def defaults(cls) -> ExampleRobotWrapperFactories:
        def primary(robot_ip: str) -> PrimaryClient:
            return PrimaryClient(robot_ip)

        def dashboard(robot_ip: str, policy: DashboardClientPolicy | str) -> DashboardClient:
            return DashboardClient(robot_ip, policy)

        return cls(primary, dashboard, UrDriver)


class ExampleRobotWrapper:
    """Opinionated, explicitly non-production robot startup helper."""

    DEFAULT_ROBOT_IP = "192.168.56.101"
    DEFAULT_SCRIPT_RESOURCE = "external_control.urscript"

    # Upstream waits 500 ms, which assumes an already powered robot. Brake
    # release is still in flight when the control program is sent, so a loaded
    # simulator needs considerably longer to attach to the reverse interface.
    PROGRAM_START_TIMEOUT = 10.0

    def __init__(
        self,
        robot_ip: str,
        output_recipe: Sequence[str] | str | Path,
        input_recipe: Sequence[str] | str | Path = (),
        headless_mode: bool = True,
        autostart_program: str = "",
        script_file: str | Path | None = None,
        *,
        factories: ExampleRobotWrapperFactories | None = None,
        protective_stop_retry_delay: float = 5.0,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        if protective_stop_retry_delay < 0:
            raise ValueError("protective_stop_retry_delay must be non-negative")
        if isinstance(output_recipe, (str, Path)):
            output_vector: Sequence[str] = ()
            output_file: str | Path | None = output_recipe
        else:
            output_vector = tuple(output_recipe)
            output_file = None
        if isinstance(input_recipe, (str, Path)):
            input_vector: Sequence[str] = ()
            input_file: str | Path | None = input_recipe
        else:
            input_vector = tuple(input_recipe)
            input_file = None

        self._condition = threading.Condition(threading.Lock())
        self._program_running: bool | None = None
        self._driver_configuration = UrDriverConfiguration(
            robot_ip=robot_ip,
            script_file=script_file,
            output_recipe_file=output_file,
            input_recipe_file=input_file,
            output_recipe=output_vector,
            input_recipe=input_vector,
            handle_program_state=self._handle_program_state,
            headless_mode=headless_mode,
        )
        self._factories = factories
        self._protective_stop_retry_delay = float(protective_stop_retry_delay)
        self._sleep = sleep
        self._headless_mode = headless_mode
        self._autostart_program = autostart_program
        self._primary: PrimaryClient | None = None
        self._dashboard: DashboardClient | None = None
        self._driver: UrDriver | None = None
        self._robot_initialized = False
        self._rtde_started = False
        self._connected = False
        self._closed = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def primary_client(self) -> PrimaryClient:
        return self._require(self._primary, "Primary client")

    @property
    def dashboard_client(self) -> DashboardClient | None:
        return self._dashboard

    @property
    def driver(self) -> UrDriver:
        return self._require(self._driver, "UrDriver")

    def connect(self) -> ExampleRobotWrapper:
        if self._closed:
            raise LifecycleError("A closed ExampleRobotWrapper cannot be connected")
        if self._connected:
            return self
        factories = self._factories or ExampleRobotWrapperFactories.defaults()
        self._factories = factories

        try:
            self._connect_components(factories)
        except BaseException:
            self._cleanup_failed_connect()
            raise
        return self

    def _connect_components(self, factories: ExampleRobotWrapperFactories) -> None:
        primary = factories.primary(self._driver_configuration.robot_ip)
        self._primary = primary
        primary.connect()
        primary.start()
        version = primary.wait_for_version(2.0)

        policy: DashboardClientPolicy | None = None
        if version < VersionInformation(10, 0):
            policy = DashboardClientPolicy.G5
        elif version >= VersionInformation(10, 11):
            policy = DashboardClientPolicy.POLYSCOPE_X
        if policy is not None:
            dashboard = factories.dashboard(self._driver_configuration.robot_ip, policy)
            self._dashboard = dashboard
            dashboard.connect()
            dashboard.set_receive_timeout(10.0)

        self.initialize_robot_with_primary_client()
        driver = factories.driver(self._driver_configuration)
        self._driver = driver
        driver.connect()

        if not self._headless_mode and self._autostart_program:
            self.start_robot_program(self._autostart_program)
        if (self._headless_mode or self._autostart_program) and not self.wait_for_program_running(
            self.PROGRAM_START_TIMEOUT
        ):
            raise URCLTimeoutError(
                "Program did not start; is the robot in remote control mode?",
                self.PROGRAM_START_TIMEOUT,
            )
        self._connected = True

    def _cleanup_failed_connect(self) -> None:
        with suppress(Exception):
            self.close()
        self._primary = None
        self._dashboard = None
        self._driver = None
        self._robot_initialized = False
        self._rtde_started = False
        with self._condition:
            self._program_running = None
        self._closed = False

    def initialize_robot_with_primary_client(self) -> None:
        primary = self.primary_client
        wait_for(
            lambda: primary.robot_mode_data is not None,
            5.0,
        )
        self.clear_protective_stop()
        primary.command_stop()
        primary.command_brake_release()
        _LOGGER.info("Robot ready to start a program")
        self._robot_initialized = True

    def initialize_robot_with_dashboard(self) -> None:
        dashboard = self._dashboard
        if dashboard is None:
            raise LifecycleError("This controller version has no selected dashboard backend")
        self.clear_protective_stop()
        dashboard.stop()
        dashboard.power_off()
        dashboard.power_on()
        dashboard.brake_release()
        self._robot_initialized = True

    def clear_protective_stop(self) -> None:
        primary = self.primary_client
        if not primary.is_protective_stopped:
            return
        if self._dashboard is not None:
            for close_popup in (
                self._dashboard.close_popup,
                self._dashboard.close_safety_popup,
            ):
                with suppress(NotImplementedError):
                    close_popup()
        try:
            primary.command_unlock_protective_stop()
        except TimeoutError:
            self._sleep(self._protective_stop_retry_delay)
            primary.command_unlock_protective_stop()

    def start_rtde_communication(self, consume_data: bool = False) -> None:
        if self._rtde_started:
            return
        self.driver.start_rtde_communication(consume_data)
        self._rtde_started = True

    def _handle_program_state(self, running: bool) -> None:
        if running:
            _LOGGER.info("Robot control program is running and connected to the driver")
        else:
            _LOGGER.info("Robot should be connected - Control program not running yet")
        with self._condition:
            self._program_running = bool(running)
            self._condition.notify_all()

    def wait_for_program_running(self, timeout: float = 0.1) -> bool:
        if timeout < 0:
            raise ValueError("timeout must be non-negative")
        with self._condition:
            return self._condition.wait_for(lambda: self._program_running is True, timeout=timeout)

    def wait_for_program_not_running(self, timeout: float = 0.1) -> bool:
        if timeout < 0:
            raise ValueError("timeout must be non-negative")
        with self._condition:
            return self._condition.wait_for(lambda: self._program_running is False, timeout=timeout)

    def start_robot_program(self, program_file_name: str) -> None:
        dashboard = self._dashboard
        if dashboard is None:
            raise LifecycleError("No dashboard is available for loading a teach-pendant program")
        dashboard.load_program(program_file_name)
        dashboard.play()

    def resend_robot_program(self) -> None:
        if self._headless_mode:
            self.driver.send_robot_program()
            return
        if not self._autostart_program:
            raise LifecycleError("No autostart program was configured")
        self.start_robot_program(self._autostart_program)

    @property
    def is_healthy(self) -> bool:
        with self._condition:
            running = self._program_running
        return self._robot_initialized and running is True

    @staticmethod
    def _require(component: ComponentT | None, name: str) -> ComponentT:
        if component is None:
            raise LifecycleError(f"{name} is unavailable before connect()")
        return component

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._connected = False
        for component in (self._driver, self._dashboard, self._primary):
            if component is None:
                continue
            component.close()

    def __enter__(self) -> ExampleRobotWrapper:
        return self.connect()

    def __exit__(
        self,
        _exc_type: type[BaseException] | None,
        _exc_value: BaseException | None,
        _traceback: object,
    ) -> None:
        self.close()
