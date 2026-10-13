from __future__ import annotations

from collections.abc import Callable, Sequence
from pathlib import Path
from typing import cast

import pytest

from ur_client_library.comm.control_mode import ControlMode
from ur_client_library.control._server import ClientToken
from ur_client_library.control.motion_primitives import MotionPrimitive
from ur_client_library.control.reverse_interface import (
    MULT_JOINTSTATE,
    FreedriveControlMessage,
    TrajectoryControlMessage,
)
from ur_client_library.control.script_command_interface import ToolContactResult
from ur_client_library.control.script_reader import ScriptData, ScriptReader
from ur_client_library.control.trajectory_point_interface import MULT_VEL_ACC, TrajectoryResult
from ur_client_library.exceptions import LifecycleError
from ur_client_library.primary.packages import ConfigurationData, ErrorCode
from ur_client_library.rtde import DataPackage, RTDEWriter
from ur_client_library.rtde.rtde_writer import WritableStream
from ur_client_library.types import Q
from ur_client_library.ur.robot_receive_timeout import RobotReceiveTimeout
from ur_client_library.ur.ur_driver import (
    PrimaryClientProtocol,
    ReverseInterfaceProtocol,
    RTDEClientProtocol,
    ScriptCommandInterfaceProtocol,
    ScriptReaderProtocol,
    ScriptSenderProtocol,
    TrajectoryInterfaceProtocol,
    UrDriver,
    UrDriverConfiguration,
    UrDriverFactories,
)
from ur_client_library.ur.version_information import VersionInformation


class FakeWritableStream(WritableStream):
    def write(self, data: bytes | bytearray | memoryview) -> int:
        return len(data)


class FakeRTDE:
    version = VersionInformation(5, 15)
    target_frequency = 500.0
    local_ip = "10.0.0.2"

    def __init__(self, events: list[str]) -> None:
        self.events = events
        self.output_recipe: tuple[str, ...] = ()
        self.input_recipe: tuple[str, ...] = ()
        self.writer = RTDEWriter(FakeWritableStream(), ())
        self.package = DataPackage(
            ("timestamp", "actual_q"),
            values={"timestamp": 12.5, "actual_q": (0.0,) * 6},
        )

    def connect(
        self,
        *,
        max_connection_attempts: int = 0,
        reconnection_timeout: float = 10.0,
        max_initialization_attempts: int = 3,
        initialization_timeout: float = 1.0,
    ) -> None:
        del (
            max_connection_attempts,
            reconnection_timeout,
            max_initialization_attempts,
            initialization_timeout,
        )
        self.events.append("rtde-connect")

    def start(self, *, background: bool = True) -> None:
        self.events.append(f"rtde-start:{background}")

    def get_data_package(self, timeout: float | None = None) -> DataPackage | None:
        del timeout
        return self.package.copy()

    def read_data_package(self) -> DataPackage:
        return self.package.copy()

    def start_background_read(self) -> None:
        self.events.append("rtde-background-start")

    def stop_background_read(self) -> None:
        self.events.append("rtde-background-stop")

    def close(self) -> None:
        self.events.append("rtde-close")


class FakePrimary:
    configuration_data = cast(ConfigurationData, object())

    def __init__(self, events: list[str]) -> None:
        self.events = events
        self.scripts: list[str] = []

    def connect(self) -> None:
        self.events.append("primary-connect")

    def start(self) -> None:
        self.events.append("primary-start")

    def close(self) -> None:
        self.events.append("primary-close")

    def send_script(self, program: str) -> None:
        self.scripts.append(program)

    def check_calibration(self, checksum: str, *, timeout: float | None = None) -> bool:
        del timeout
        return checksum == "good"

    def pop_error_codes(self) -> tuple[ErrorCode, ...]:
        return ()

    def send_script_blocking(
        self,
        program: str,
        script_name: str = "",
        *,
        start_timeout: float = 1.0,
        fail_on_warnings: bool = True,
        retry_on_readonly_interface: bool = True,
    ) -> None:
        del (
            script_name,
            start_timeout,
            fail_on_warnings,
            retry_on_readonly_interface,
        )
        self.scripts.append(program)


class FakeReverse:
    is_connected = True

    def __init__(self, events: list[str]) -> None:
        self.events = events

    def start(self) -> None:
        self.events.append("reverse-start")

    def write(
        self,
        positions: Sequence[float] | None = None,
        control_mode: ControlMode | int = ControlMode.IDLE,
        robot_receive_timeout: RobotReceiveTimeout | None = None,
    ) -> None:
        del positions, robot_receive_timeout
        if control_mode is ControlMode.STOPPED:
            self.events.append("stop-control")

    def write_trajectory_control_message(
        self,
        action: TrajectoryControlMessage | int,
        point_number: int = 0,
        robot_receive_timeout: RobotReceiveTimeout | None = None,
    ) -> None:
        del action, point_number, robot_receive_timeout
        return

    def write_freedrive_control_message(
        self,
        action: FreedriveControlMessage | int,
        robot_receive_timeout: RobotReceiveTimeout | None = None,
    ) -> None:
        del action, robot_receive_timeout
        return

    def close(self) -> None:
        self.events.append("reverse-close")


class FakeTrajectory:
    is_connected = True

    def __init__(self, events: list[str]) -> None:
        self.events = events
        self._vel_acc_multiplier = 0
        self.callbacks: list[Callable[[TrajectoryResult], None]] = []

    def start(self) -> None:
        self.events.append("trajectory-start")

    def subscribe_trajectory_result(
        self, callback: Callable[[TrajectoryResult], None]
    ) -> Callable[[], None]:
        self.callbacks.append(callback)

        def unsubscribe() -> None:
            if callback in self.callbacks:
                self.callbacks.remove(callback)

        return unsubscribe

    def subscribe_disconnection(
        self, _callback: Callable[[ClientToken], None]
    ) -> Callable[[], None]:
        return lambda: None

    def write_trajectory_point(
        self,
        positions: Sequence[float],
        *,
        acceleration: float = 1.4,
        velocity: float = 1.05,
        goal_time: float = 0.0,
        blend_radius: float = 0.0,
        cartesian: bool = False,
    ) -> None:
        del positions, acceleration, velocity, goal_time, blend_radius, cartesian

    def write_trajectory_spline_point(
        self,
        positions: Sequence[float],
        velocities: Sequence[float],
        accelerations: Sequence[float] | None = None,
        *,
        goal_time: float = 0.0,
    ) -> None:
        del positions, velocities, accelerations, goal_time

    def write_motion_primitive(self, primitive: MotionPrimitive) -> None:
        del primitive
        return

    @property
    def vel_acc_multiplier(self) -> int:
        return self._vel_acc_multiplier

    def set_vel_acc_multiplier(self, multiplier: int) -> None:
        self._vel_acc_multiplier = multiplier

    def close(self) -> None:
        self.events.append("trajectory-close")


class FakeScriptCommands:
    is_connected = False

    def __init__(self, events: list[str]) -> None:
        self.events = events

    def start(self) -> None:
        self.events.append("commands-start")

    def close(self) -> None:
        self.events.append("commands-close")

    def subscribe_tool_contact_result(
        self, _callback: Callable[[ToolContactResult], None]
    ) -> Callable[[], None]:
        return lambda: None


class FakeReader:
    def __init__(self, captured: dict[str, object]) -> None:
        self.captured = captured

    def read_script_file(self, file_path: str | Path, data: ScriptData | None = None) -> str:
        del file_path
        assert data is not None
        self.captured.update(data)
        return 'textmsg("ready")\n'

    def is_variable_registered(self, key: str) -> bool:
        return key in self.captured


class FakeSender:
    def start(self) -> None:
        return

    def close(self) -> None:
        return


def make_factories(
    events: list[str],
    captured: dict[str, object],
    components: dict[str, object],
    *,
    reader_factory: Callable[[], ScriptReaderProtocol] | None = None,
) -> UrDriverFactories:
    def rtde(
        _ip: str,
        output: tuple[str, ...],
        input_recipe: tuple[str, ...],
        _frequency: float,
        _ignore: bool,
    ) -> RTDEClientProtocol:
        instance = FakeRTDE(events)
        instance.output_recipe = output
        instance.input_recipe = input_recipe
        components["rtde"] = instance
        return instance

    def primary(_ip: str) -> PrimaryClientProtocol:
        instance = FakePrimary(events)
        components["primary"] = instance
        return instance

    def reverse(
        _port: int,
        _callback: Callable[[bool], None],
        _step: int,
        _version: VersionInformation,
    ) -> ReverseInterfaceProtocol:
        instance = FakeReverse(events)
        components["reverse"] = instance
        return instance

    def trajectory(_port: int) -> TrajectoryInterfaceProtocol:
        instance = FakeTrajectory(events)
        components["trajectory"] = instance
        return instance

    return UrDriverFactories(
        rtde=rtde,
        primary=primary,
        reverse=reverse,
        trajectory=trajectory,
        script_command=lambda _port, _version: cast(
            ScriptCommandInterfaceProtocol, FakeScriptCommands(events)
        ),
        script_reader=reader_factory or (lambda: FakeReader(captured)),
        script_sender=lambda _port, _program: cast(ScriptSenderProtocol, FakeSender()),
    )


def test_connect_is_deferred_and_uses_recipe_vectors(tmp_path: Path) -> None:
    events: list[str] = []
    captured: dict[str, object] = {}
    components: dict[str, object] = {}
    script = tmp_path / "external_control.urscript"
    script.write_text("unused")
    config = UrDriverConfiguration(
        robot_ip="192.0.2.1",
        script_file=script,
        output_recipe=("timestamp", "actual_q"),
        input_recipe=("speed_slider_mask",),
        output_recipe_file=tmp_path / "missing-output",
        input_recipe_file=tmp_path / "missing-input",
        headless_mode=True,
    )
    driver = UrDriver(config, factories=make_factories(events, captured, components))

    assert events == []
    driver.connect()

    assert events[:3] == ["rtde-connect", "reverse-start", "trajectory-start"]
    assert driver.output_recipe == ("timestamp", "actual_q")
    assert driver.input_recipe == ("speed_slider_mask",)
    assert captured["SERVER_IP_REPLACE"] == "10.0.0.2"
    assert captured["ROBOT_SOFTWARE_VERSION"] == VersionInformation(5, 15)
    primary = components["primary"]
    assert isinstance(primary, FakePrimary)
    assert primary.scripts[0].startswith("stop program\ndef externalControl():")


def test_fallback_script_and_cleanup_order(tmp_path: Path) -> None:
    events: list[str] = []
    captured: dict[str, object] = {}
    components: dict[str, object] = {}
    script = tmp_path / "script"
    script.write_text("unused")
    driver = UrDriver(
        UrDriverConfiguration(
            robot_ip="192.0.2.2",
            script_file=script,
            output_recipe=("timestamp",),
            headless_mode=True,
        ),
        factories=make_factories(events, captured, components),
    ).connect()

    driver.set_payload(1.5, (0.1, 0.2, 0.3))
    driver.write_joint_command(Q(0, 0, 0, 0, 0, 0), ControlMode.IDLE)
    primary = components["primary"]
    assert isinstance(primary, FakePrimary)
    assert "set_payload(1.5, [0.1, 0.2, 0.3])" in primary.scripts[-1]

    driver.close()
    assert events[-6:] == [
        "stop-control",
        "commands-close",
        "trajectory-close",
        "reverse-close",
        "primary-close",
        "rtde-close",
    ]
    driver.close()


def test_callback_unsubscribe_is_idempotent(tmp_path: Path) -> None:
    events: list[str] = []
    captured: dict[str, object] = {}
    components: dict[str, object] = {}
    script = tmp_path / "script"
    script.write_text("unused")
    driver = UrDriver(
        UrDriverConfiguration(
            robot_ip="192.0.2.3",
            script_file=script,
            output_recipe=("timestamp",),
            headless_mode=True,
        ),
        factories=make_factories(events, captured, components),
    ).connect()
    unsubscribe = driver.subscribe_trajectory_done(lambda _result: None)
    trajectory = components["trajectory"]
    assert isinstance(trajectory, FakeTrajectory)
    assert len(trajectory.callbacks) == 1
    unsubscribe()
    unsubscribe()
    assert trajectory.callbacks == []
    driver.close()


def test_rtde_reads_return_data_packages_without_dynamic_adapters(
    tmp_path: Path,
) -> None:
    events: list[str] = []
    captured: dict[str, object] = {}
    components: dict[str, object] = {}
    script = tmp_path / "script"
    script.write_text("unused")
    driver = UrDriver(
        UrDriverConfiguration(
            robot_ip="192.0.2.8",
            script_file=script,
            output_recipe=("timestamp", "actual_q"),
            headless_mode=True,
        ),
        factories=make_factories(events, captured, components),
    ).connect()

    driver.start_rtde_communication(read_in_background=True)
    polled = driver.get_data_package(timeout=0.1)
    blocking = driver.get_data_package_blocking()

    assert isinstance(polled, DataPackage)
    assert isinstance(blocking, DataPackage)
    assert polled["timestamp"] == 12.5
    assert blocking["actual_q"] == (0.0,) * 6
    assert "rtde-start:True" in events
    driver.close()


def test_packaged_external_control_script_is_loaded_with_importlib_resources() -> None:
    events: list[str] = []
    captured: dict[str, object] = {}
    components: dict[str, object] = {}
    driver = UrDriver(
        UrDriverConfiguration(
            robot_ip="192.0.2.4",
            output_recipe=("timestamp",),
            headless_mode=True,
        ),
        factories=make_factories(
            events,
            captured,
            components,
            reader_factory=ScriptReader,
        ),
    ).connect()

    primary = components["primary"]
    assert isinstance(primary, FakePrimary)
    assert "def externalControl():" in primary.scripts[0]
    assert "# HEADER_BEGIN" in primary.scripts[0]
    assert "MULT_jointstate = 1000000" in primary.scripts[0]
    assert "MULT_velacc = 100000000" in primary.scripts[0]
    assert "10.0.0.2" in primary.scripts[0]
    assert driver.vel_acc_multiplier == MULT_VEL_ACC
    trajectory = components["trajectory"]
    assert isinstance(trajectory, FakeTrajectory)
    assert trajectory.vel_acc_multiplier == MULT_VEL_ACC
    driver.close()


def test_script_without_vel_acc_placeholder_falls_back_to_legacy_encoding(
    tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    events: list[str] = []
    captured: dict[str, object] = {}
    components: dict[str, object] = {}
    script = tmp_path / "legacy.urscript"
    script.write_text("MULT_jointstate = {{JOINT_STATE_REPLACE}}\n")
    driver = UrDriver(
        UrDriverConfiguration(
            robot_ip="192.0.2.4",
            script_file=script,
            output_recipe=("timestamp",),
            headless_mode=True,
        ),
        factories=make_factories(events, captured, components, reader_factory=ScriptReader),
    )

    with caplog.at_level("WARNING", logger="ur_client_library.ur.ur_driver"):
        driver.connect()

    assert driver.vel_acc_multiplier == MULT_JOINTSTATE
    assert any("VEL_ACC_REPLACE" in record.getMessage() for record in caplog.records)
    driver.close()


def test_vel_acc_multiplier_requires_connect() -> None:
    driver = UrDriver(
        UrDriverConfiguration(robot_ip="192.0.2.4", output_recipe=("timestamp",))
    )
    with pytest.raises(LifecycleError):
        _ = driver.vel_acc_multiplier


def test_rtde_initialization_attempts_must_match_implemented_client_contract() -> None:
    with pytest.raises(ValueError, match="greater than zero"):
        UrDriverConfiguration(
            robot_ip="192.0.2.5",
            output_recipe=("timestamp",),
            rtde_initialization_attempts=0,
        )


def test_default_factories_match_subsystem_constructor_signatures() -> None:
    factories = UrDriverFactories.defaults()
    version = VersionInformation(5, 15)
    components = (
        factories.rtde("192.0.2.6", ("timestamp",), (), 0.0, False),
        factories.primary("192.0.2.6"),
        factories.reverse(0, lambda _running: None, 2, version),
        factories.trajectory(0),
        factories.script_command(0, version),
        factories.script_reader(),
        factories.script_sender(0, 'textmsg("fixture")\n'),
    )

    assert all(component is not None for component in components)
    for component in components:
        close = getattr(component, "close", None)
        if callable(close):
            close()


def test_failed_connect_cleans_partial_components_and_allows_retry(
    tmp_path: Path,
) -> None:
    events: list[str] = []
    captured: dict[str, object] = {}
    components: dict[str, object] = {}
    script = tmp_path / "script"
    script.write_text("unused")

    class FlakyReader(FakeReader):
        attempts = 0

        def read_script_file(self, path: str | Path, data: ScriptData | None = None) -> str:
            self.attempts += 1
            if self.attempts == 1:
                raise RuntimeError("render failed")
            return super().read_script_file(path, data)

    reader = FlakyReader(captured)
    driver = UrDriver(
        UrDriverConfiguration(
            robot_ip="192.0.2.7",
            script_file=script,
            output_recipe=("timestamp",),
            headless_mode=True,
        ),
        factories=make_factories(
            events,
            captured,
            components,
            reader_factory=lambda: reader,
        ),
    )

    with pytest.raises(RuntimeError, match="render failed"):
        driver.connect()
    assert not driver.is_connected
    assert "rtde-close" in events
    assert "primary-close" in events

    driver.connect()
    assert driver.is_connected
    driver.close()
