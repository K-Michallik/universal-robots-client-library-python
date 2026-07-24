from __future__ import annotations

import threading
from collections.abc import Callable, Sequence

import pytest

from ur_client_library.comm.control_mode import ControlMode
from ur_client_library.control._server import ClientToken
from ur_client_library.control.motion_primitives import MotionPrimitive
from ur_client_library.control.reverse_interface import TrajectoryControlMessage
from ur_client_library.control.trajectory_point_interface import TrajectoryResult
from ur_client_library.exceptions import LifecycleError
from ur_client_library.types import Pose, Q
from ur_client_library.ur.instruction_executor import (
    InstructionExecutor,
    TrajectoryCanceledError,
    TrajectoryCommandError,
    TrajectoryDisconnectedError,
)
from ur_client_library.ur.robot_receive_timeout import RobotReceiveTimeout


class FakeDriver:
    def __init__(self) -> None:
        self.done: Callable[[TrajectoryResult], None] | None = None
        self.disconnected: Callable[[ClientToken], None] | None = None
        self.actions: list[TrajectoryControlMessage | int] = []
        self.primitives: list[MotionPrimitive] = []
        self.idle_writes = 0
        self.result_on_primitive: TrajectoryResult | None = None
        self.cancel_result: TrajectoryResult | None = TrajectoryResult.CANCELED

    def subscribe_trajectory_done(
        self, callback: Callable[[TrajectoryResult], None]
    ) -> Callable[[], None]:
        self.done = callback
        return lambda: setattr(self, "done", None)

    def subscribe_trajectory_disconnected(
        self, callback: Callable[[ClientToken], None]
    ) -> Callable[[], None]:
        self.disconnected = callback
        return lambda: setattr(self, "disconnected", None)

    def write_trajectory_control_message(
        self,
        action: TrajectoryControlMessage | int,
        point_number: int = 0,
        robot_receive_timeout: RobotReceiveTimeout | None = None,
    ) -> None:
        del point_number, robot_receive_timeout
        self.actions.append(action)
        if action == TrajectoryControlMessage.CANCEL and self.cancel_result is not None:
            assert self.done is not None
            self.done(self.cancel_result)

    def write_motion_primitive(self, primitive: MotionPrimitive) -> None:
        self.primitives.append(primitive)
        if self.result_on_primitive is not None:
            assert self.done is not None
            self.done(self.result_on_primitive)

    def write_joint_command(
        self,
        target: Q | Pose | Sequence[float],
        control_mode: ControlMode,
        robot_receive_timeout: RobotReceiveTimeout | None = None,
    ) -> None:
        del target, robot_receive_timeout
        assert control_mode is ControlMode.IDLE
        self.idle_writes += 1


def test_execute_success_handles_synchronous_callback() -> None:
    driver = FakeDriver()
    driver.result_on_primitive = TrajectoryResult.SUCCESS
    executor = InstructionExecutor(driver)

    result = executor.move_j(Pose(0.1, 0.2, 0.3, 0, 0, 0))

    assert result is TrajectoryResult.SUCCESS
    primitive = driver.primitives[0]
    assert primitive.target == Pose(0.1, 0.2, 0.3, 0, 0, 0)
    assert primitive.type.name == "MOVEJ_POSE"
    assert driver.actions == [TrajectoryControlMessage.START]
    assert driver.idle_writes == 1


def test_keepalive_and_failure_result() -> None:
    driver = FakeDriver()
    executor = InstructionExecutor(driver)

    def finish_with_failure() -> None:
        assert driver.done is not None
        driver.done(TrajectoryResult.FAILURE)

    timer = threading.Timer(0.13, finish_with_failure)
    timer.start()
    try:
        with pytest.raises(TrajectoryCommandError) as raised:
            executor.move_l(Q(0, 0, 0, 0, 0, 0))
    finally:
        timer.join()

    assert raised.value.result is TrajectoryResult.FAILURE
    assert TrajectoryControlMessage.NOOP in driver.actions
    assert driver.primitives[0].type.name == "MOVEL_JOINT"


def test_disconnect_wakes_waiter() -> None:
    driver = FakeDriver()
    executor = InstructionExecutor(driver)

    def disconnect() -> None:
        assert driver.disconnected is not None
        driver.disconnected(object())

    timer = threading.Timer(0.01, disconnect)
    timer.start()
    try:
        with pytest.raises(TrajectoryDisconnectedError):
            executor.move_p(Pose())
    finally:
        timer.join()


def test_cancel_waits_for_canceled_response() -> None:
    driver = FakeDriver()
    executor = InstructionExecutor(driver)
    execution_error: list[BaseException] = []

    def execute() -> None:
        try:
            executor.move_j(Q(0, 0, 0, 0, 0, 0))
        except BaseException as exc:
            execution_error.append(exc)

    worker = threading.Thread(target=execute)
    worker.start()
    assert threading.Event().wait(0.01) is False
    assert executor.is_trajectory_running

    assert executor.cancel_motion() is TrajectoryResult.CANCELED
    worker.join(timeout=1.0)
    assert isinstance(execution_error[0], TrajectoryCanceledError)
    assert not executor.is_trajectory_running


def test_cancel_without_motion_is_invalid_state() -> None:
    executor = InstructionExecutor(FakeDriver())
    with pytest.raises(LifecycleError):
        executor.cancel_motion()
    executor.close()
    executor.close()
