import pytest

from ur_client_library.comm.control_mode import ControlMode
from ur_client_library.ur.robot_receive_timeout import RobotReceiveTimeout


def test_non_realtime_timeout_can_be_disabled() -> None:
    assert RobotReceiveTimeout.off().verified(ControlMode.IDLE, 2) == 0


def test_non_realtime_timeout_is_at_least_step_time() -> None:
    assert RobotReceiveTimeout(1).verified(ControlMode.FORWARD, 2) == 2


def test_realtime_timeout_is_bounded() -> None:
    assert RobotReceiveTimeout.off().verified(ControlMode.SERVOJ, 2) == 2
    assert RobotReceiveTimeout(500).verified(ControlMode.SERVOJ, 2) == 200


def test_stationary_mode_rejects_timeout() -> None:
    with pytest.raises(ValueError):
        RobotReceiveTimeout(20).verified(ControlMode.STOPPED, 2)
