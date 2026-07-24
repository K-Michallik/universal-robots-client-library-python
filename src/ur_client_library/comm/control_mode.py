"""External-control mode values shared with the bundled URScript."""

from enum import IntEnum


class ControlMode(IntEnum):
    STOPPED = -2
    UNINITIALIZED = -1
    IDLE = 0
    SERVOJ = 1
    SPEEDJ = 2
    FORWARD = 3
    SPEEDL = 4
    POSE = 5
    FREEDRIVE = 6
    TOOL_IN_CONTACT = 7
    TORQUE = 8

    @property
    def is_realtime(self) -> bool:
        return self in {
            ControlMode.SERVOJ,
            ControlMode.SPEEDJ,
            ControlMode.SPEEDL,
            ControlMode.POSE,
            ControlMode.TORQUE,
        }

    @property
    def is_non_realtime(self) -> bool:
        return self in {
            ControlMode.IDLE,
            ControlMode.FORWARD,
            ControlMode.FREEDRIVE,
        }

    @property
    def is_stationary(self) -> bool:
        return self in {
            ControlMode.STOPPED,
            ControlMode.UNINITIALIZED,
            ControlMode.TOOL_IN_CONTACT,
        }
