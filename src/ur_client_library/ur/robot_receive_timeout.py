"""Robot-side reverse-socket receive timeout configuration."""

from __future__ import annotations

import logging
from dataclasses import dataclass

from ur_client_library.comm.control_mode import ControlMode

_LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True, slots=True)
class RobotReceiveTimeout:
    """A timeout sent to the external-control URScript, in milliseconds."""

    milliseconds: int

    MAX_REALTIME_MILLISECONDS = 200

    def __post_init__(self) -> None:
        if isinstance(self.milliseconds, bool) or not isinstance(self.milliseconds, int):
            raise TypeError("milliseconds must be an integer")
        if self.milliseconds < 0:
            raise ValueError("milliseconds must be non-negative")

    @classmethod
    def from_milliseconds(cls, milliseconds: int = 20) -> RobotReceiveTimeout:
        return cls(milliseconds)

    @classmethod
    def from_seconds(cls, seconds: float = 0.02) -> RobotReceiveTimeout:
        if seconds < 0:
            raise ValueError("seconds must be non-negative")
        return cls(int(seconds * 1000))

    @classmethod
    def off(cls) -> RobotReceiveTimeout:
        return cls(0)

    def verified(self, control_mode: ControlMode, step_time_ms: int) -> int:
        """Clamp this timeout to limits required by ``control_mode``."""
        if step_time_ms <= 0:
            raise ValueError("step_time_ms must be positive")
        if control_mode.is_non_realtime:
            if 0 < self.milliseconds < step_time_ms:
                _LOGGER.warning(
                    "Robot receive timeout %d ms is below step time %d ms; using step time",
                    self.milliseconds,
                    step_time_ms,
                )
                return step_time_ms
            return self.milliseconds
        if control_mode.is_realtime:
            if self.milliseconds < step_time_ms:
                _LOGGER.warning(
                    "Realtime receive timeout %d ms is below step time %d ms; using step time",
                    self.milliseconds,
                    step_time_ms,
                )
                return step_time_ms
            if self.milliseconds > self.MAX_REALTIME_MILLISECONDS:
                _LOGGER.warning(
                    "Realtime receive timeout %d ms exceeds %d ms; clamping",
                    self.milliseconds,
                    self.MAX_REALTIME_MILLISECONDS,
                )
                return self.MAX_REALTIME_MILLISECONDS
            return self.milliseconds
        raise ValueError(f"Control mode {control_mode.name} does not accept a receive timeout")
