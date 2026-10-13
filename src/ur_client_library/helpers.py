"""Small cross-cutting helpers used by protocol implementations."""

from __future__ import annotations

import logging
import math
import os
import time
from collections.abc import Callable, Iterable, Sequence
from typing import TypeVar

from ur_client_library.exceptions import URCLTimeoutError
from ur_client_library.types import MotionTarget
from ur_client_library.ur.datatypes import RobotSeries, RobotType
from ur_client_library.ur.version_information import VersionInformation

_LOGGER = logging.getLogger(__name__)
_T = TypeVar("_T", int, float)
_UINT64_MASK = (1 << 64) - 1


def wait_for(
    condition: Callable[[], bool],
    timeout: float,
    *,
    check_interval: float = 0.05,
) -> None:
    """Wait until ``condition`` succeeds, using a monotonic deadline."""
    if timeout < 0:
        raise ValueError("timeout must be non-negative")
    if check_interval <= 0:
        raise ValueError("check_interval must be positive")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return
        time.sleep(min(check_interval, max(0.0, deadline - time.monotonic())))
    if condition():
        return
    raise URCLTimeoutError("Timeout while waiting for condition", timeout)


def parse_boolean(value: str) -> bool:
    normalized = value.lower()
    if normalized in {"true", "1", "yes", "on"}:
        return True
    if normalized in {"false", "0", "no", "off"}:
        return False
    raise ValueError(
        f"Invalid boolean value {value!r}; expected true/false, 1/0, yes/no, or on/off"
    )


def split_string(value: str, delimiter: str = ",") -> list[str]:
    if not delimiter:
        raise ValueError("delimiter must not be empty")
    return value.split(delimiter)


def clamp_to_unit_range(values: Iterable[_T]) -> tuple[_T, ...]:
    return tuple(max(0, min(1, value)) for value in values)


def robot_series_from_type_and_version(
    robot_type: RobotType,
    version: VersionInformation,
) -> RobotSeries:
    if robot_type in {RobotType.UR3, RobotType.UR5, RobotType.UR10}:
        return RobotSeries.E_SERIES if version.major >= 5 else RobotSeries.CB3
    if robot_type is RobotType.UR16:
        return RobotSeries.E_SERIES if version.major >= 5 else RobotSeries.UNDEFINED
    if robot_type in {
        RobotType.UR15,
        RobotType.UR18,
        RobotType.UR20,
        RobotType.UR30,
        RobotType.UR8_LONG,
    }:
        return RobotSeries.UR_SERIES if version.major >= 5 else RobotSeries.UNDEFINED
    return RobotSeries.UNDEFINED


_ROBOT_TYPES = {
    "ur3": RobotType.UR3,
    "ur3e": RobotType.UR3,
    "ur5": RobotType.UR5,
    "ur5e": RobotType.UR5,
    "ur7e": RobotType.UR5,
    "ur10": RobotType.UR10,
    "ur10e": RobotType.UR10,
    "ur12e": RobotType.UR10,
    "ur16e": RobotType.UR16,
    "ur15": RobotType.UR15,
    "ur18": RobotType.UR18,
    "ur20": RobotType.UR20,
    "ur30": RobotType.UR30,
    "ur8long": RobotType.UR8_LONG,
}


def robot_type_from_string(value: str) -> RobotType:
    try:
        return _ROBOT_TYPES[value]
    except KeyError as exc:
        raise ValueError(f"Unknown robot type: {value}") from exc


def string_from_motion_target(target: MotionTarget) -> str:
    """Return the upstream ``stringFromMotionTarget()`` text of a ``Q`` or ``Pose``."""
    return target.to_string()


def round_half_away_from_zero(value: float) -> int:
    """Match C++ ``std::round`` for control-protocol fixed-point values."""
    return math.floor(value + 0.5) if value >= 0 else math.ceil(value - 0.5)


def try_set_fifo(priority: int) -> bool:
    """Attempt FIFO scheduling for the current worker thread."""
    if not hasattr(os, "sched_setscheduler"):
        _LOGGER.warning("FIFO scheduling is unavailable on this platform")
        return False
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(priority))
    except PermissionError:
        _LOGGER.warning(
            "FIFO scheduling was denied; configure real-time privileges for lower jitter"
        )
        return False
    except OSError as exc:
        _LOGGER.warning("Could not enable FIFO scheduling: %s", exc)
        return False
    return True


def libstdcxx_string_hash(data: str | bytes) -> int:
    """Return libstdc++'s current 64-bit ``std::hash<std::string>`` value."""
    raw = data.encode() if isinstance(data, str) else data
    multiplier = 0xC6A4A7935BD1E995
    value = (0xC70F6907 ^ (len(raw) * multiplier)) & _UINT64_MASK

    full_length = len(raw) - (len(raw) % 8)
    for offset in range(0, full_length, 8):
        chunk = int.from_bytes(raw[offset : offset + 8], "little")
        chunk = (chunk * multiplier) & _UINT64_MASK
        chunk ^= chunk >> 47
        chunk = (chunk * multiplier) & _UINT64_MASK
        value ^= chunk
        value = (value * multiplier) & _UINT64_MASK

    tail = raw[full_length:]
    for index, byte in enumerate(tail):
        value ^= byte << (index * 8)
    if tail:
        value = (value * multiplier) & _UINT64_MASK

    value ^= value >> 47
    value = (value * multiplier) & _UINT64_MASK
    value ^= value >> 47
    return value & _UINT64_MASK


def calibration_hash(
    dh_theta: Sequence[float],
    dh_d: Sequence[float],
    dh_a: Sequence[float],
    dh_alpha: Sequence[float],
) -> str:
    """Calculate the checksum emitted by URCL's ``KinematicsInfo::toHash``."""
    vectors = (dh_theta, dh_d, dh_a, dh_alpha)
    if any(len(vector) != 6 for vector in vectors):
        raise ValueError("Each DH vector must contain exactly six values")
    serialized = "".join(
        format(float(vector[index]), ".6g")
        for index in range(6)
        for vector in vectors
    )
    return f"calib_{libstdcxx_string_hash(serialized)}"
