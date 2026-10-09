"""Shared semantic value types and validation helpers."""

from __future__ import annotations

from collections.abc import Iterable, Iterator, Sequence
from dataclasses import dataclass
from numbers import Real
from typing import TypeAlias, cast

Vector3: TypeAlias = tuple[float, float, float]
Vector6: TypeAlias = tuple[float, float, float, float, float, float]
Vector6Int: TypeAlias = tuple[int, int, int, int, int, int]


def validate_floats(values: Iterable[float], length: int, *, name: str) -> tuple[float, ...]:
    """Validate and normalize a fixed-length sequence of real numbers."""
    normalized = tuple(values)
    if len(normalized) != length:
        raise ValueError(f"{name} must contain exactly {length} values, got {len(normalized)}")
    if any(isinstance(value, bool) or not isinstance(value, Real) for value in normalized):
        raise TypeError(f"{name} values must be real numbers")
    return tuple(float(value) for value in normalized)


def validate_ints(
    values: Iterable[int],
    length: int,
    *,
    name: str,
    minimum: int | None = None,
    maximum: int | None = None,
) -> tuple[int, ...]:
    """Validate a fixed-length sequence of integers and optional bounds."""
    normalized = tuple(values)
    if len(normalized) != length:
        raise ValueError(f"{name} must contain exactly {length} values, got {len(normalized)}")
    if any(isinstance(value, bool) or not isinstance(value, int) for value in normalized):
        raise TypeError(f"{name} values must be integers")
    if minimum is not None and any(value < minimum for value in normalized):
        raise ValueError(f"{name} values must be at least {minimum}")
    if maximum is not None and any(value > maximum for value in normalized):
        raise ValueError(f"{name} values must be at most {maximum}")
    return normalized


def _format_double(value: float) -> str:
    # Matches a default-configured C++ ostream: %g with six significant digits.
    return format(value, "g")


def _format_joint_values(values: Iterable[float]) -> str:
    return "[" + ", ".join(_format_double(value) for value in values) + "]"


@dataclass(frozen=True, slots=True, init=False)
class Q:
    """Six joint positions in radians."""

    values: Vector6

    def __init__(self, *values: float | Sequence[float]) -> None:
        source: Iterable[float]
        if len(values) == 1 and isinstance(values[0], Sequence):
            source = values[0]
        else:
            source = cast(tuple[float, ...], values)
        normalized = validate_floats(source, 6, name="Q")
        object.__setattr__(self, "values", normalized)

    def __iter__(self) -> Iterator[float]:
        return iter(self.values)

    def __len__(self) -> int:
        return 6

    def __getitem__(self, index: int) -> float:
        return self.values[index]

    def to_string(self) -> str:
        """Return the upstream ``Q::toString()`` text, e.g. ``Q([0, 0.1, ...])``."""
        return f"Q({_format_joint_values(self.values)})"

    def __str__(self) -> str:
        return self.to_string()


@dataclass(frozen=True, slots=True)
class Pose:
    """Cartesian position and angle-axis orientation, with an optional IK hint."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    rz: float = 0.0
    q_near: Q | None = None

    def __post_init__(self) -> None:
        normalized = validate_floats(self.values, 6, name="Pose")
        for field, value in zip(("x", "y", "z", "rx", "ry", "rz"), normalized, strict=True):
            object.__setattr__(self, field, value)

    @property
    def values(self) -> Vector6:
        return (self.x, self.y, self.z, self.rx, self.ry, self.rz)

    def __iter__(self) -> Iterator[float]:
        return iter(self.values)

    def __len__(self) -> int:
        return 6

    def __getitem__(self, index: int) -> float:
        return self.values[index]

    def to_string(self) -> str:
        """Return the upstream ``Pose::toString()`` text.

        A ``q_near`` hint is appended on a second line as ``With Q_near([...])``.
        """
        text = (
            f"Pose(x = {_format_double(self.x)}, y = {_format_double(self.y)}, "
            f"z = {_format_double(self.z)}, rx = {_format_double(self.rx)}, "
            f"ry = {_format_double(self.ry)}, rz = {_format_double(self.rz)})"
        )
        if self.q_near is not None:
            text += f"\nWith Q_near({_format_joint_values(self.q_near.values)})"
        return text

    def __str__(self) -> str:
        return self.to_string()


MotionTarget: TypeAlias = Q | Pose


def normalize_target(target: MotionTarget | Sequence[float], *, pose: bool = False) -> MotionTarget:
    """Normalize an explicit motion target without guessing its semantics."""
    if isinstance(target, (Q, Pose)):
        return target
    values = validate_floats(target, 6, name="target")
    if pose:
        return Pose(values[0], values[1], values[2], values[3], values[4], values[5])
    return Q(values)
