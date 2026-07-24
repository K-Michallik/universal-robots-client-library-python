"""Target-aware trajectory motion primitives."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, field
from enum import IntEnum
from numbers import Real
from typing import TypeAlias

from ur_client_library.types import MotionTarget, Pose, Q, validate_floats


class MotionType(IntEnum):
    MOVEJ = 0
    MOVEL = 1
    MOVEP = 2
    MOVEC = 3
    OPTIMOVEJ = 4
    OPTIMOVEL = 5
    MOVEJ_POSE = 6
    MOVEL_JOINT = 7
    MOVEP_JOINT = 8
    MOVEC_JOINT = 9
    MOVEC_JOINT_POSE = 10
    MOVEC_POSE_JOINT = 11
    OPTIMOVEJ_POSE = 12
    OPTIMOVEL_JOINT = 13
    SPLINE = 51
    UNKNOWN = 255

    def __str__(self) -> str:
        return self.name


class TrajectorySplineType(IntEnum):
    CUBIC = 1
    QUINTIC = 2


def motion_type_to_string(motion_type: MotionType | int) -> str:
    if isinstance(motion_type, bool):
        raise TypeError("motion_type must not be a bool")
    try:
        return MotionType(motion_type).name
    except ValueError:
        return MotionType.UNKNOWN.name


def _real(value: object, *, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a real number")
    return float(value)


def _target(value: MotionTarget, *, name: str = "target") -> MotionTarget:
    if not isinstance(value, (Q, Pose)):
        raise TypeError(f"{name} must be a Q or Pose")
    return value


@dataclass(slots=True)
class MotionPrimitive:
    blend_radius: float = 0.0
    duration: float = 0.0
    acceleration: float = 1.4
    velocity: float = 1.04
    type: MotionType = field(default=MotionType.UNKNOWN, init=False)

    def __post_init__(self) -> None:
        self.blend_radius = _real(self.blend_radius, name="blend_radius")
        self.duration = _real(self.duration, name="duration")
        self.acceleration = _real(self.acceleration, name="acceleration")
        self.velocity = _real(self.velocity, name="velocity")

    def validate(self) -> bool:
        values = (self.blend_radius, self.duration, self.acceleration, self.velocity)
        return all(math.isfinite(value) for value in values) and all(
            value >= 0 for value in values
        )


class MotionPrimitiveWithTarget(MotionPrimitive):
    _target: MotionTarget

    def __init__(
        self,
        target: MotionTarget,
        blend_radius: float = 0.0,
        duration: float = 0.0,
        acceleration: float = 1.4,
        velocity: float = 1.04,
    ) -> None:
        super().__init__(blend_radius, duration, acceleration, velocity)
        self._target = _target(target)

    @property
    def target(self) -> MotionTarget:
        return self._target

    def set_target(self, target: MotionTarget) -> None:
        self._target = _target(target)


class MoveJPrimitive(MotionPrimitiveWithTarget):
    def __init__(
        self,
        target: MotionTarget,
        blend_radius: float = 0.0,
        duration: float = 0.0,
        acceleration: float = 1.4,
        velocity: float = 1.04,
    ) -> None:
        super().__init__(target, blend_radius, duration, acceleration, velocity)
        self._refresh_type()

    def set_target(self, target: MotionTarget) -> None:
        super().set_target(target)
        self._refresh_type()

    def _refresh_type(self) -> None:
        self.type = MotionType.MOVEJ_POSE if isinstance(self.target, Pose) else MotionType.MOVEJ


class MoveLPrimitive(MotionPrimitiveWithTarget):
    def __init__(
        self,
        target: MotionTarget,
        blend_radius: float = 0.0,
        duration: float = 0.0,
        acceleration: float = 1.4,
        velocity: float = 1.04,
    ) -> None:
        super().__init__(target, blend_radius, duration, acceleration, velocity)
        self._refresh_type()

    def set_target(self, target: MotionTarget) -> None:
        super().set_target(target)
        self._refresh_type()

    def _refresh_type(self) -> None:
        self.type = MotionType.MOVEL if isinstance(self.target, Pose) else MotionType.MOVEL_JOINT


class MovePPrimitive(MotionPrimitiveWithTarget):
    def __init__(
        self,
        target: MotionTarget,
        blend_radius: float = 0.0,
        acceleration: float = 1.4,
        velocity: float = 1.04,
    ) -> None:
        super().__init__(target, blend_radius, 0.0, acceleration, velocity)
        self._refresh_type()

    def set_target(self, target: MotionTarget) -> None:
        super().set_target(target)
        self._refresh_type()

    def _refresh_type(self) -> None:
        self.type = MotionType.MOVEP if isinstance(self.target, Pose) else MotionType.MOVEP_JOINT


class MoveCPrimitive(MotionPrimitiveWithTarget):
    _via: MotionTarget
    mode: int

    def __init__(
        self,
        via: MotionTarget,
        target: MotionTarget,
        blend_radius: float = 0.0,
        acceleration: float = 1.4,
        velocity: float = 1.04,
        mode: int = 0,
    ) -> None:
        self._via = _target(via, name="via")
        if isinstance(mode, bool) or not isinstance(mode, int):
            raise TypeError("mode must be an integer")
        self.mode = mode
        super().__init__(target, blend_radius, 0.0, acceleration, velocity)
        self._refresh_type()

    @property
    def via(self) -> MotionTarget:
        return self._via

    def set_via(self, via: MotionTarget) -> None:
        self._via = _target(via, name="via")
        self._refresh_type()

    def set_target(self, target: MotionTarget) -> None:
        super().set_target(target)
        self._refresh_type()

    def validate(self) -> bool:
        return super().validate() and self.mode in (0, 1)

    def _refresh_type(self) -> None:
        target_pose = isinstance(self.target, Pose)
        via_pose = isinstance(self.via, Pose)
        if target_pose and via_pose:
            self.type = MotionType.MOVEC
        elif not target_pose and not via_pose:
            self.type = MotionType.MOVEC_JOINT
        elif target_pose:
            self.type = MotionType.MOVEC_POSE_JOINT
        else:
            self.type = MotionType.MOVEC_JOINT_POSE


class SplinePrimitive(MotionPrimitive):
    target_positions: tuple[float, ...]
    target_velocities: tuple[float, ...]
    target_accelerations: tuple[float, ...] | None

    def __init__(
        self,
        target_positions: Sequence[float],
        target_velocities: Sequence[float],
        target_accelerations: Sequence[float] | None = None,
        duration: float = 0.0,
    ) -> None:
        super().__init__(duration=duration)
        self.type = MotionType.SPLINE
        self.target_positions = validate_floats(
            target_positions, 6, name="target_positions"
        )
        self.target_velocities = validate_floats(
            target_velocities, 6, name="target_velocities"
        )
        self.target_accelerations = (
            None
            if target_accelerations is None
            else validate_floats(target_accelerations, 6, name="target_accelerations")
        )

    @property
    def spline_type(self) -> TrajectorySplineType:
        return (
            TrajectorySplineType.CUBIC
            if self.target_accelerations is None
            else TrajectorySplineType.QUINTIC
        )

    def validate(self) -> bool:
        values = self.target_positions + self.target_velocities
        if self.target_accelerations is not None:
            values += self.target_accelerations
        return math.isfinite(self.duration) and all(math.isfinite(value) for value in values)


class OptimoveJPrimitive(MotionPrimitiveWithTarget):
    def __init__(
        self,
        target: MotionTarget,
        blend_radius: float = 0.0,
        acceleration_fraction: float = 0.3,
        velocity_fraction: float = 0.3,
    ) -> None:
        super().__init__(
            target, blend_radius, 0.0, acceleration_fraction, velocity_fraction
        )
        self._refresh_type()

    def set_target(self, target: MotionTarget) -> None:
        super().set_target(target)
        self._refresh_type()

    def validate(self) -> bool:
        return (
            super().validate()
            and 0 < self.acceleration <= 1
            and 0 < self.velocity <= 1
        )

    def _refresh_type(self) -> None:
        self.type = (
            MotionType.OPTIMOVEJ_POSE
            if isinstance(self.target, Pose)
            else MotionType.OPTIMOVEJ
        )


class OptimoveLPrimitive(MotionPrimitiveWithTarget):
    def __init__(
        self,
        target: MotionTarget,
        blend_radius: float = 0.0,
        acceleration_fraction: float = 0.3,
        velocity_fraction: float = 0.3,
    ) -> None:
        super().__init__(
            target, blend_radius, 0.0, acceleration_fraction, velocity_fraction
        )
        self._refresh_type()

    def set_target(self, target: MotionTarget) -> None:
        super().set_target(target)
        self._refresh_type()

    def validate(self) -> bool:
        return (
            super().validate()
            and 0 < self.acceleration <= 1
            and 0 < self.velocity <= 1
        )

    def _refresh_type(self) -> None:
        self.type = (
            MotionType.OPTIMOVEL
            if isinstance(self.target, Pose)
            else MotionType.OPTIMOVEL_JOINT
        )


MotionSequence: TypeAlias = Sequence[MotionPrimitive]


@dataclass(frozen=True, slots=True)
class MotionInstruction:
    """Instruction-executor-facing wrapper around a validated primitive."""

    primitive: MotionPrimitive

    def __post_init__(self) -> None:
        if not isinstance(self.primitive, MotionPrimitive):
            raise TypeError("primitive must be a MotionPrimitive")
        if not self.primitive.validate():
            raise ValueError("motion primitive validation failed")
