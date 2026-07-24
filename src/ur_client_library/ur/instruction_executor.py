"""Blocking execution of finite trajectory instruction sequences."""

from __future__ import annotations

import logging
import threading
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Protocol, TypeAlias

from ur_client_library.comm.control_mode import ControlMode
from ur_client_library.control._server import ClientToken
from ur_client_library.control.motion_primitives import MotionPrimitive
from ur_client_library.control.reverse_interface import TrajectoryControlMessage
from ur_client_library.control.trajectory_point_interface import TrajectoryResult
from ur_client_library.exceptions import (
    CommandError,
    LifecycleError,
    URCLConnectionError,
    URCLTimeoutError,
)
from ur_client_library.types import MotionTarget, Pose, Q, normalize_target
from ur_client_library.ur.robot_receive_timeout import RobotReceiveTimeout

_LOGGER = logging.getLogger(__name__)
Unsubscribe: TypeAlias = Callable[[], None]


class InstructionDriver(Protocol):
    def subscribe_trajectory_done(
        self, callback: Callable[[TrajectoryResult], None]
    ) -> Unsubscribe: ...

    def subscribe_trajectory_disconnected(
        self, callback: Callable[[ClientToken], None]
    ) -> Unsubscribe: ...

    def write_trajectory_control_message(
        self,
        action: TrajectoryControlMessage | int,
        point_number: int = 0,
        robot_receive_timeout: RobotReceiveTimeout | None = None,
    ) -> None: ...

    def write_motion_primitive(self, primitive: MotionPrimitive) -> None: ...

    def write_joint_command(
        self,
        target: Q,
        control_mode: ControlMode,
        robot_receive_timeout: RobotReceiveTimeout,
    ) -> None: ...


class TrajectoryCommandError(CommandError):
    """A finite trajectory finished with an unsuccessful result."""

    def __init__(self, message: str, result: TrajectoryResult | None = None) -> None:
        self.result = result
        super().__init__(message)


class TrajectoryCanceledError(TrajectoryCommandError):
    """A trajectory was canceled before completion."""


class TrajectoryDisconnectedError(TrajectoryCommandError):
    """The trajectory interface disconnected during execution."""


@dataclass(frozen=True, slots=True)
class MotionPrimitiveFactories:
    """Construction seam for the current target-aware primitive classes."""

    move_j: Callable[[MotionTarget, float, float, float, float], MotionPrimitive]
    move_l: Callable[[MotionTarget, float, float, float, float], MotionPrimitive]
    move_p: Callable[[MotionTarget, float, float, float], MotionPrimitive]
    move_c: Callable[
        [MotionTarget, MotionTarget, float, float, float, int],
        MotionPrimitive,
    ]
    optimove_j: Callable[[MotionTarget, float, float, float], MotionPrimitive]
    optimove_l: Callable[[MotionTarget, float, float, float], MotionPrimitive]

    @classmethod
    def defaults(cls) -> MotionPrimitiveFactories:
        from ur_client_library.control.motion_primitives import (
            MoveCPrimitive,
            MoveJPrimitive,
            MoveLPrimitive,
            MovePPrimitive,
            OptimoveJPrimitive,
            OptimoveLPrimitive,
        )

        return cls(
            MoveJPrimitive,
            MoveLPrimitive,
            MovePPrimitive,
            MoveCPrimitive,
            OptimoveJPrimitive,
            OptimoveLPrimitive,
        )


class InstructionExecutor:
    """Execute motion primitives while maintaining the reverse-socket keepalive."""

    KEEPALIVE_INTERVAL = 0.1
    CANCEL_RESPONSE_TIMEOUT = 0.2

    def __init__(
        self,
        driver: InstructionDriver,
        *,
        primitive_factories: MotionPrimitiveFactories | None = None,
    ) -> None:
        if driver is None:
            raise TypeError("driver must not be None")
        self._driver = driver
        self._primitive_factories = primitive_factories
        self._condition = threading.Condition(threading.Lock())
        self._trajectory_running = False
        self._cancel_requested = False
        self._disconnected = False
        self._result: TrajectoryResult | None = None
        self._closed = False
        self._unsubscribe_done = driver.subscribe_trajectory_done(self._trajectory_done)
        self._unsubscribe_disconnected = driver.subscribe_trajectory_disconnected(
            self._trajectory_disconnected
        )

    @property
    def is_trajectory_running(self) -> bool:
        with self._condition:
            return self._trajectory_running

    def _trajectory_done(self, result: TrajectoryResult) -> None:
        with self._condition:
            if not self._trajectory_running:
                return
            self._result = result
            self._trajectory_running = False
            self._condition.notify_all()

    def _trajectory_disconnected(self, _client: ClientToken) -> None:
        with self._condition:
            if not self._trajectory_running:
                return
            self._disconnected = True
            self._trajectory_running = False
            self._condition.notify_all()

    def _wait_for_trajectory_completion(self) -> bool:
        with self._condition:
            if not self._trajectory_running:
                return True
            self._condition.wait(self.KEEPALIVE_INTERVAL)
            return not self._trajectory_running

    def _completion_state(
        self,
    ) -> tuple[TrajectoryResult | None, bool, bool]:
        with self._condition:
            return self._result, self._disconnected, self._cancel_requested

    def execute_motion(self, motion_sequence: Sequence[MotionPrimitive]) -> TrajectoryResult:
        """Execute one finite sequence and return its successful result enum."""
        if self._closed:
            raise LifecycleError("InstructionExecutor is closed")
        sequence = tuple(motion_sequence)
        for primitive in sequence:
            if not primitive.validate():
                raise TrajectoryCommandError("Motion primitive validation failed")

        with self._condition:
            if self._trajectory_running:
                raise LifecycleError("A trajectory is already running")
            self._trajectory_running = True
            self._cancel_requested = False
            self._disconnected = False
            self._result = None

        try:
            self._driver.write_trajectory_control_message(
                TrajectoryControlMessage.START, len(sequence)
            )
            for primitive in sequence:
                self._driver.write_motion_primitive(primitive)

            while not self._wait_for_trajectory_completion():
                self._driver.write_trajectory_control_message(TrajectoryControlMessage.NOOP)
        except BaseException:
            with self._condition:
                self._trajectory_running = False
                self._condition.notify_all()
            raise

        result, disconnected, canceled = self._completion_state()

        # Keep the external-control program alive after a trajectory. A failure
        # here does not replace the more meaningful trajectory result.
        try:
            self._driver.write_joint_command(
                Q(0, 0, 0, 0, 0, 0),
                ControlMode.IDLE,
                RobotReceiveTimeout.off(),
            )
        except (CommandError, URCLConnectionError):
            _LOGGER.debug("Could not send post-trajectory idle command", exc_info=True)

        if disconnected:
            raise TrajectoryDisconnectedError(
                "Trajectory interface disconnected during execution", result
            )
        if canceled or result is TrajectoryResult.CANCELED:
            raise TrajectoryCanceledError("Trajectory was canceled", result)
        if result is TrajectoryResult.SUCCESS:
            return result
        raise TrajectoryCommandError(f"Trajectory execution failed with result {result!r}", result)

    def cancel_motion(self) -> TrajectoryResult:
        """Cancel the active motion and return the controller's canceled result."""
        with self._condition:
            if not self._trajectory_running:
                raise LifecycleError("No trajectory is running")
            self._cancel_requested = True

        try:
            self._driver.write_trajectory_control_message(
                TrajectoryControlMessage.CANCEL,
                -1,
                RobotReceiveTimeout.from_milliseconds(2000),
            )
        except BaseException:
            with self._condition:
                self._trajectory_running = False
                self._condition.notify_all()
            raise

        with self._condition:
            received = self._condition.wait_for(
                lambda: not self._trajectory_running,
                timeout=self.CANCEL_RESPONSE_TIMEOUT,
            )
            result = self._result
            if not received:
                self._trajectory_running = False
                self._condition.notify_all()
        if not received:
            raise URCLTimeoutError(
                "Timed out waiting for trajectory cancel response",
                self.CANCEL_RESPONSE_TIMEOUT,
            )
        if self._disconnected:
            raise TrajectoryDisconnectedError(
                "Trajectory interface disconnected while canceling", result
            )
        if result is not TrajectoryResult.CANCELED:
            raise TrajectoryCommandError(
                f"Expected a canceled trajectory result, got {result!r}", result
            )
        assert result is not None
        return result

    def move_j(
        self,
        target: MotionTarget | Sequence[float],
        acceleration: float = 1.4,
        velocity: float = 1.04,
        time: float = 0.0,
        blend_radius: float = 0.0,
    ) -> TrajectoryResult:
        normalized = self._normalize_target(target, pose_default=False)
        primitive = self._factories.move_j(normalized, blend_radius, time, acceleration, velocity)
        return self.execute_motion((primitive,))

    def move_l(
        self,
        target: MotionTarget | Sequence[float],
        acceleration: float = 1.4,
        velocity: float = 1.04,
        time: float = 0.0,
        blend_radius: float = 0.0,
    ) -> TrajectoryResult:
        normalized = self._normalize_target(target, pose_default=True)
        primitive = self._factories.move_l(normalized, blend_radius, time, acceleration, velocity)
        return self.execute_motion((primitive,))

    def move_p(
        self,
        target: MotionTarget | Sequence[float],
        acceleration: float = 1.4,
        velocity: float = 1.04,
        blend_radius: float = 0.0,
    ) -> TrajectoryResult:
        normalized = self._normalize_target(target, pose_default=True)
        primitive = self._factories.move_p(normalized, blend_radius, acceleration, velocity)
        return self.execute_motion((primitive,))

    def move_c(
        self,
        via: MotionTarget | Sequence[float],
        target: MotionTarget | Sequence[float],
        acceleration: float = 1.4,
        velocity: float = 1.04,
        blend_radius: float = 0.0,
        mode: int = 0,
    ) -> TrajectoryResult:
        via_target = self._normalize_target(via, pose_default=True)
        normalized = self._normalize_target(target, pose_default=True)
        primitive = self._factories.move_c(
            via_target, normalized, blend_radius, acceleration, velocity, mode
        )
        return self.execute_motion((primitive,))

    def optimove_j(
        self,
        target: MotionTarget | Sequence[float],
        acceleration_fraction: float = 0.3,
        velocity_fraction: float = 0.3,
        blend_radius: float = 0.0,
    ) -> TrajectoryResult:
        normalized = self._normalize_target(target, pose_default=False)
        primitive = self._factories.optimove_j(
            normalized, blend_radius, acceleration_fraction, velocity_fraction
        )
        return self.execute_motion((primitive,))

    def optimove_l(
        self,
        target: MotionTarget | Sequence[float],
        acceleration_fraction: float = 0.3,
        velocity_fraction: float = 0.3,
        blend_radius: float = 0.0,
    ) -> TrajectoryResult:
        normalized = self._normalize_target(target, pose_default=True)
        primitive = self._factories.optimove_l(
            normalized, blend_radius, acceleration_fraction, velocity_fraction
        )
        return self.execute_motion((primitive,))

    @property
    def _factories(self) -> MotionPrimitiveFactories:
        if self._primitive_factories is None:
            self._primitive_factories = MotionPrimitiveFactories.defaults()
        return self._primitive_factories

    @staticmethod
    def _normalize_target(
        target: MotionTarget | Sequence[float], *, pose_default: bool
    ) -> MotionTarget:
        if isinstance(target, (Q, Pose)):
            return target
        return normalize_target(target, pose=pose_default)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._unsubscribe_done()
        self._unsubscribe_disconnected()
        with self._condition:
            if self._trajectory_running:
                self._disconnected = True
                self._trajectory_running = False
                self._condition.notify_all()

    def __enter__(self) -> InstructionExecutor:
        if self._closed:
            raise LifecycleError("InstructionExecutor is closed")
        return self

    def __exit__(
        self,
        _exc_type: type[BaseException] | None,
        _exc_value: BaseException | None,
        _traceback: object,
    ) -> None:
        self.close()
