"""Trajectory-point protocol and host-side stream accounting."""

from __future__ import annotations

import logging
import struct
import threading
from collections.abc import Callable, Sequence
from enum import IntEnum

from ur_client_library.exceptions import (
    InvalidRangeError,
    StreamNotConnectedError,
    UnsupportedMotionTypeError,
)
from ur_client_library.types import Pose, Q, validate_floats

from ._serialization import INT32_MAX, fixed, pack_int32s
from ._server import ClientToken, Server, ServerFactory, socket_server_factory
from .motion_primitives import (
    MotionPrimitive,
    MotionPrimitiveWithTarget,
    MotionType,
    MoveCPrimitive,
    MoveJPrimitive,
    MoveLPrimitive,
    SplinePrimitive,
)
from .reverse_interface import (
    DEFAULT_TRAJECTORY_PORT,
    MULT_JOINTSTATE,
    MULT_TIME,
    ReverseInterface,
    ReverseInterfaceConfig,
    TrajectoryControlMessage,
    Unsubscribe,
)

_LOGGER = logging.getLogger(__name__)


class TrajectoryResult(IntEnum):
    UNKNOWN = -1
    SUCCESS = 0
    CANCELED = 1
    FAILURE = 2


def trajectory_result_to_string(result: TrajectoryResult | int) -> str:
    if isinstance(result, bool):
        raise TypeError("result must not be a bool")
    try:
        return TrajectoryResult(result).name
    except ValueError as exc:
        raise ValueError(f"Illegal trajectory result: {result}") from exc


TrajectoryResultCallback = Callable[[TrajectoryResult], None]


class TrajectoryPointInterface(ReverseInterface):
    """Serialize target-aware motion primitives as fixed 21-word packets."""

    MESSAGE_WORDS = 21
    MAX_GOAL_TIME = INT32_MAX / MULT_TIME

    def __init__(
        self,
        port: int = DEFAULT_TRAJECTORY_PORT,
        *,
        host: str = "0.0.0.0",
        server: Server | None = None,
        server_factory: ServerFactory = socket_server_factory,
    ) -> None:
        super().__init__(
            ReverseInterfaceConfig(port=port, host=host),
            server=server,
            server_factory=server_factory,
        )
        self._trajectory_callbacks: dict[int, TrajectoryResultCallback] = {}
        self._receive_buffers: dict[ClientToken, bytearray] = {}

    def subscribe_trajectory_result(
        self, callback: TrajectoryResultCallback
    ) -> Unsubscribe:
        callback_id = self._add_callback(self._trajectory_callbacks, callback)
        return self._unsubscribe(self._trajectory_callbacks, callback_id)

    def write_motion_primitive(self, primitive: MotionPrimitive) -> None:
        packet = self.serialize_motion_primitive(primitive)
        self._write_packet(packet)

    @classmethod
    def serialize_motion_primitive(cls, primitive: MotionPrimitive) -> bytes:
        if not isinstance(primitive, MotionPrimitive):
            raise TypeError("primitive must be a MotionPrimitive")
        if not primitive.validate():
            raise InvalidRangeError("Motion primitive validation failed")
        if primitive.duration > cls.MAX_GOAL_TIME:
            raise InvalidRangeError(
                f"duration exceeds the wire maximum of {cls.MAX_GOAL_TIME:g} seconds"
            )

        first: tuple[float, ...] = (0.0,) * 6
        second: tuple[float, ...] = (0.0,) * 6
        third: tuple[float, ...] = (0.0,) * 6

        if isinstance(primitive, MoveCPrimitive):
            first = tuple(primitive.target)
            second = tuple(primitive.via)
            third = (
                primitive.velocity,
                primitive.acceleration,
                float(primitive.mode),
                0.0,
                0.0,
                0.0,
            )
        elif isinstance(primitive, SplinePrimitive):
            first = primitive.target_positions
            second = primitive.target_velocities
            third = primitive.target_accelerations or (0.0,) * 6
        elif isinstance(primitive, MotionPrimitiveWithTarget):
            first = tuple(primitive.target)
            q_near = primitive.target.q_near if isinstance(primitive.target, Pose) else None
            second = tuple(q_near) if q_near is not None else (0.0,) * 6
            third = (
                primitive.velocity,
                primitive.acceleration,
                float(q_near is not None),
                0.0,
                0.0,
                0.0,
            )
        else:
            raise UnsupportedMotionTypeError(
                f"Unsupported motion primitive type: {primitive.type}"
            )

        if primitive.type is MotionType.UNKNOWN:
            raise UnsupportedMotionTypeError("Unknown motion primitive type")

        words = [
            fixed(value, MULT_JOINTSTATE, name=f"first_block[{index}]")
            for index, value in enumerate(first)
        ]
        words.extend(
            fixed(value, MULT_JOINTSTATE, name=f"second_block[{index}]")
            for index, value in enumerate(second)
        )
        words.extend(
            fixed(value, MULT_JOINTSTATE, name=f"third_block[{index}]")
            for index, value in enumerate(third)
        )
        words.append(fixed(primitive.duration, MULT_TIME, name="duration"))
        if isinstance(primitive, SplinePrimitive):
            words.append(int(primitive.spline_type))
        else:
            words.append(fixed(primitive.blend_radius, MULT_TIME, name="blend_radius"))
        words.append(int(primitive.type))
        packet = pack_int32s(words)
        if len(packet) != cls.MESSAGE_WORDS * 4:
            raise AssertionError("Trajectory packet must contain exactly 21 int32 words")
        return packet

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
        if not isinstance(cartesian, bool):
            raise TypeError("cartesian must be a bool")
        values = validate_floats(positions, 6, name="positions")
        primitive: MotionPrimitive
        if cartesian:
            primitive = MoveLPrimitive(
                Pose(
                    values[0],
                    values[1],
                    values[2],
                    values[3],
                    values[4],
                    values[5],
                ),
                blend_radius,
                goal_time,
                acceleration,
                velocity,
            )
        else:
            primitive = MoveJPrimitive(
                Q(values),
                blend_radius,
                goal_time,
                acceleration,
                velocity,
            )
        self.write_motion_primitive(primitive)

    def write_trajectory_spline_point(
        self,
        positions: Sequence[float],
        velocities: Sequence[float],
        accelerations: Sequence[float] | None = None,
        *,
        goal_time: float,
    ) -> None:
        self.write_motion_primitive(
            SplinePrimitive(positions, velocities, accelerations, goal_time)
        )

    def _connection_callback(self, client: ClientToken) -> None:
        with self._lock:
            if self._client is not None:
                _LOGGER.error("Ignoring a second trajectory-interface client")
                return
            self._client = client
            self._receive_buffers[client] = bytearray()

    def _disconnection_callback(self, client: ClientToken) -> None:
        with self._lock:
            if self._client != client:
                return
            self._client = None
            self._receive_buffers.pop(client, None)
            callbacks = tuple(self._disconnection_callbacks.values())
        self._invoke_callbacks(callbacks, client)

    def _message_callback(self, client: ClientToken, data: bytes) -> None:
        with self._lock:
            buffer = self._receive_buffers.setdefault(client, bytearray())
            buffer.extend(data)
            raw_results: list[int] = []
            while len(buffer) >= 4:
                raw_results.append(struct.unpack("!i", buffer[:4])[0])
                del buffer[:4]
            callbacks = tuple(self._trajectory_callbacks.values())
        for raw_result in raw_results:
            try:
                result = TrajectoryResult(raw_result)
            except ValueError:
                _LOGGER.warning("Ignoring unknown trajectory result %d", raw_result)
                continue
            self._invoke_callbacks(callbacks, result)


class TrajectoryStream:
    """Coordinate reverse control and trajectory packets with exact counts."""

    def __init__(
        self,
        reverse_interface: ReverseInterface,
        trajectory_interface: TrajectoryPointInterface,
    ) -> None:
        self._reverse = reverse_interface
        self._trajectory = trajectory_interface
        self._lock = threading.RLock()
        self._streaming = False
        self._finite_expected: int | None = None
        self._sent_points = 0
        self._unsubscribe = trajectory_interface.subscribe_trajectory_result(
            self._trajectory_finished
        )

    @property
    def sent_points(self) -> int:
        with self._lock:
            return self._sent_points

    @property
    def is_streaming(self) -> bool:
        with self._lock:
            return self._streaming

    def start(self, point_count: int) -> None:
        if isinstance(point_count, bool) or not isinstance(point_count, int):
            raise TypeError("point_count must be an integer")
        if point_count < 0:
            raise ValueError("point_count must be non-negative")
        self._reverse.write_trajectory_control_message(
            TrajectoryControlMessage.START, point_count
        )
        with self._lock:
            self._streaming = False
            self._finite_expected = point_count
            self._sent_points = 0

    def start_stream(self) -> None:
        self._reverse.write_trajectory_control_message(
            TrajectoryControlMessage.STREAM_START
        )
        with self._lock:
            self._streaming = True
            self._finite_expected = None
            self._sent_points = 0

    def write(self, primitive: MotionPrimitive) -> None:
        with self._lock:
            finite_expected = self._finite_expected
            sent_points = self._sent_points
            streaming = self._streaming
        if not streaming and finite_expected is None:
            raise StreamNotConnectedError("No trajectory has been started")
        if finite_expected is not None and sent_points >= finite_expected:
            raise InvalidRangeError("More trajectory points were written than declared")
        self._trajectory.write_motion_primitive(primitive)
        with self._lock:
            self._sent_points += 1

    def keepalive(self) -> None:
        self._reverse.write_trajectory_control_message(TrajectoryControlMessage.NOOP)

    def end_stream(self) -> int:
        with self._lock:
            if not self._streaming:
                raise StreamNotConnectedError("No streaming trajectory is active")
            point_count = self._sent_points
        self._reverse.write_trajectory_control_message(
            TrajectoryControlMessage.STREAM_END, point_count
        )
        with self._lock:
            self._streaming = False
            self._finite_expected = point_count
        return point_count

    def cancel(self) -> int:
        with self._lock:
            streaming = self._streaming
            point_count = self._sent_points if streaming else 0
        self._reverse.write_trajectory_control_message(
            TrajectoryControlMessage.CANCEL, point_count
        )
        with self._lock:
            self._streaming = False
            self._finite_expected = None
        return point_count

    def close(self) -> None:
        self._unsubscribe()

    def _trajectory_finished(self, _result: TrajectoryResult) -> None:
        with self._lock:
            self._streaming = False
            self._finite_expected = None
