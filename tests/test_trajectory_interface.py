from __future__ import annotations

import struct
from typing import cast

import pytest

from test_control_protocols import FakeServer, unpack
from ur_client_library.control import (
    MotionType,
    MoveCPrimitive,
    MoveJPrimitive,
    MoveLPrimitive,
    OptimoveJPrimitive,
    ReverseInterface,
    SplinePrimitive,
    TrajectoryPointInterface,
    TrajectoryResult,
    TrajectoryStream,
    motion_type_to_string,
    trajectory_result_to_string,
)
from ur_client_library.exceptions import InvalidRangeError
from ur_client_library.types import Pose, Q
from ur_client_library.ur.robot_receive_timeout import RobotReceiveTimeout


def test_target_aware_trajectory_layout_includes_q_near() -> None:
    target = Pose(0.1, -0.2, 0.3, -0.4, 0.5, -0.6, Q(1, 2, 3, 4, 5, 6))
    primitive = MoveLPrimitive(
        target,
        blend_radius=0.25,
        duration=0.500001,
        acceleration=0.7,
        velocity=0.6,
    )

    words = unpack(TrajectoryPointInterface.serialize_motion_primitive(primitive))

    assert len(words) == 21
    assert words[:6] == (100_000, -200_000, 300_000, -400_000, 500_000, -600_000)
    assert words[6:12] == (1_000_000, 2_000_000, 3_000_000, 4_000_000, 5_000_000, 6_000_000)
    assert words[12:18] == (600_000, 700_000, 1_000_000, 0, 0, 0)
    assert words[18:] == (500_001, 250_000, MotionType.MOVEL)


@pytest.mark.parametrize(
    ("via", "target", "motion_type"),
    [
        (Pose(), Pose(), MotionType.MOVEC),
        (Q(0, 0, 0, 0, 0, 0), Q(1, 1, 1, 1, 1, 1), MotionType.MOVEC_JOINT),
        (Q(0, 0, 0, 0, 0, 0), Pose(), MotionType.MOVEC_POSE_JOINT),
        (Pose(), Q(1, 1, 1, 1, 1, 1), MotionType.MOVEC_JOINT_POSE),
    ],
)
def test_movec_target_combinations_select_wire_type(
    via: Q | Pose, target: Q | Pose, motion_type: MotionType
) -> None:
    primitive = MoveCPrimitive(via, target, mode=1)
    assert primitive.type is motion_type
    assert unpack(TrajectoryPointInterface.serialize_motion_primitive(primitive))[-1] == motion_type


def test_spline_layout_uses_microseconds_and_spline_type() -> None:
    primitive = SplinePrimitive(
        (1, 2, 3, 4, 5, 6),
        (0, 0, 0, 0, 0, 0),
        (0.1, 0.2, 0.3, 0.4, 0.5, 0.6),
        duration=1.234567,
    )
    words = unpack(TrajectoryPointInterface.serialize_motion_primitive(primitive))
    assert words[18:] == (1_234_567, 2, MotionType.SPLINE)


def test_invalid_primitive_and_duration_are_rejected() -> None:
    with pytest.raises(InvalidRangeError):
        TrajectoryPointInterface.serialize_motion_primitive(
            OptimoveJPrimitive(Q(0, 0, 0, 0, 0, 0), acceleration_fraction=0)
        )
    with pytest.raises(InvalidRangeError):
        TrajectoryPointInterface.serialize_motion_primitive(
            MoveJPrimitive(Q(0, 0, 0, 0, 0, 0), duration=3_000)
        )


def test_trajectory_boolean_inputs_are_not_coerced_to_integers() -> None:
    interface = TrajectoryPointInterface(server=FakeServer(50003))
    with pytest.raises(TypeError):
        interface.write_trajectory_point(
            (0, 0, 0, 0, 0, 0),
            cartesian=cast(bool, 1),
        )
    with pytest.raises(TypeError):
        motion_type_to_string(cast(MotionType, False))
    with pytest.raises(TypeError):
        trajectory_result_to_string(cast(TrajectoryResult, False))


def test_result_callbacks_accept_fragmented_network_reads() -> None:
    server = FakeServer(50003)
    interface = TrajectoryPointInterface(server=server)
    results: list[TrajectoryResult] = []
    unsubscribe = interface.subscribe_trajectory_result(results.append)
    server.connect_client()

    payload = struct.pack("!ii", TrajectoryResult.CANCELED, TrajectoryResult.SUCCESS)
    server.receive(payload[:3])
    server.receive(payload[3:])
    assert results == [TrajectoryResult.CANCELED, TrajectoryResult.SUCCESS]

    unsubscribe()
    server.receive(struct.pack("!i", TrajectoryResult.FAILURE))
    assert results == [TrajectoryResult.CANCELED, TrajectoryResult.SUCCESS]


def test_stream_helper_sends_exact_successful_point_count_and_keepalive() -> None:
    reverse_server = FakeServer(50001)
    trajectory_server = FakeServer(50003)
    reverse = ReverseInterface(server=reverse_server)
    trajectory = TrajectoryPointInterface(server=trajectory_server)
    reverse_server.connect_client()
    trajectory_server.connect_client()
    stream = TrajectoryStream(
        reverse,
        trajectory,
        robot_receive_timeout=RobotReceiveTimeout.off(),
    )

    stream.start_stream()
    stream.write(MoveJPrimitive(Q(0, 0, 0, 0, 0, 0)))
    stream.write(MoveJPrimitive(Q(1, 1, 1, 1, 1, 1)))
    stream.keepalive()
    assert stream.end_stream() == 2

    reverse_words = [unpack(packet) for packet in reverse_server.writes]
    assert all(words[0] == 0 for words in reverse_words)
    assert reverse_words[0][1:3] == (2, 0)
    assert reverse_words[-2][1:3] == (0, 0)
    assert reverse_words[-1][1:3] == (3, 2)
    assert len(trajectory_server.writes) == 2


def test_stream_cancel_reports_count_only_for_streaming_mode() -> None:
    reverse_server = FakeServer(50001)
    trajectory_server = FakeServer(50003)
    reverse = ReverseInterface(server=reverse_server)
    trajectory = TrajectoryPointInterface(server=trajectory_server)
    reverse_server.connect_client()
    trajectory_server.connect_client()
    stream = TrajectoryStream(reverse, trajectory)
    stream.start_stream()
    stream.write(MoveJPrimitive(Q(0, 0, 0, 0, 0, 0)))

    assert stream.cancel() == 1
    assert unpack(reverse_server.writes[-1])[1:3] == (-1, 1)
