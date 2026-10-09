from __future__ import annotations

import struct
from typing import cast

import pytest

from test_control_protocols import FakeServer, unpack
from ur_client_library.control import (
    MULT_JOINTSTATE,
    MULT_VEL_ACC,
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
from ur_client_library.control._serialization import INT32_MAX
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
    assert words[:6] == (1_000_000, 2_000_000, 3_000_000, 4_000_000, 5_000_000, 6_000_000)
    assert words[12:18] == (
        10_000_000,
        20_000_000,
        30_000_000,
        40_000_000,
        50_000_000,
        60_000_000,
    )


def test_spline_near_zero_vel_acc_survive_encoding() -> None:
    velocities = (1e-5, -1e-5, 3.4e-6, -3.4e-6, 9.9e-5, -9.9e-5)
    accelerations = (2.3e-5, -2.3e-5, 7e-6, -7e-6, 5.5e-5, -5.5e-5)
    primitive = SplinePrimitive((0,) * 6, velocities, accelerations, duration=0.02)
    words = unpack(TrajectoryPointInterface.serialize_motion_primitive(primitive))

    resolution = 1 / MULT_VEL_ACC
    for sent, received in zip((*velocities, *accelerations), words[6:18], strict=True):
        assert received != 0
        assert sent == pytest.approx(received / MULT_VEL_ACC, abs=resolution / 2)

    tiny = SplinePrimitive((0,) * 6, (1e-7,) * 6, (1e-7,) * 6, duration=0.02)
    tiny_words = unpack(TrajectoryPointInterface.serialize_motion_primitive(tiny))
    assert tiny_words[6:18] == (10,) * 12
    legacy_words = unpack(
        TrajectoryPointInterface.serialize_motion_primitive(
            tiny, vel_acc_multiplier=MULT_JOINTSTATE
        )
    )
    assert legacy_words[6:18] == (0,) * 12


def test_spline_vel_acc_out_of_range_is_rejected() -> None:
    max_vel_acc = INT32_MAX / MULT_VEL_ACC
    in_range = (0, 0, 0, 0, 0, max_vel_acc - 1.0)
    out_of_range = (0, 0, 0, 0, 0, -(max_vel_acc + 1.0))

    with pytest.raises(InvalidRangeError, match="velocity or acceleration out of range"):
        TrajectoryPointInterface.serialize_motion_primitive(
            SplinePrimitive((0,) * 6, out_of_range, in_range, duration=0.02)
        )
    with pytest.raises(InvalidRangeError, match="velocity or acceleration out of range"):
        TrajectoryPointInterface.serialize_motion_primitive(
            SplinePrimitive((0,) * 6, in_range, out_of_range, duration=0.02)
        )
    TrajectoryPointInterface.serialize_motion_primitive(
        SplinePrimitive((0,) * 6, in_range, in_range, duration=0.02)
    )
    # The legacy multiplier has a correspondingly larger range.
    TrajectoryPointInterface.serialize_motion_primitive(
        SplinePrimitive((0,) * 6, out_of_range, None, duration=0.02),
        vel_acc_multiplier=MULT_JOINTSTATE,
    )


def test_legacy_vel_acc_multiplier_encodes_spline_like_joint_state() -> None:
    server = FakeServer(50003)
    interface = TrajectoryPointInterface(server=server)
    server.connect_client()
    interface.set_vel_acc_multiplier(MULT_JOINTSTATE)
    assert interface.vel_acc_multiplier == MULT_JOINTSTATE

    positions = (1.2, 3.1, 2.2, -1.4, -2.1, -3.2)
    velocities = (2.2, 2.1, 3.2, -2.4, -3.1, -2.2)
    accelerations = (3.2, 1.1, 1.2, -3.4, -1.1, -1.2)
    interface.write_trajectory_spline_point(
        positions, velocities, accelerations, goal_time=0.5
    )

    words = unpack(server.writes[-1])
    assert words[:18] == tuple(
        round(value * MULT_JOINTSTATE) for value in (*positions, *velocities, *accelerations)
    )


def test_set_vel_acc_multiplier_rejects_invalid_values() -> None:
    interface = TrajectoryPointInterface(server=FakeServer(50003))
    assert interface.vel_acc_multiplier == MULT_VEL_ACC
    for invalid in (0, -1, INT32_MAX + 1):
        with pytest.raises(InvalidRangeError):
            interface.set_vel_acc_multiplier(invalid)
    with pytest.raises(TypeError):
        interface.set_vel_acc_multiplier(cast(int, True))
    with pytest.raises(TypeError):
        interface.set_vel_acc_multiplier(cast(int, 1e8))
    with pytest.raises(InvalidRangeError):
        TrajectoryPointInterface.serialize_motion_primitive(
            MoveJPrimitive(Q(0, 0, 0, 0, 0, 0)), vel_acc_multiplier=0
        )
    assert interface.vel_acc_multiplier == MULT_VEL_ACC


def test_non_spline_blocks_ignore_vel_acc_multiplier() -> None:
    server = FakeServer(50003)
    interface = TrajectoryPointInterface(server=server)
    server.connect_client()
    assert interface.vel_acc_multiplier == MULT_VEL_ACC
    target = Pose(0.1, -0.2, 0.3, -0.4, 0.5, -0.6, Q(1, 2, 3, 4, 5, 6))
    interface.write_motion_primitive(
        MoveLPrimitive(target, duration=0.5, acceleration=0.7, velocity=0.6)
    )
    words = unpack(server.writes[-1])
    assert words[6:12] == (1_000_000, 2_000_000, 3_000_000, 4_000_000, 5_000_000, 6_000_000)
    assert words[12:18] == (600_000, 700_000, 1_000_000, 0, 0, 0)

    movec = MoveCPrimitive(Q(0, 0, 0, 0, 0, 0), Q(1, 1, 1, 1, 1, 1), velocity=0.25)
    interface.write_motion_primitive(movec)
    assert unpack(server.writes[-1])[12:15] == (250_000, 1_400_000, 0)


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
