import pytest

from ur_client_library.types import Pose, Q, normalize_target, validate_ints


def test_q_accepts_sequence_or_six_values() -> None:
    expected = Q(0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
    assert Q([0.1, 0.2, 0.3, 0.4, 0.5, 0.6]) == expected
    assert expected.values == (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)


def test_q_rejects_wrong_shape_and_boolean() -> None:
    with pytest.raises(ValueError):
        Q([1.0, 2.0])
    with pytest.raises(TypeError):
        Q([1.0, 2.0, 3.0, 4.0, 5.0, True])


def test_pose_has_immutable_values_and_q_near() -> None:
    hint = Q([0, 1, 2, 3, 4, 5])
    pose = Pose(1, 2, 3, 0.1, 0.2, 0.3, hint)
    assert pose.values == (1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
    assert pose.q_near == hint


def test_target_semantics_are_explicit() -> None:
    values = [0, 1, 2, 3, 4, 5]
    assert isinstance(normalize_target(values), Q)
    assert isinstance(normalize_target(values, pose=True), Pose)


def test_integer_validation_checks_bounds() -> None:
    assert validate_ints(range(6), 6, name="words", minimum=0, maximum=5) == tuple(range(6))
    with pytest.raises(ValueError):
        validate_ints(range(6), 6, name="words", maximum=4)


def test_to_string_uses_cpp_ostream_number_formatting() -> None:
    q = Q(0.0, -0.0, 1.0 / 3.0, 1234567.0, 1e-5, -2.5)
    assert q.to_string() == "Q([0, -0, 0.333333, 1.23457e+06, 1e-05, -2.5])"
    assert str(q) == q.to_string()
    assert repr(q).startswith("Q(values=")

    pose = Pose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3, Q(0, 1, 2, 3, 4, 5))
    assert pose.to_string() == (
        "Pose(x = 1, y = 2, z = 3, rx = 0.1, ry = 0.2, rz = 0.3)\nWith Q_near([0, 1, 2, 3, 4, 5])"
    )
    assert f"{pose}" == pose.to_string()
    assert repr(pose).startswith("Pose(x=1.0")
