import pytest

from ur_client_library.helpers import (
    calibration_hash,
    libstdcxx_string_hash,
    parse_boolean,
    robot_series_from_type_and_version,
    robot_type_from_string,
    round_half_away_from_zero,
    split_string,
)
from ur_client_library.ur.datatypes import RobotSeries, RobotType
from ur_client_library.ur.version_information import VersionInformation


@pytest.mark.parametrize("value", ["true", "True", "ON", "yes", "1"])
def test_parse_boolean_true(value: str) -> None:
    assert parse_boolean(value)


@pytest.mark.parametrize("value", ["false", "False", "OFF", "no", "0"])
def test_parse_boolean_false(value: str) -> None:
    assert not parse_boolean(value)


def test_split_string_preserves_empty_fields() -> None:
    assert split_string("a,,b,") == ["a", "", "b", ""]


def test_robot_type_and_series_mapping_matches_upstream() -> None:
    assert robot_type_from_string("ur7e") is RobotType.UR5
    assert robot_type_from_string("ur8long") is RobotType.UR8_LONG
    assert (
        robot_series_from_type_and_version(RobotType.UR10, VersionInformation(3, 15))
        is RobotSeries.CB3
    )
    assert (
        robot_series_from_type_and_version(RobotType.UR20, VersionInformation(5, 12))
        is RobotSeries.UR_SERIES
    )


def test_rounding_matches_std_round() -> None:
    assert round_half_away_from_zero(1.5) == 2
    assert round_half_away_from_zero(-1.5) == -2
    assert round_half_away_from_zero(1.49) == 1
    assert round_half_away_from_zero(-1.49) == -1


def test_libstdcxx_hash_known_values() -> None:
    assert libstdcxx_string_hash("") == 6142509188972423790
    assert libstdcxx_string_hash("hello") == 2762169579135187400


def test_calibration_hash_matches_upstream_serialization_order() -> None:
    zeros = [0.0] * 6
    assert calibration_hash(zeros, zeros, zeros, zeros) == "calib_8811247710799071897"
