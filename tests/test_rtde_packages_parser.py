from __future__ import annotations

import pytest

from ur_client_library.exceptions import ParseError, RTDEInvalidKeyError
from ur_client_library.rtde import (
    FIELD_TYPES,
    ControlPackageSetupOutputsRequest,
    DataPackage,
    FieldType,
    RequestProtocolVersion,
    RequestProtocolVersionRequest,
    RTDEParser,
)


def test_full_registry_contains_latest_urcl_fields() -> None:
    assert FIELD_TYPES["control_step"] is FieldType.UINT64
    assert FIELD_TYPES["target_base_wrench"] is FieldType.VECTOR6D
    assert FIELD_TYPES["input_bit_register_127"] is FieldType.BOOL
    assert FIELD_TYPES["output_double_register_47"] is FieldType.DOUBLE
    assert FIELD_TYPES["actual_current_as_torque"] is FieldType.VECTOR6D


def test_data_package_serialization_and_mapping_validation() -> None:
    package = DataPackage(["speed_slider_mask"])
    package.recipe_id = 1
    package["speed_slider_mask"] = 1
    assert package.serialize() == bytes.fromhex("0008550100000001")
    with pytest.raises(RTDEInvalidKeyError):
        package["actual_q"] = (0.0,) * 6
    with pytest.raises(TypeError):
        package["speed_slider_mask"] = True


def test_data_package_protocol_versions_and_immutable_vectors() -> None:
    raw_v2 = bytes.fromhex(
        "01"
        "40d0758c49ba5e35"
        "bff99c77d110b460bffba233d110b460c0019fbe68885a30"
        "bfe9db22a22168c03ff98587a0000000bf9fbe74442d1800"
    )
    package = DataPackage.from_payload(["timestamp", "actual_q"], raw_v2, 2)
    assert package.recipe_id == 1
    assert package["timestamp"] == pytest.approx(16854.1919)
    assert isinstance(package["actual_q"], tuple)
    assert package["actual_q"][0] == pytest.approx(-1.6007)

    protocol_v1 = DataPackage.from_payload(["timestamp", "actual_q"], raw_v2[1:], 1)
    assert protocol_v1["actual_q"] == package["actual_q"]
    with pytest.raises(ParseError):
        DataPackage.from_payload(["timestamp"], b"\0", 2)
    with pytest.raises(TypeError):
        DataPackage(["timestamp"], protocol_version=True)
    with pytest.raises(TypeError):
        RTDEParser(["timestamp"], protocol_version=True)


def test_data_package_missing_key_uses_domain_and_mapping_error_contract() -> None:
    package = DataPackage(("timestamp",))

    with pytest.raises(RTDEInvalidKeyError) as raised:
        _ = package["actual_q"]

    assert isinstance(raised.value, KeyError)
    assert raised.value.invalid_keys == ("actual_q",)
    assert str(raised.value) == "'actual_q' is not part of this RTDE recipe"

    with pytest.raises(RTDEInvalidKeyError) as unknown:
        DataPackage(("timestamp", "nope_a", "nope_b"))
    assert unknown.value.invalid_keys == ("nope_a", "nope_b")


def test_parser_and_requests_match_upstream_wire_bytes() -> None:
    request = RequestProtocolVersionRequest.serialize(2)
    assert request == bytes.fromhex("0005560002")
    response = RTDEParser(["timestamp"]).parse(bytes.fromhex("00045601"))
    assert isinstance(response, RequestProtocolVersion)
    assert response.accepted

    setup = ControlPackageSetupOutputsRequest.serialize(
        ["actual_q", "actual_qd"],
        protocol_version=2,
        frequency=500.0,
    )
    assert setup == bytes.fromhex("001d4f407f40000000000061637475616c5f712c61637475616c5f7164")
    with pytest.raises(ParseError):
        RTDEParser(["timestamp"]).parse(bytes.fromhex("0006560102"))
