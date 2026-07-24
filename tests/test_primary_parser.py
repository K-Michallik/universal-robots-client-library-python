import struct

import pytest

from ur_client_library.exceptions import ParseError
from ur_client_library.primary import (
    ConfigurationData,
    ErrorCodeMessage,
    KeyMessage,
    KinematicsInfo,
    MasterboardData,
    PrimaryParser,
    RawPrimaryPackage,
    RobotMessage,
    RobotMessageType,
    RobotModeData,
    RobotPackageType,
    RobotState,
    RobotStateType,
    RuntimeExceptionMessage,
    SafetyModeMessage,
    TextMessage,
    VersionMessage,
)
from ur_client_library.ur.datatypes import ReportLevel, RobotMode, SafetyMode


def frame(package_type: int, payload: bytes) -> bytes:
    return struct.pack(">iB", len(payload) + 5, package_type) + payload


def robot_state(*subpackages: tuple[int, bytes]) -> bytes:
    payload = b"".join(
        struct.pack(">IB", len(body) + 5, state_type) + body
        for state_type, body in subpackages
    )
    return frame(RobotPackageType.ROBOT_STATE, payload)


def robot_message(message_type: int, payload: bytes, *, timestamp: int = 7) -> bytes:
    return frame(
        RobotPackageType.ROBOT_MESSAGE,
        struct.pack(">QBB", timestamp, 0xFE, message_type) + payload,
    )


def test_parses_multiple_robot_state_subpackages_without_overconsumption() -> None:
    kinematics = struct.pack(
        ">6I24dI",
        *(range(1, 7)),
        *(float(value) for value in range(24)),
        9,
    )
    packet = robot_state(
        (RobotStateType.KINEMATICS_INFO, kinematics + b"\xAA" * 10),
        (RobotStateType.ADDITIONAL_INFO, b"\x42"),
    )

    parsed = PrimaryParser().parse(packet)

    assert len(parsed) == 2
    assert isinstance(parsed[0], KinematicsInfo)
    assert parsed[0].calibration_status == 9
    assert parsed[0].unparsed_data == b"\xAA" * 10
    assert isinstance(parsed[1], RobotState)
    assert parsed[1].raw_data == b"\x42"
    with pytest.raises(ParseError, match="unparsed trailing"):
        PrimaryParser(strict=True).parse(packet)


def test_parses_configuration_optional_reserved_tail() -> None:
    legacy = struct.pack(">53d4i", *(float(value) for value in range(53)), 1, 2, 3, 4)

    without_tail = PrimaryParser(strict=True).parse_one(
        robot_state((RobotStateType.CONFIGURATION_DATA, legacy))
    )
    with_tail = PrimaryParser(strict=True).parse_one(
        robot_state(
            (
                RobotStateType.CONFIGURATION_DATA,
                legacy + struct.pack(">hh", 0x1234, 0x5678),
            )
        )
    )

    assert isinstance(without_tail, ConfigurationData)
    assert without_tail.reserved_1 == without_tail.reserved_2 == 0
    assert isinstance(with_tail, ConfigurationData)
    assert with_tail.reserved_1 == 0x1234
    assert with_tail.reserved_2 == 0x5678
    assert with_tail.robot_type == 3
    assert with_tail.dh_theta == tuple(float(value) for value in range(47, 53))


@pytest.mark.parametrize("has_immi", [False, True])
def test_parses_masterboard_conditional_immi_fields(has_immi: bool) -> None:
    payload = struct.pack(
        ">IIBBddbbddffffBBB",
        0xDEADBEEF,
        0xCAFEBABE,
        1,
        2,
        1.5,
        -2.25,
        3,
        4,
        3.75,
        -4.5,
        42.0,
        48.1,
        1.23,
        0.5,
        SafetyMode.REDUCED,
        1,
        has_immi,
    )
    if has_immi:
        payload += struct.pack(">IIff", 0x11112222, 0x33334444, 24.0, 0.75)
    payload += struct.pack(">IBBB", 0, 2, 1, 0)

    parsed = PrimaryParser(strict=True).parse_one(
        robot_state((RobotStateType.MASTERBOARD_DATA, payload))
    )

    assert isinstance(parsed, MasterboardData)
    assert parsed.digital_input_bits == 0xDEADBEEF
    assert parsed.safety_mode == SafetyMode.REDUCED
    assert parsed.immi_interface_installed is has_immi
    assert parsed.immi_input_bits == (0x11112222 if has_immi else 0)
    assert parsed.immi_voltage_24v == (24.0 if has_immi else 0.0)
    assert parsed.operational_mode_selector_input == 2


def test_parses_robot_mode_and_reserved_bytes() -> None:
    payload = struct.pack(
        ">Q7?bB3d",
        123,
        True,
        True,
        True,
        False,
        False,
        True,
        False,
        RobotMode.RUNNING,
        2,
        1.0,
        0.75,
        0.5,
    )

    parsed = PrimaryParser(strict=True).parse_one(
        robot_state((RobotStateType.ROBOT_MODE_DATA, payload + b"\x99"))
    )

    assert isinstance(parsed, RobotModeData)
    assert parsed.robot_mode == RobotMode.RUNNING
    assert parsed.is_program_running
    assert parsed.reserved == b"\x99"


def test_parses_all_implemented_robot_messages() -> None:
    version_payload = (
        b"\x09URControl"
        + struct.pack(">BBii", 5, 24, 2, 99)
        + b"04-10-2025, 00:25:33"
    )
    key_payload = struct.pack(">iiB", 0, 0, 19) + b"PROGRAM_XXX_STARTED" + b"test_fun"
    runtime_payload = struct.pack(">II", 3, 1) + b"compile_error"
    error_payload = struct.pack(">iiiII", 210, 1, ReportLevel.VIOLATION, 4, 5) + b"read only"

    messages = [
        PrimaryParser(strict=True).parse_one(
            robot_message(RobotMessageType.VERSION, version_payload)
        ),
        PrimaryParser(strict=True).parse_one(
            robot_message(RobotMessageType.TEXT, b"hello")
        ),
        PrimaryParser(strict=True).parse_one(
            robot_message(RobotMessageType.KEY, key_payload)
        ),
        PrimaryParser(strict=True).parse_one(
            robot_message(RobotMessageType.RUNTIME_EXCEPTION, runtime_payload)
        ),
        PrimaryParser(strict=True).parse_one(
            robot_message(RobotMessageType.ERROR_CODE, error_payload)
        ),
    ]

    assert isinstance(messages[0], VersionMessage)
    assert messages[0].project_name == "URControl"
    assert messages[0].version.to_string() == "5.24.2.99"
    assert messages[0].source == -2
    assert isinstance(messages[1], TextMessage)
    assert messages[1].text == "hello"
    assert isinstance(messages[2], KeyMessage)
    assert messages[2].title == "PROGRAM_XXX_STARTED"
    assert isinstance(messages[3], RuntimeExceptionMessage)
    assert messages[3].line_number == 3
    assert isinstance(messages[4], ErrorCodeMessage)
    assert messages[4].report_level == ReportLevel.VIOLATION


@pytest.mark.parametrize(
    ("report_type", "wire", "expected"),
    [
        (0, struct.pack(">I", 1), 1),
        (1, struct.pack(">I", 1), 1),
        (2, struct.pack(">i", -2), -2),
        (3, struct.pack(">f", 1.25), 1.25),
        (4, struct.pack(">I", 0x80070605), 0x80070605),
    ],
)
def test_parses_safety_report_data_variants(
    report_type: int, wire: bytes, expected: int | float
) -> None:
    payload = struct.pack(">iiBI", 0, 0, SafetyMode.PROTECTIVE_STOP, report_type) + wire

    parsed = PrimaryParser(strict=True).parse_one(
        robot_message(RobotMessageType.SAFETY_MODE, payload)
    )

    assert isinstance(parsed, SafetyModeMessage)
    assert parsed.safety_mode == SafetyMode.PROTECTIVE_STOP
    assert parsed.report_data == expected


def test_lenient_unknown_fallback_and_strict_rejection() -> None:
    unknown_outer = frame(99, b"future")
    unknown_message = robot_message(88, b"future")

    outer = PrimaryParser().parse_one(unknown_outer)
    message = PrimaryParser().parse_one(unknown_message)

    assert isinstance(outer, RawPrimaryPackage)
    assert outer.raw_data == b"future"
    assert isinstance(message, RobotMessage)
    assert message.raw_data == b"future"
    with pytest.raises(ParseError, match="Unknown Primary"):
        PrimaryParser(strict=True).parse(unknown_outer)
    with pytest.raises(ParseError, match="Unknown RobotMessage"):
        PrimaryParser(strict=True).parse(unknown_message)


def test_rejects_invalid_declared_lengths() -> None:
    with pytest.raises(ParseError, match="declares"):
        PrimaryParser().parse(struct.pack(">iB", 100, RobotPackageType.ROBOT_MESSAGE))
    with pytest.raises(ParseError, match="subpackage"):
        PrimaryParser().parse(
            frame(RobotPackageType.ROBOT_STATE, struct.pack(">IB", 99, 0))
        )
