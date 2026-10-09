"""Immutable Primary-interface package models and protocol enumerations."""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum

from ur_client_library.helpers import calibration_hash
from ur_client_library.types import Vector6
from ur_client_library.ur.datatypes import ReportLevel, RobotMode, SafetyMode
from ur_client_library.ur.error_codes import error_code_text
from ur_client_library.ur.version_information import VersionInformation

UR_PRIMARY_PORT = 30001
UR_SECONDARY_PORT = 30002


class RobotPackageType(IntEnum):
    DISCONNECT = -1
    MODBUS_INFO_MESSAGE = 5
    ROBOT_STATE = 16
    ROBOT_MESSAGE = 20
    HMC_MESSAGE = 22
    SAFETY_SETUP_BROADCAST_MESSAGE = 23
    SAFETY_COMPLIANCE_TOLERANCES_MESSAGE = 24
    PROGRAM_STATE_MESSAGE = 25


class RobotStateType(IntEnum):
    ROBOT_MODE_DATA = 0
    JOINT_DATA = 1
    TOOL_DATA = 2
    MASTERBOARD_DATA = 3
    CARTESIAN_INFO = 4
    KINEMATICS_INFO = 5
    CONFIGURATION_DATA = 6
    FORCE_MODE_DATA = 7
    ADDITIONAL_INFO = 8
    CALIBRATION_DATA = 9
    SAFETY_DATA = 10
    TOOL_COMM_INFO = 11
    TOOL_MODE_INFO = 12


class RobotMessageType(IntEnum):
    TEXT = 0
    PROGRAM_LABEL = 1
    POPUP = 2
    VERSION = 3
    SAFETY_MODE = 5
    ERROR_CODE = 6
    KEY = 7
    REQUEST_VALUE = 9
    RUNTIME_EXCEPTION = 10


# The longer upstream name remains useful when comparing protocol documentation.
RobotMessagePackageType = RobotMessageType


class SafetyReportDataType(IntEnum):
    UNSIGNED = 0
    UNSIGNED_1 = 1
    SIGNED = 2
    FLOAT = 3
    HEX = 4


class ScriptType(IntEnum):
    DEF = 0
    SEC = 1


class PackageHeader:
    """Helpers for the four-byte, big-endian Primary frame header."""

    SIZE = 4

    @staticmethod
    def get_package_length(buffer: bytes | bytearray | memoryview) -> int:
        if len(buffer) < PackageHeader.SIZE:
            raise ValueError("A Primary package header requires four bytes")
        return int(struct.unpack_from(">i", buffer)[0])


class PrimaryPackage:
    """Marker base class for parsed Primary-interface packages."""


@dataclass(frozen=True, slots=True)
class RawPrimaryPackage(PrimaryPackage):
    package_type: RobotPackageType | int
    raw_data: bytes = b""


@dataclass(frozen=True, slots=True)
class RobotState(PrimaryPackage):
    state_type: RobotStateType | int
    raw_data: bytes = b""


@dataclass(frozen=True, slots=True)
class RobotMessage(PrimaryPackage):
    timestamp: int
    source: int
    message_type: RobotMessageType | int
    raw_data: bytes = b""


@dataclass(frozen=True, slots=True)
class RobotModeData(RobotState):
    timestamp: int = 0
    is_real_robot_connected: bool = False
    is_real_robot_enabled: bool = False
    is_robot_power_on: bool = False
    is_emergency_stopped: bool = False
    is_protective_stopped: bool = False
    is_program_running: bool = False
    is_program_paused: bool = False
    robot_mode: RobotMode | int = RobotMode.UNKNOWN
    control_mode: int = 0
    target_speed_fraction: float = 0.0
    speed_scaling: float = 0.0
    target_speed_fraction_limit: float = 0.0
    reserved: bytes = b""


@dataclass(frozen=True, slots=True)
class JointPositionLimits:
    joint_min_limit: float
    joint_max_limit: float


@dataclass(frozen=True, slots=True)
class JointMotionLimits:
    joint_max_speed: float
    joint_max_acceleration: float


_ZERO_VECTOR6: Vector6 = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True, slots=True)
class ConfigurationData(RobotState):
    joint_position_limits: tuple[JointPositionLimits, ...] = ()
    joint_motion_limits: tuple[JointMotionLimits, ...] = ()
    v_joint_default: float = 0.0
    a_joint_default: float = 0.0
    v_tool_default: float = 0.0
    a_tool_default: float = 0.0
    eq_radius: float = 0.0
    dh_a: Vector6 = _ZERO_VECTOR6
    dh_d: Vector6 = _ZERO_VECTOR6
    dh_alpha: Vector6 = _ZERO_VECTOR6
    dh_theta: Vector6 = _ZERO_VECTOR6
    masterboard_version: int = 0
    controller_box_type: int = 0
    robot_type: int = 0
    robot_sub_type: int = 0
    reserved_1: int = 0
    reserved_2: int = 0
    unparsed_data: bytes = b""


@dataclass(frozen=True, slots=True)
class KinematicsInfo(RobotState):
    checksum: tuple[int, int, int, int, int, int] = (0, 0, 0, 0, 0, 0)
    dh_theta: Vector6 = _ZERO_VECTOR6
    dh_a: Vector6 = _ZERO_VECTOR6
    dh_d: Vector6 = _ZERO_VECTOR6
    dh_alpha: Vector6 = _ZERO_VECTOR6
    calibration_status: int = 0
    unparsed_data: bytes = b""

    @property
    def calibration_hash(self) -> str:
        return calibration_hash(self.dh_theta, self.dh_d, self.dh_a, self.dh_alpha)

    def to_hash(self) -> str:
        return self.calibration_hash


@dataclass(frozen=True, slots=True)
class MasterboardData(RobotState):
    digital_input_bits: int = 0
    digital_output_bits: int = 0
    analog_input_range_0: int = 0
    analog_input_range_1: int = 0
    analog_input_0: float = 0.0
    analog_input_1: float = 0.0
    analog_output_domain_0: int = 0
    analog_output_domain_1: int = 0
    analog_output_0: float = 0.0
    analog_output_1: float = 0.0
    master_board_temperature: float = 0.0
    robot_voltage_48v: float = 0.0
    robot_current: float = 0.0
    master_io_current: float = 0.0
    safety_mode: SafetyMode | int = SafetyMode.UNDEFINED_SAFETY_MODE
    in_reduced_mode: bool = False
    immi_interface_installed: bool = False
    immi_input_bits: int = 0
    immi_output_bits: int = 0
    immi_voltage_24v: float = 0.0
    immi_current: float = 0.0
    reserved_1: int = 0
    operational_mode_selector_input: int = 0
    three_position_enabling_device_input: bool = False
    reserved_2: int = 0
    unparsed_data: bytes = b""


@dataclass(frozen=True, slots=True)
class VersionMessage(RobotMessage):
    project_name: str = ""
    major_version: int = 0
    minor_version: int = 0
    svn_version: int = 0
    build_number: int = 0
    build_date: str = ""

    @property
    def version(self) -> VersionInformation:
        return VersionInformation(
            self.major_version,
            self.minor_version,
            self.svn_version,
            self.build_number,
        )


@dataclass(frozen=True, slots=True)
class TextMessage(RobotMessage):
    text: str = ""


@dataclass(frozen=True, slots=True)
class RuntimeExceptionMessage(RobotMessage):
    line_number: int = 0
    column_number: int = 0
    text: str = ""


@dataclass(frozen=True, slots=True)
class KeyMessage(RobotMessage):
    message_code: int = 0
    message_argument: int = 0
    title: str = ""
    text: str = ""

    @property
    def title_length(self) -> int:
        return len(self.title.encode())


def _describe_error_code(message_code: int, message_argument: int) -> str:
    code = f"C{message_code}A{message_argument}"
    text = error_code_text(message_code, message_argument)
    return code if text is None else f"{code}: {text}"


@dataclass(frozen=True, slots=True)
class ErrorCode:
    message_code: int = -1
    message_argument: int = -1
    report_level: ReportLevel | int = ReportLevel.DEBUG
    data_type: int = 0
    data: int = 0
    text: str = ""
    timestamp: int = 0

    @property
    def code(self) -> str:
        return f"C{self.message_code}A{self.message_argument}"

    @property
    def description(self) -> str:
        """``C<code>A<arg>: <text>`` from the error-code table, else :attr:`code`.

        Mirrors upstream ``ErrorCodeMessage::toString()``; :attr:`text` keeps the
        controller-supplied payload.
        """
        return _describe_error_code(self.message_code, self.message_argument)


@dataclass(frozen=True, slots=True)
class ErrorCodeMessage(RobotMessage):
    message_code: int = 0
    message_argument: int = 0
    report_level: ReportLevel | int = ReportLevel.DEBUG
    data_type: int = 0
    data: int = 0
    text: str = ""

    @property
    def code(self) -> str:
        return f"C{self.message_code}A{self.message_argument}"

    @property
    def description(self) -> str:
        """``C<code>A<arg>: <text>`` from the error-code table, else :attr:`code`.

        Mirrors upstream ``ErrorCodeMessage::toString()``; :attr:`text` keeps the
        controller-supplied payload.
        """
        return _describe_error_code(self.message_code, self.message_argument)

    def to_error_code(self) -> ErrorCode:
        return ErrorCode(
            message_code=self.message_code,
            message_argument=self.message_argument,
            report_level=self.report_level,
            data_type=self.data_type,
            data=self.data,
            text=self.text,
            timestamp=self.timestamp,
        )


@dataclass(frozen=True, slots=True)
class SafetyModeMessage(RobotMessage):
    message_code: int = 0
    message_argument: int = 0
    safety_mode: SafetyMode | int = SafetyMode.UNDEFINED_SAFETY_MODE
    report_data_type: SafetyReportDataType | int = SafetyReportDataType.UNSIGNED
    report_data: int | float = 0

    @property
    def safety_mode_type(self) -> SafetyMode | int:
        return self.safety_mode


@dataclass(frozen=True, slots=True)
class ScriptInfo:
    script_name: str
    script_code: str
    script_type: ScriptType
