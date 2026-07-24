"""Binary parser for Universal Robots Primary-interface packages."""

from __future__ import annotations

import struct
from enum import IntEnum
from typing import TypeVar

from ur_client_library.exceptions import ParseError
from ur_client_library.primary.packages import (
    ConfigurationData,
    ErrorCodeMessage,
    JointMotionLimits,
    JointPositionLimits,
    KeyMessage,
    KinematicsInfo,
    MasterboardData,
    PrimaryPackage,
    RawPrimaryPackage,
    RobotMessage,
    RobotMessageType,
    RobotModeData,
    RobotPackageType,
    RobotState,
    RobotStateType,
    RuntimeExceptionMessage,
    SafetyModeMessage,
    SafetyReportDataType,
    TextMessage,
    VersionMessage,
)
from ur_client_library.types import Vector6
from ur_client_library.ur.datatypes import ReportLevel, RobotMode, SafetyMode

_EnumT = TypeVar("_EnumT", bound=IntEnum)
_ROBOT_MODE = struct.Struct(">Q7?bB3d")
_CONFIGURATION = struct.Struct(">53d4i")
_KINEMATICS = struct.Struct(">6I24dI")
_MASTERBOARD_BASE = struct.Struct(">IIBBddbbddffffBBB")
_MASTERBOARD_IMMI = struct.Struct(">IIff")
_MASTERBOARD_TAIL = struct.Struct(">IBBB")
_ROBOT_MESSAGE_HEADER = struct.Struct(">QbB")


def _enum_or_int(enum_type: type[_EnumT], value: int) -> _EnumT | int:
    try:
        return enum_type(value)
    except ValueError:
        return value


def _decode(raw: bytes) -> str:
    return raw.decode("utf-8", errors="replace")


def _vector(values: tuple[float, ...], offset: int) -> Vector6:
    return values[offset : offset + 6]  # type: ignore[return-value]


class PrimaryParser:
    """Parse framed Primary traffic.

    In the default lenient mode, known-but-unimplemented and unknown package
    types are retained as immutable raw packages. Strict mode additionally
    rejects unknown numeric types and unconsumed bytes in implemented layouts.
    """

    def __init__(self, *, strict: bool = False) -> None:
        self.strict = strict

    def parse(self, data: bytes | bytearray | memoryview) -> tuple[PrimaryPackage, ...]:
        """Parse one or more complete top-level frames."""
        view = memoryview(data)
        offset = 0
        packages: list[PrimaryPackage] = []
        while offset < len(view):
            if len(view) - offset < 4:
                raise ParseError("Primary frame ended before its four-byte size header")
            package_size = struct.unpack_from(">i", view, offset)[0]
            if package_size < 5:
                raise ParseError(f"Invalid Primary package size {package_size}")
            end = offset + package_size
            if end > len(view):
                raise ParseError(
                    f"Primary package declares {package_size} bytes, "
                    f"but only {len(view) - offset} are available"
                )
            packages.extend(self._parse_frame(view[offset:end]))
            offset = end
        return tuple(packages)

    def parse_one(self, data: bytes | bytearray | memoryview) -> PrimaryPackage:
        packages = self.parse(data)
        if len(packages) != 1:
            raise ParseError(f"Expected exactly one parsed package, got {len(packages)}")
        return packages[0]

    def _parse_frame(self, frame: memoryview) -> list[PrimaryPackage]:
        package_type_value = struct.unpack_from(">b", frame, 4)[0]
        package_type = _enum_or_int(RobotPackageType, package_type_value)
        payload = bytes(frame[5:])

        if package_type == RobotPackageType.ROBOT_STATE:
            return self._parse_robot_state(payload)
        if package_type == RobotPackageType.ROBOT_MESSAGE:
            return [self._parse_robot_message(payload)]
        if self.strict and isinstance(package_type, int) and not isinstance(
            package_type, RobotPackageType
        ):
            raise ParseError(f"Unknown Primary package type {package_type}")
        return [RawPrimaryPackage(package_type=package_type, raw_data=payload)]

    def _parse_robot_state(self, payload: bytes) -> list[PrimaryPackage]:
        offset = 0
        packages: list[PrimaryPackage] = []
        while offset < len(payload):
            if len(payload) - offset < 5:
                raise ParseError("RobotState ended before a complete subpackage header")
            sub_size = struct.unpack_from(">I", payload, offset)[0]
            if sub_size < 5:
                raise ParseError(f"Invalid RobotState subpackage size {sub_size}")
            end = offset + sub_size
            if end > len(payload):
                raise ParseError(
                    f"RobotState subpackage declares {sub_size} bytes, "
                    f"but only {len(payload) - offset} are available"
                )
            state_type_value = payload[offset + 4]
            state_type = _enum_or_int(RobotStateType, state_type_value)
            sub_payload = payload[offset + 5 : end]
            if self.strict and isinstance(state_type, int) and not isinstance(
                state_type, RobotStateType
            ):
                raise ParseError(f"Unknown RobotState type {state_type}")
            packages.append(self._parse_state_package(state_type, sub_payload))
            # Advancing by the declared size, rather than bytes consumed by the
            # concrete decoder, prevents oversized packages consuming the next one.
            offset = end
        return packages

    def _parse_state_package(
        self, state_type: RobotStateType | int, payload: bytes
    ) -> PrimaryPackage:
        try:
            if state_type == RobotStateType.ROBOT_MODE_DATA:
                return self._parse_robot_mode(state_type, payload)
            if state_type == RobotStateType.CONFIGURATION_DATA:
                return self._parse_configuration(state_type, payload)
            if state_type == RobotStateType.KINEMATICS_INFO:
                return self._parse_kinematics(state_type, payload)
            if state_type == RobotStateType.MASTERBOARD_DATA:
                return self._parse_masterboard(state_type, payload)
        except struct.error as exc:
            raise ParseError(
                f"RobotState {getattr(state_type, 'name', state_type)} payload is truncated"
            ) from exc
        return RobotState(state_type=state_type, raw_data=payload)

    def _parse_robot_mode(
        self, state_type: RobotStateType | int, payload: bytes
    ) -> RobotModeData:
        if len(payload) < _ROBOT_MODE.size:
            raise ParseError("RobotModeData payload is truncated")
        values = _ROBOT_MODE.unpack_from(payload)
        return RobotModeData(
            state_type=state_type,
            raw_data=payload,
            timestamp=values[0],
            is_real_robot_connected=values[1],
            is_real_robot_enabled=values[2],
            is_robot_power_on=values[3],
            is_emergency_stopped=values[4],
            is_protective_stopped=values[5],
            is_program_running=values[6],
            is_program_paused=values[7],
            robot_mode=_enum_or_int(RobotMode, values[8]),
            control_mode=values[9],
            target_speed_fraction=values[10],
            speed_scaling=values[11],
            target_speed_fraction_limit=values[12],
            reserved=payload[_ROBOT_MODE.size :],
        )

    def _parse_configuration(
        self, state_type: RobotStateType | int, payload: bytes
    ) -> ConfigurationData:
        if len(payload) < _CONFIGURATION.size:
            raise ParseError("ConfigurationData payload is truncated")
        values = _CONFIGURATION.unpack_from(payload)
        doubles = values[:53]
        position_limits = tuple(
            JointPositionLimits(doubles[index], doubles[index + 1])
            for index in range(0, 12, 2)
        )
        motion_limits = tuple(
            JointMotionLimits(doubles[index], doubles[index + 1])
            for index in range(12, 24, 2)
        )
        offset = _CONFIGURATION.size
        reserved_1 = 0
        reserved_2 = 0
        if len(payload) - offset >= 2:
            reserved_1 = struct.unpack_from(">h", payload, offset)[0]
            offset += 2
        if len(payload) - offset >= 2:
            reserved_2 = struct.unpack_from(">h", payload, offset)[0]
            offset += 2
        unparsed = payload[offset:]
        self._check_unparsed(state_type, unparsed)
        ints = values[53:]
        return ConfigurationData(
            state_type=state_type,
            raw_data=payload,
            joint_position_limits=position_limits,
            joint_motion_limits=motion_limits,
            v_joint_default=doubles[24],
            a_joint_default=doubles[25],
            v_tool_default=doubles[26],
            a_tool_default=doubles[27],
            eq_radius=doubles[28],
            dh_a=_vector(doubles, 29),
            dh_d=_vector(doubles, 35),
            dh_alpha=_vector(doubles, 41),
            dh_theta=_vector(doubles, 47),
            masterboard_version=ints[0],
            controller_box_type=ints[1],
            robot_type=ints[2],
            robot_sub_type=ints[3],
            reserved_1=reserved_1,
            reserved_2=reserved_2,
            unparsed_data=unparsed,
        )

    def _parse_kinematics(
        self, state_type: RobotStateType | int, payload: bytes
    ) -> KinematicsInfo:
        if len(payload) < _KINEMATICS.size:
            raise ParseError("KinematicsInfo payload is truncated")
        values = _KINEMATICS.unpack_from(payload)
        unparsed = payload[_KINEMATICS.size :]
        self._check_unparsed(state_type, unparsed)
        doubles = values[6:30]
        return KinematicsInfo(
            state_type=state_type,
            raw_data=payload,
            checksum=values[:6],
            dh_theta=_vector(doubles, 0),
            dh_a=_vector(doubles, 6),
            dh_d=_vector(doubles, 12),
            dh_alpha=_vector(doubles, 18),
            calibration_status=values[30],
            unparsed_data=unparsed,
        )

    def _parse_masterboard(
        self, state_type: RobotStateType | int, payload: bytes
    ) -> MasterboardData:
        minimum = _MASTERBOARD_BASE.size + _MASTERBOARD_TAIL.size
        if len(payload) < minimum:
            raise ParseError("MasterboardData payload is truncated")
        values = _MASTERBOARD_BASE.unpack_from(payload)
        offset = _MASTERBOARD_BASE.size
        has_immi = values[16]
        immi_values: tuple[int, int, float, float] = (0, 0, 0.0, 0.0)
        if has_immi:
            if len(payload) - offset < _MASTERBOARD_IMMI.size + _MASTERBOARD_TAIL.size:
                raise ParseError("MasterboardData IMMI payload is truncated")
            immi_values = _MASTERBOARD_IMMI.unpack_from(payload, offset)
            offset += _MASTERBOARD_IMMI.size
        tail = _MASTERBOARD_TAIL.unpack_from(payload, offset)
        offset += _MASTERBOARD_TAIL.size
        unparsed = payload[offset:]
        self._check_unparsed(state_type, unparsed)
        return MasterboardData(
            state_type=state_type,
            raw_data=payload,
            digital_input_bits=values[0],
            digital_output_bits=values[1],
            analog_input_range_0=values[2],
            analog_input_range_1=values[3],
            analog_input_0=values[4],
            analog_input_1=values[5],
            analog_output_domain_0=values[6],
            analog_output_domain_1=values[7],
            analog_output_0=values[8],
            analog_output_1=values[9],
            master_board_temperature=values[10],
            robot_voltage_48v=values[11],
            robot_current=values[12],
            master_io_current=values[13],
            safety_mode=_enum_or_int(SafetyMode, values[14]),
            in_reduced_mode=bool(values[15]),
            immi_interface_installed=bool(has_immi),
            immi_input_bits=immi_values[0],
            immi_output_bits=immi_values[1],
            immi_voltage_24v=immi_values[2],
            immi_current=immi_values[3],
            reserved_1=tail[0],
            operational_mode_selector_input=tail[1],
            three_position_enabling_device_input=bool(tail[2]),
            reserved_2=tail[3],
            unparsed_data=unparsed,
        )

    def _parse_robot_message(self, payload: bytes) -> PrimaryPackage:
        if len(payload) < _ROBOT_MESSAGE_HEADER.size:
            raise ParseError("RobotMessage payload is truncated")
        timestamp, source, message_type_value = _ROBOT_MESSAGE_HEADER.unpack_from(payload)
        message_type = _enum_or_int(RobotMessageType, message_type_value)
        body = payload[_ROBOT_MESSAGE_HEADER.size :]
        if self.strict and isinstance(message_type, int) and not isinstance(
            message_type, RobotMessageType
        ):
            raise ParseError(f"Unknown RobotMessage type {message_type}")
        try:
            if message_type == RobotMessageType.VERSION:
                return self._parse_version(timestamp, source, message_type, body)
            if message_type == RobotMessageType.TEXT:
                return TextMessage(timestamp, source, message_type, body, _decode(body))
            if message_type == RobotMessageType.RUNTIME_EXCEPTION:
                line, column = struct.unpack_from(">II", body)
                return RuntimeExceptionMessage(
                    timestamp,
                    source,
                    message_type,
                    body,
                    line,
                    column,
                    _decode(body[8:]),
                )
            if message_type == RobotMessageType.KEY:
                return self._parse_key(timestamp, source, message_type, body)
            if message_type == RobotMessageType.ERROR_CODE:
                return self._parse_error_code(timestamp, source, message_type, body)
            if message_type == RobotMessageType.SAFETY_MODE:
                return self._parse_safety_mode(timestamp, source, message_type, body)
        except struct.error as exc:
            raise ParseError(
                f"RobotMessage {getattr(message_type, 'name', message_type)} payload is truncated"
            ) from exc
        return RobotMessage(timestamp, source, message_type, body)

    def _parse_version(
        self,
        timestamp: int,
        source: int,
        message_type: RobotMessageType | int,
        body: bytes,
    ) -> VersionMessage:
        if not body:
            raise ParseError("VersionMessage payload is truncated")
        name_length = body[0]
        fixed_end = 1 + name_length + 10
        if len(body) < fixed_end:
            raise ParseError("VersionMessage project or version fields are truncated")
        project_name = _decode(body[1 : 1 + name_length])
        major, minor, svn, build = struct.unpack_from(">BBii", body, 1 + name_length)
        return VersionMessage(
            timestamp,
            source,
            message_type,
            body,
            project_name,
            major,
            minor,
            svn,
            build,
            _decode(body[fixed_end:]),
        )

    def _parse_key(
        self,
        timestamp: int,
        source: int,
        message_type: RobotMessageType | int,
        body: bytes,
    ) -> KeyMessage:
        message_code, message_argument, title_length = struct.unpack_from(">iiB", body)
        title_end = 9 + title_length
        if len(body) < title_end:
            raise ParseError("KeyMessage title is truncated")
        return KeyMessage(
            timestamp,
            source,
            message_type,
            body,
            message_code,
            message_argument,
            _decode(body[9:title_end]),
            _decode(body[title_end:]),
        )

    def _parse_error_code(
        self,
        timestamp: int,
        source: int,
        message_type: RobotMessageType | int,
        body: bytes,
    ) -> ErrorCodeMessage:
        message_code, argument, level, data_type, data = struct.unpack_from(">iiiII", body)
        return ErrorCodeMessage(
            timestamp,
            source,
            message_type,
            body,
            message_code,
            argument,
            _enum_or_int(ReportLevel, level),
            data_type,
            data,
            _decode(body[20:]),
        )

    def _parse_safety_mode(
        self,
        timestamp: int,
        source: int,
        message_type: RobotMessageType | int,
        body: bytes,
    ) -> SafetyModeMessage:
        message_code, argument, safety_value, report_type = struct.unpack_from(">iiBI", body)
        report_raw = body[13:17]
        if len(report_raw) < 4:
            raise ParseError("SafetyModeMessage report data is truncated")
        if report_type == SafetyReportDataType.SIGNED:
            report_data: int | float = struct.unpack(">i", report_raw)[0]
        elif report_type == SafetyReportDataType.FLOAT:
            report_data = struct.unpack(">f", report_raw)[0]
        else:
            report_data = struct.unpack(">I", report_raw)[0]
        unparsed = body[17:]
        self._check_unparsed(message_type, unparsed)
        return SafetyModeMessage(
            timestamp,
            source,
            message_type,
            body,
            message_code,
            argument,
            _enum_or_int(SafetyMode, safety_value),
            _enum_or_int(SafetyReportDataType, report_type),
            report_data,
        )

    def _check_unparsed(self, package_type: object, unparsed: bytes) -> None:
        if self.strict and unparsed:
            name = getattr(package_type, "name", package_type)
            raise ParseError(f"{name} contains {len(unparsed)} unparsed trailing bytes")
