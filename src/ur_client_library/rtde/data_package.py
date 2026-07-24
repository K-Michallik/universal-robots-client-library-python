"""Recipe-driven RTDE data packages."""

from __future__ import annotations

import struct
from collections.abc import Iterator, Mapping, MutableMapping, Sequence
from enum import Enum, IntEnum
from numbers import Real
from types import MappingProxyType
from typing import TypeAlias, cast

from ur_client_library.comm.bin_parser import BinParser
from ur_client_library.exceptions import ParseError, RTDEInvalidKeyError
from ur_client_library.rtde.package_header import PackageHeader, PackageType
from ur_client_library.rtde.rtde_package import RTDEPackage
from ur_client_library.types import Vector3, Vector6, Vector6Int


class RuntimeState(IntEnum):
    STOPPING = 0
    STOPPED = 1
    PLAYING = 2
    PAUSING = 3
    PAUSED = 4
    RESUMING = 5


class FieldType(Enum):
    BOOL = ("?", 1, bool)
    UINT8 = ("B", 1, int)
    UINT32 = ("I", 4, int)
    UINT64 = ("Q", 8, int)
    INT32 = ("i", 4, int)
    DOUBLE = ("d", 8, float)
    VECTOR3D = ("3d", 24, tuple)
    VECTOR6D = ("6d", 48, tuple)
    VECTOR6INT32 = ("6i", 24, tuple)
    VECTOR6UINT32 = ("6I", 24, tuple)

    def __init__(self, format_string: str, size: int, python_type: type[object]) -> None:
        self.format_string = format_string
        self.size = size
        self.python_type = python_type


RTDEValue: TypeAlias = bool | int | float | Vector3 | Vector6 | Vector6Int


def _build_field_types() -> dict[str, FieldType]:
    fields: dict[str, FieldType] = {
        # Inputs
        "speed_slider_mask": FieldType.UINT32,
        "speed_slider_fraction": FieldType.DOUBLE,
        "standard_digital_output_mask": FieldType.UINT8,
        "standard_digital_output": FieldType.UINT8,
        "configurable_digital_output_mask": FieldType.UINT8,
        "configurable_digital_output": FieldType.UINT8,
        "standard_analog_output_mask": FieldType.UINT8,
        "standard_analog_output_type": FieldType.UINT8,
        "standard_analog_output_0": FieldType.DOUBLE,
        "standard_analog_output_1": FieldType.DOUBLE,
        "external_force_torque": FieldType.VECTOR6D,
        # Input/output register aggregates.
        "input_bit_registers0_to_31": FieldType.UINT32,
        "input_bit_registers32_to_63": FieldType.UINT32,
        # Core outputs.
        "timestamp": FieldType.DOUBLE,
        "actual_digital_input_bits": FieldType.UINT64,
        "actual_configurable_digital_input_bits": FieldType.UINT64,
        "actual_execution_time": FieldType.DOUBLE,
        "target_execution_time": FieldType.DOUBLE,
        "robot_mode": FieldType.INT32,
        "safety_mode": FieldType.INT32,
        "safety_status": FieldType.INT32,
        "speed_scaling": FieldType.DOUBLE,
        "target_speed_fraction": FieldType.DOUBLE,
        "actual_momentum": FieldType.DOUBLE,
        "actual_main_voltage": FieldType.DOUBLE,
        "actual_robot_voltage": FieldType.DOUBLE,
        "actual_robot_current": FieldType.DOUBLE,
        "actual_digital_output_bits": FieldType.UINT64,
        "actual_configurable_digital_output_bits": FieldType.UINT64,
        "runtime_state": FieldType.UINT32,
        "robot_status_bits": FieldType.UINT32,
        "safety_status_bits": FieldType.UINT32,
        "analog_io_types": FieldType.UINT32,
        "standard_analog_input0": FieldType.DOUBLE,
        "standard_analog_input1": FieldType.DOUBLE,
        "standard_analog_output0": FieldType.DOUBLE,
        "standard_analog_output1": FieldType.DOUBLE,
        "io_current": FieldType.DOUBLE,
        "output_bit_registers0_to_31": FieldType.UINT32,
        "output_bit_registers32_to_63": FieldType.UINT32,
        "actual_robot_energy_consumed": FieldType.DOUBLE,
        "actual_robot_braking_energy_dissipated": FieldType.DOUBLE,
        "encoder0_raw": FieldType.INT32,
        "encoder1_raw": FieldType.INT32,
        "euromap67_input_bits": FieldType.UINT32,
        "euromap67_output_bits": FieldType.UINT32,
        "euromap67_24V_voltage": FieldType.DOUBLE,
        "euromap67_24V_current": FieldType.DOUBLE,
        "tool_mode": FieldType.UINT32,
        "tool_analog_input_types": FieldType.UINT32,
        "tool_analog_input0": FieldType.DOUBLE,
        "tool_analog_input1": FieldType.DOUBLE,
        "tool_output_voltage": FieldType.INT32,
        "tool_output_current": FieldType.DOUBLE,
        "tool_temperature": FieldType.DOUBLE,
        "tool_output_mode": FieldType.UINT8,
        "tool_digital_output0_mode": FieldType.UINT8,
        "tool_digital_output1_mode": FieldType.UINT8,
        "tcp_force_scalar": FieldType.DOUBLE,
        "joint_position_deviation_ratio": FieldType.DOUBLE,
        "collision_detection_ratio": FieldType.DOUBLE,
        "payload": FieldType.DOUBLE,
        "script_control_line": FieldType.UINT32,
        "time_scale_source": FieldType.INT32,
        "control_step": FieldType.UINT64,
        # Undocumented inputs retained by upstream URCL.
        "tool_digital_output_mask": FieldType.UINT8,
        "tool_digital_output": FieldType.UINT8,
    }
    vector6d = (
        "target_q",
        "target_qd",
        "target_qdd",
        "target_current",
        "target_moment",
        "actual_q",
        "actual_qd",
        "actual_current",
        "actual_current_window",
        "actual_current_as_torque",
        "joint_control_output",
        "actual_TCP_pose",
        "actual_TCP_speed",
        "actual_TCP_force",
        "target_TCP_pose",
        "target_TCP_speed",
        "tcp_offset",
        "actual_TCP_acceleration",
        "target_TCP_acceleration",
        "joint_temperatures",
        "actual_joint_voltage",
        "ft_raw_wrench",
        "wrench_calc_from_currents",
        "payload_inertia",
        "target_base_acceleration",
        "target_base_wrench",
    )
    vector3d = (
        "actual_tool_accelerometer",
        "elbow_position",
        "elbow_velocity",
        "payload_cog",
        "target_gravity",
    )
    fields.update(dict.fromkeys(vector6d, FieldType.VECTOR6D))
    fields.update(dict.fromkeys(vector3d, FieldType.VECTOR3D))
    fields["joint_mode"] = FieldType.VECTOR6INT32
    for index in range(64, 128):
        fields[f"input_bit_register_{index}"] = FieldType.BOOL
        fields[f"output_bit_register_{index}"] = FieldType.BOOL
    for prefix in ("input", "output"):
        for index in range(48):
            fields[f"{prefix}_int_register_{index}"] = FieldType.INT32
            fields[f"{prefix}_double_register_{index}"] = FieldType.DOUBLE
    return fields


FIELD_TYPES: Mapping[str, FieldType] = MappingProxyType(_build_field_types())


def _default_value(field_type: FieldType) -> RTDEValue:
    if field_type is FieldType.BOOL:
        return False
    if field_type in (
        FieldType.UINT8,
        FieldType.UINT32,
        FieldType.UINT64,
        FieldType.INT32,
    ):
        return 0
    if field_type is FieldType.DOUBLE:
        return 0.0
    if field_type is FieldType.VECTOR3D:
        return (0.0, 0.0, 0.0)
    if field_type is FieldType.VECTOR6D:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    return (0, 0, 0, 0, 0, 0)


class DataPackage(RTDEPackage, MutableMapping[str, RTDEValue]):
    """A typed mutable mapping whose keys and layout are fixed by an RTDE recipe."""

    def __init__(
        self,
        recipe: Sequence[str],
        protocol_version: int = 2,
        *,
        recipe_id: int = 0,
        values: Mapping[str, object] | None = None,
    ) -> None:
        if isinstance(protocol_version, bool) or not isinstance(protocol_version, int):
            raise TypeError("RTDE protocol_version must be an integer")
        if protocol_version not in (1, 2):
            raise ValueError("RTDE protocol_version must be 1 or 2")
        normalized_recipe = tuple(recipe)
        if not normalized_recipe:
            raise ValueError("RTDE data package recipe cannot be empty")
        if len(set(normalized_recipe)) != len(normalized_recipe):
            raise ValueError("RTDE recipe cannot contain duplicate keys")
        for key in normalized_recipe:
            if key not in FIELD_TYPES:
                raise RTDEInvalidKeyError(f"Unknown item in RTDE recipe: {key!r}")
        self._recipe = normalized_recipe
        self._protocol_version = protocol_version
        self._recipe_id = 0
        self.recipe_id = recipe_id
        self._values = {key: _default_value(FIELD_TYPES[key]) for key in self._recipe}
        if values:
            for key, value in values.items():
                self[key] = value
        RTDEPackage.__init__(self, PackageType.DATA_PACKAGE, b"")

    @property
    def recipe(self) -> tuple[str, ...]:
        return self._recipe

    @property
    def protocol_version(self) -> int:
        return self._protocol_version

    @property
    def recipe_id(self) -> int:
        return self._recipe_id

    @recipe_id.setter
    def recipe_id(self, value: int) -> None:
        if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 0xFF:
            raise ValueError("recipe_id must fit an unsigned 8-bit integer")
        self._recipe_id = value

    def __getitem__(self, key: str) -> RTDEValue:
        try:
            return self._values[key]
        except KeyError as exc:
            raise RTDEInvalidKeyError(f"{key!r} is not part of this RTDE recipe") from exc

    def __setitem__(self, key: str, value: object) -> None:
        if key not in self._values:
            raise RTDEInvalidKeyError(f"{key!r} is not part of this RTDE recipe")
        self._values[key] = self._normalize(FIELD_TYPES[key], value, key)

    def __delitem__(self, key: str) -> None:
        if key not in self._values:
            raise RTDEInvalidKeyError(f"{key!r} is not part of this RTDE recipe")
        raise TypeError("RTDE recipe fields cannot be deleted")

    def __iter__(self) -> Iterator[str]:
        return iter(self._recipe)

    def __len__(self) -> int:
        return len(self._recipe)

    def copy(self) -> DataPackage:
        return type(self)(
            self._recipe,
            self._protocol_version,
            recipe_id=self._recipe_id,
            values=self._values,
        )

    @classmethod
    def from_payload(
        cls,
        recipe: Sequence[str],
        payload: bytes | bytearray | memoryview,
        protocol_version: int = 2,
    ) -> DataPackage:
        package = cls(recipe, protocol_version)
        package.parse_payload(payload)
        return package

    def parse_payload(self, payload: bytes | bytearray | memoryview) -> None:
        parser = BinParser(payload)
        if self._protocol_version == 2:
            self._recipe_id = parser.read_uint8()
        parsed: dict[str, RTDEValue] = {}
        for key in self._recipe:
            parsed[key] = self._parse_value(parser, FIELD_TYPES[key])
        if not parser.empty:
            raise ParseError(f"RTDE data package has {parser.remaining} trailing bytes")
        self._values = parsed
        object.__setattr__(self, "payload", bytes(payload))

    def serialize(self) -> bytes:
        payload = bytearray()
        if self._protocol_version == 2:
            payload.extend(struct.pack(">B", self._recipe_id))
        for key in self._recipe:
            field_type = FIELD_TYPES[key]
            value = self._values[key]
            values = value if isinstance(value, tuple) else (value,)
            payload.extend(struct.pack(f">{field_type.format_string}", *values))
        return PackageHeader.serialize(PackageType.DATA_PACKAGE, len(payload)) + payload

    serialize_package = serialize

    @staticmethod
    def _parse_value(parser: BinParser, field_type: FieldType) -> RTDEValue:
        value = parser.read(field_type.format_string)
        if field_type is FieldType.BOOL:
            return bool(value)
        if field_type in (
            FieldType.VECTOR3D,
            FieldType.VECTOR6D,
            FieldType.VECTOR6INT32,
            FieldType.VECTOR6UINT32,
        ):
            return cast(RTDEValue, tuple(value))
        if field_type is FieldType.DOUBLE:
            return float(value)
        return int(value)

    @staticmethod
    def _normalize(field_type: FieldType, value: object, key: str) -> RTDEValue:
        if field_type is FieldType.BOOL:
            if not isinstance(value, bool):
                raise TypeError(f"{key} must be a bool")
            return value
        integer_bounds = {
            FieldType.UINT8: (0, 0xFF),
            FieldType.UINT32: (0, 0xFFFFFFFF),
            FieldType.UINT64: (0, 0xFFFFFFFFFFFFFFFF),
            FieldType.INT32: (-(2**31), 2**31 - 1),
        }
        if field_type in integer_bounds:
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError(f"{key} must be an integer")
            minimum, maximum = integer_bounds[field_type]
            if not minimum <= value <= maximum:
                raise ValueError(f"{key} must be in [{minimum}, {maximum}]")
            return value
        if field_type is FieldType.DOUBLE:
            if isinstance(value, bool) or not isinstance(value, Real):
                raise TypeError(f"{key} must be a real number")
            return float(value)
        if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
            raise TypeError(f"{key} must be a sequence")
        expected_length = 3 if field_type is FieldType.VECTOR3D else 6
        normalized = tuple(value)
        if len(normalized) != expected_length:
            raise ValueError(f"{key} must contain exactly {expected_length} values")
        if field_type in (FieldType.VECTOR3D, FieldType.VECTOR6D):
            if any(isinstance(item, bool) or not isinstance(item, Real) for item in normalized):
                raise TypeError(f"{key} must contain real numbers")
            return cast(RTDEValue, tuple(float(item) for item in normalized))
        minimum, maximum = (
            (0, 0xFFFFFFFF) if field_type is FieldType.VECTOR6UINT32 else (-(2**31), 2**31 - 1)
        )
        if any(isinstance(item, bool) or not isinstance(item, int) for item in normalized):
            raise TypeError(f"{key} must contain integers")
        if any(not minimum <= item <= maximum for item in normalized):
            raise ValueError(f"{key} values must be in [{minimum}, {maximum}]")
        return cast(RTDEValue, normalized)

    def __repr__(self) -> str:
        fields = ", ".join(f"{key}={self._values[key]!r}" for key in self._recipe)
        return f"DataPackage(recipe_id={self._recipe_id}, {fields})"
