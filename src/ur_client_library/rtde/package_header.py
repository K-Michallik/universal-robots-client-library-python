"""RTDE package header constants and serialization."""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum

from ur_client_library.exceptions import ParseError


class PackageType(IntEnum):
    REQUEST_PROTOCOL_VERSION = 86
    GET_URCONTROL_VERSION = 118
    TEXT_MESSAGE = 77
    DATA_PACKAGE = 85
    CONTROL_PACKAGE_SETUP_OUTPUTS = 79
    CONTROL_PACKAGE_SETUP_INPUTS = 73
    CONTROL_PACKAGE_START = 83
    CONTROL_PACKAGE_PAUSE = 80


@dataclass(frozen=True, slots=True)
class PackageHeader:
    package_size: int
    package_type: PackageType | int

    SIZE = 3

    @classmethod
    def parse(cls, data: bytes | bytearray | memoryview) -> PackageHeader:
        if len(data) < cls.SIZE:
            raise ParseError("RTDE package is shorter than its three-byte header")
        package_size, raw_type = struct.unpack_from(">HB", data)
        if package_size < cls.SIZE:
            raise ParseError(f"Invalid RTDE package size {package_size}")
        try:
            package_type: PackageType | int = PackageType(raw_type)
        except ValueError:
            package_type = raw_type
        return cls(package_size, package_type)

    @classmethod
    def serialize(cls, package_type: PackageType | int, payload_length: int) -> bytes:
        if isinstance(payload_length, bool) or not isinstance(payload_length, int):
            raise TypeError("payload_length must be an integer")
        total_size = cls.SIZE + payload_length
        if not cls.SIZE <= total_size <= 0xFFFF:
            raise ValueError("RTDE package size must fit an unsigned 16-bit integer")
        return struct.pack(">HB", total_size, int(package_type))

    @staticmethod
    def get_package_length(prefix: bytes | bytearray | memoryview) -> int:
        if len(prefix) < 2:
            raise ParseError("RTDE package length prefix is incomplete")
        return int(struct.unpack_from(">H", prefix)[0])
