"""Big-endian serialization helpers used by UR wire protocols."""

from __future__ import annotations

import struct
from collections.abc import Sequence
from numbers import Real


class PackageSerializer:
    """Stateless big-endian serializer."""

    @staticmethod
    def pack(format_string: str, *values: object) -> bytes:
        wire_format = format_string if format_string[:1] in "@=<>!" else f">{format_string}"
        return struct.pack(wire_format, *values)

    @staticmethod
    def boolean(value: bool) -> bytes:
        if not isinstance(value, bool):
            raise TypeError("value must be a bool")
        return struct.pack(">B", value)

    @staticmethod
    def uint8(value: int) -> bytes:
        return PackageSerializer._integer("B", value, 0, 0xFF)

    @staticmethod
    def uint16(value: int) -> bytes:
        return PackageSerializer._integer("H", value, 0, 0xFFFF)

    @staticmethod
    def uint32(value: int) -> bytes:
        return PackageSerializer._integer("I", value, 0, 0xFFFFFFFF)

    @staticmethod
    def uint64(value: int) -> bytes:
        return PackageSerializer._integer("Q", value, 0, 0xFFFFFFFFFFFFFFFF)

    @staticmethod
    def int32(value: int) -> bytes:
        return PackageSerializer._integer("i", value, -(2**31), 2**31 - 1)

    @staticmethod
    def double(value: object) -> bytes:
        return struct.pack(">d", PackageSerializer._real(value))

    @staticmethod
    def string(value: str) -> bytes:
        if not isinstance(value, str):
            raise TypeError("value must be a string")
        return value.encode("utf-8")

    @staticmethod
    def doubles(values: Sequence[object], length: int) -> bytes:
        if isinstance(length, bool) or not isinstance(length, int):
            raise TypeError("length must be an integer")
        if length < 0:
            raise ValueError("length cannot be negative")
        normalized = tuple(values)
        if len(normalized) != length:
            raise ValueError(f"values must contain exactly {length} items")
        return struct.pack(
            f">{length}d",
            *(PackageSerializer._real(value) for value in normalized),
        )

    @staticmethod
    def _integer(format_string: str, value: int, minimum: int, maximum: int) -> bytes:
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError("value must be an integer")
        if not minimum <= value <= maximum:
            raise ValueError(f"value must be in [{minimum}, {maximum}]")
        return struct.pack(f">{format_string}", value)

    @staticmethod
    def _real(value: object) -> float:
        if isinstance(value, bool) or not isinstance(value, Real):
            raise TypeError("value must be a real number")
        return float(value)
