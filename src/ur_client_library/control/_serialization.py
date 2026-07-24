"""Shared fixed-point helpers for external-control wire packets."""

from __future__ import annotations

import math
import struct
from collections.abc import Iterable
from numbers import Real

from ur_client_library.exceptions import InvalidRangeError
from ur_client_library.helpers import round_half_away_from_zero

INT32_MIN = -(1 << 31)
INT32_MAX = (1 << 31) - 1


def int32(value: int, *, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{name} must be an integer")
    if not INT32_MIN <= value <= INT32_MAX:
        raise InvalidRangeError(f"{name} must fit a signed 32-bit integer")
    return value


def fixed(value: object, multiplier: int, *, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a real number")
    numeric = float(value)
    if not math.isfinite(numeric):
        raise InvalidRangeError(f"{name} must be finite")
    return int32(round_half_away_from_zero(numeric * multiplier), name=name)


def pack_int32s(values: Iterable[int]) -> bytes:
    normalized = tuple(values)
    for index, value in enumerate(normalized):
        int32(value, name=f"word[{index}]")
    return struct.pack(f"!{len(normalized)}i", *normalized)
