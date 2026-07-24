"""Bounded helpers for parsing Universal Robots' big-endian wire formats."""

from __future__ import annotations

import struct
from contextlib import AbstractContextManager
from types import TracebackType
from typing import Any, cast

from ur_client_library.exceptions import ParseError


class BinParser(AbstractContextManager["BinParser"]):
    """A cursor over a bounded byte buffer.

    Child parsers cannot read beyond their declared region.  When used as a
    context manager, the parent cursor advances by the bytes consumed by the
    child, matching URCL's sub-parser behavior.
    """

    def __init__(
        self,
        data: bytes | bytearray | memoryview,
        *,
        _parent: BinParser | None = None,
        _start: int = 0,
        _length: int | None = None,
    ) -> None:
        self._buffer = memoryview(data).cast("B")
        self._start = _start
        self._position = _start
        length = len(self._buffer) - _start if _length is None else _length
        self._end = _start + length
        self._parent = _parent
        if _start < 0 or length < 0 or self._end > len(self._buffer):
            raise ParseError("Parser bounds exceed the supplied buffer")

    @property
    def position(self) -> int:
        return self._position - self._start

    @property
    def remaining(self) -> int:
        return self._end - self._position

    @property
    def empty(self) -> bool:
        return self._position == self._end

    def check_size(self, size: int) -> bool:
        return isinstance(size, int) and size >= 0 and size <= self.remaining

    def _require(self, size: int) -> None:
        if not self.check_size(size):
            raise ParseError(
                f"Could not parse received package: need {size} bytes, only {self.remaining} remain"
            )

    def peek(self, format_string: str) -> Any:
        wire_format = format_string if format_string[:1] in "@=<>!" else f">{format_string}"
        size = struct.calcsize(wire_format)
        self._require(size)
        values = struct.unpack_from(wire_format, self._buffer, self._position)
        return values[0] if len(values) == 1 else values

    def read(self, format_string: str) -> Any:
        wire_format = format_string if format_string[:1] in "@=<>!" else f">{format_string}"
        value = self.peek(wire_format)
        self._position += struct.calcsize(wire_format)
        return value

    def read_bool(self) -> bool:
        return self.read_uint8() != 0

    def read_uint8(self) -> int:
        return cast(int, self.read("B"))

    def read_int8(self) -> int:
        return cast(int, self.read("b"))

    def read_uint16(self) -> int:
        return cast(int, self.read("H"))

    def read_int16(self) -> int:
        return cast(int, self.read("h"))

    def read_uint32(self) -> int:
        return cast(int, self.read("I"))

    def read_int32(self) -> int:
        return cast(int, self.read("i"))

    def read_uint64(self) -> int:
        return cast(int, self.read("Q"))

    def read_int64(self) -> int:
        return cast(int, self.read("q"))

    def read_float(self) -> float:
        return cast(float, self.read("f"))

    def read_double(self) -> float:
        return cast(float, self.read("d"))

    def read_bytes(self, length: int) -> bytes:
        self._require(length)
        start = self._position
        self._position += length
        return bytes(self._buffer[start : self._position])

    def read_string(self, length: int | None = None, *, encoding: str = "utf-8") -> str:
        if length is None:
            length = self.read_uint8()
        try:
            return self.read_bytes(length).decode(encoding)
        except UnicodeDecodeError as exc:
            raise ParseError(f"Invalid {encoding} string in received package") from exc

    def read_remainder(self) -> bytes:
        return self.read_bytes(self.remaining)

    def read_remainder_string(self, *, encoding: str = "utf-8") -> str:
        return self.read_string(self.remaining, encoding=encoding)

    def consume(self, size: int | None = None) -> None:
        amount = self.remaining if size is None else size
        self._require(amount)
        self._position += amount

    def subparser(self, length: int) -> BinParser:
        self._require(length)
        return type(self)(
            self._buffer,
            _parent=self,
            _start=self._position,
            _length=length,
        )

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        if self._parent is not None:
            self._parent._position = self._position
