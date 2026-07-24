"""Parser extension-point protocol."""

from __future__ import annotations

from typing import Protocol, TypeVar

T = TypeVar("T", covariant=True)


class Parser(Protocol[T]):
    def parse(self, data: bytes) -> T: ...
