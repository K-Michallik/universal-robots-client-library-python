"""Package extension-point protocol."""

from __future__ import annotations

from typing import Protocol


class Package(Protocol):
    def __str__(self) -> str: ...
