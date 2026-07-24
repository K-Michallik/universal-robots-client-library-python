"""PolyScope version representation."""

from __future__ import annotations

from dataclasses import dataclass

from ur_client_library.exceptions import ProtocolError


@dataclass(frozen=True, slots=True, order=True)
class VersionInformation:
    """A four-component PolyScope version."""

    major: int = 0
    minor: int = 0
    bugfix: int = 0
    build: int = 0

    def __post_init__(self) -> None:
        for name, value in (
            ("major", self.major),
            ("minor", self.minor),
            ("bugfix", self.bugfix),
            ("build", self.build),
        ):
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError(f"{name} must be an integer")
            if not 0 <= value <= 0xFFFFFFFF:
                raise ValueError(f"{name} must fit an unsigned 32-bit integer")

    @classmethod
    def from_string(cls, value: str) -> VersionInformation:
        """Parse ``major.minor[.bugfix[.build]]``."""
        components = value.split(".")
        if not 2 <= len(components) <= 4 or any(not item for item in components):
            raise ProtocolError(f"{value!r} is not a supported version string")
        try:
            numbers = [int(item, 10) for item in components]
        except ValueError as exc:
            raise ProtocolError(f"{value!r} is not a supported version string") from exc
        numbers.extend([0] * (4 - len(numbers)))
        try:
            return cls(*numbers)
        except (TypeError, ValueError) as exc:
            raise ProtocolError(f"{value!r} is not a supported version string") from exc

    def to_string(self) -> str:
        return f"{self.major}.{self.minor}.{self.bugfix}.{self.build}"

    @property
    def is_e_series(self) -> bool:
        return self.major >= 5

    @property
    def is_empty(self) -> bool:
        return self == type(self)()

    def __str__(self) -> str:
        return self.to_string()
