"""Tool serial-communication configuration."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum
from numbers import Real

from ur_client_library.exceptions import (
    InvalidRangeError,
    ToolCommunicationUnavailableError,
)
from ur_client_library.ur.version_information import VersionInformation


class ToolVoltage(IntEnum):
    OFF = 0
    V12 = 12
    V24 = 24


class Parity(IntEnum):
    NONE = 0
    ODD = 1
    EVEN = 2


_ALLOWED_BAUD_RATES = frozenset(
    {9_600, 19_200, 38_400, 57_600, 115_200, 1_000_000, 2_000_000, 5_000_000}
)


def _bounded_real(value: object, lower: float, upper: float, *, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a real number")
    result = float(value)
    if not math.isfinite(result) or not lower <= result <= upper:
        raise InvalidRangeError(f"{name} must be in range [{lower:g}, {upper:g}]")
    return result


@dataclass(slots=True)
class ToolCommSetup:
    tool_voltage: ToolVoltage = ToolVoltage.OFF
    parity: Parity = Parity.ODD
    baud_rate: int = 9_600
    stop_bits: int = 1
    rx_idle_chars: float = 1.0
    tx_idle_chars: float = 0.0

    def __post_init__(self) -> None:
        self.set_tool_voltage(self.tool_voltage)
        self.set_parity(self.parity)
        self.set_baud_rate(self.baud_rate)
        self.set_stop_bits(self.stop_bits)
        self.set_rx_idle_chars(self.rx_idle_chars)
        self.set_tx_idle_chars(self.tx_idle_chars)

    def set_tool_voltage(self, value: ToolVoltage | int) -> None:
        if isinstance(value, bool):
            raise TypeError("tool_voltage must be a ToolVoltage")
        try:
            self.tool_voltage = ToolVoltage(value)
        except ValueError as exc:
            raise ValueError(f"Unsupported tool voltage: {value}") from exc

    def set_parity(self, value: Parity | int) -> None:
        if isinstance(value, bool):
            raise TypeError("parity must be a Parity")
        try:
            self.parity = Parity(value)
        except ValueError as exc:
            raise ValueError(f"Unsupported parity: {value}") from exc

    def set_baud_rate(self, value: int) -> None:
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError("baud_rate must be an integer")
        if value not in _ALLOWED_BAUD_RATES:
            allowed = ", ".join(str(item) for item in sorted(_ALLOWED_BAUD_RATES))
            raise InvalidRangeError(f"baud_rate must be one of: {allowed}")
        self.baud_rate = value

    def set_stop_bits(self, value: int) -> None:
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError("stop_bits must be an integer")
        if value not in (1, 2):
            raise InvalidRangeError("stop_bits must be 1 or 2")
        self.stop_bits = value

    def set_rx_idle_chars(self, value: float) -> None:
        self.rx_idle_chars = _bounded_real(value, 1.0, 40.0, name="rx_idle_chars")

    def set_tx_idle_chars(self, value: float) -> None:
        self.tx_idle_chars = _bounded_real(value, 0.0, 40.0, name="tx_idle_chars")

    def to_urscript(self, robot_version: VersionInformation) -> str:
        """Return the exact ``BEGIN_REPLACE`` tool setup snippet."""
        minimum = VersionInformation.from_string("5.0.0")
        if robot_version < minimum:
            raise ToolCommunicationUnavailableError(
                "Tool communication setup requires PolyScope 5 or later",
                minimum,
                robot_version,
            )
        return (
            f"set_tool_voltage({int(self.tool_voltage)})\n"
            "set_tool_communication("
            f"True, {self.baud_rate}, {int(self.parity)}, {self.stop_bits}, "
            f"{self.rx_idle_chars:g}, {self.tx_idle_chars:g})"
        )
