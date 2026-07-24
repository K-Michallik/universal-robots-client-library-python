"""Typed RTDE control and informational packages."""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field
from numbers import Real

from ur_client_library.rtde.package_header import PackageHeader, PackageType
from ur_client_library.ur.version_information import VersionInformation


@dataclass(slots=True)
class RTDEPackage:
    package_type: PackageType | int
    payload: bytes = b""


@dataclass(slots=True)
class RequestProtocolVersion(RTDEPackage):
    accepted: bool = False

    def __init__(self, accepted: bool) -> None:
        object.__setattr__(self, "package_type", PackageType.REQUEST_PROTOCOL_VERSION)
        object.__setattr__(self, "payload", bytes((int(accepted),)))
        object.__setattr__(self, "accepted", bool(accepted))


@dataclass(slots=True)
class GetUrcontrolVersion(RTDEPackage):
    version: VersionInformation = field(default_factory=VersionInformation)

    def __init__(self, version: VersionInformation) -> None:
        object.__setattr__(self, "package_type", PackageType.GET_URCONTROL_VERSION)
        object.__setattr__(self, "payload", struct.pack(">4I", *version_components(version)))
        object.__setattr__(self, "version", version)


@dataclass(slots=True)
class TextMessage(RTDEPackage):
    message: str = ""
    source: str = ""
    warning_level: int = 0
    message_type: int = 0
    protocol_version: int = 2

    def __init__(
        self,
        message: str,
        *,
        source: str = "",
        warning_level: int = 0,
        message_type: int = 0,
        protocol_version: int = 2,
        payload: bytes = b"",
    ) -> None:
        object.__setattr__(self, "package_type", PackageType.TEXT_MESSAGE)
        object.__setattr__(self, "payload", payload)
        object.__setattr__(self, "message", message)
        object.__setattr__(self, "source", source)
        object.__setattr__(self, "warning_level", warning_level)
        object.__setattr__(self, "message_type", message_type)
        object.__setattr__(self, "protocol_version", protocol_version)


@dataclass(slots=True)
class ControlPackageSetupInputs(RTDEPackage):
    recipe_id: int = 0
    variable_types: tuple[str, ...] = ()

    def __init__(
        self, recipe_id: int, variable_types: tuple[str, ...], payload: bytes = b""
    ) -> None:
        object.__setattr__(self, "package_type", PackageType.CONTROL_PACKAGE_SETUP_INPUTS)
        object.__setattr__(self, "payload", payload)
        object.__setattr__(self, "recipe_id", recipe_id)
        object.__setattr__(self, "variable_types", variable_types)

    @property
    def variable_types_string(self) -> str:
        return ",".join(self.variable_types)


@dataclass(slots=True)
class ControlPackageSetupOutputs(RTDEPackage):
    recipe_id: int | None = None
    variable_types: tuple[str, ...] = ()
    protocol_version: int = 2

    def __init__(
        self,
        recipe_id: int | None,
        variable_types: tuple[str, ...],
        protocol_version: int,
        payload: bytes = b"",
    ) -> None:
        object.__setattr__(self, "package_type", PackageType.CONTROL_PACKAGE_SETUP_OUTPUTS)
        object.__setattr__(self, "payload", payload)
        object.__setattr__(self, "recipe_id", recipe_id)
        object.__setattr__(self, "variable_types", variable_types)
        object.__setattr__(self, "protocol_version", protocol_version)

    @property
    def variable_types_string(self) -> str:
        return ",".join(self.variable_types)


@dataclass(slots=True)
class ControlPackageStart(RTDEPackage):
    accepted: bool = False

    def __init__(self, accepted: bool) -> None:
        object.__setattr__(self, "package_type", PackageType.CONTROL_PACKAGE_START)
        object.__setattr__(self, "payload", bytes((int(accepted),)))
        object.__setattr__(self, "accepted", bool(accepted))


@dataclass(slots=True)
class ControlPackagePause(RTDEPackage):
    accepted: bool = False

    def __init__(self, accepted: bool) -> None:
        object.__setattr__(self, "package_type", PackageType.CONTROL_PACKAGE_PAUSE)
        object.__setattr__(self, "payload", bytes((int(accepted),)))
        object.__setattr__(self, "accepted", bool(accepted))


def version_components(version: VersionInformation) -> tuple[int, int, int, int]:
    return version.major, version.minor, version.bugfix, version.build


def serialize_protocol_version_request(version: int) -> bytes:
    if isinstance(version, bool) or not isinstance(version, int) or not 0 <= version <= 0xFFFF:
        raise ValueError("protocol version must fit an unsigned 16-bit integer")
    payload = struct.pack(">H", version)
    return PackageHeader.serialize(PackageType.REQUEST_PROTOCOL_VERSION, len(payload)) + payload


def serialize_get_urcontrol_version_request() -> bytes:
    return PackageHeader.serialize(PackageType.GET_URCONTROL_VERSION, 0)


def serialize_setup_inputs_request(recipe: tuple[str, ...] | list[str]) -> bytes:
    if not recipe:
        return b""
    payload = ",".join(recipe).encode("utf-8")
    return PackageHeader.serialize(PackageType.CONTROL_PACKAGE_SETUP_INPUTS, len(payload)) + payload


def serialize_setup_outputs_request(
    recipe: tuple[str, ...] | list[str],
    *,
    protocol_version: int,
    frequency: float,
) -> bytes:
    if not recipe:
        return b""
    if isinstance(protocol_version, bool) or not isinstance(protocol_version, int):
        raise TypeError("protocol_version must be an integer")
    normalized_frequency = _normalize_frequency(frequency)
    names = ",".join(recipe).encode("utf-8")
    if protocol_version == 2:
        payload = struct.pack(">d", normalized_frequency) + names
    elif protocol_version == 1:
        payload = names
    else:
        raise ValueError(f"Unsupported RTDE protocol version {protocol_version}")
    return (
        PackageHeader.serialize(PackageType.CONTROL_PACKAGE_SETUP_OUTPUTS, len(payload)) + payload
    )


def serialize_start_request() -> bytes:
    return PackageHeader.serialize(PackageType.CONTROL_PACKAGE_START, 0)


def serialize_pause_request() -> bytes:
    return PackageHeader.serialize(PackageType.CONTROL_PACKAGE_PAUSE, 0)


def _normalize_frequency(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError("frequency must be a real number")
    normalized = float(value)
    if not math.isfinite(normalized) or normalized < 0:
        raise ValueError("frequency must be finite and non-negative")
    return normalized
