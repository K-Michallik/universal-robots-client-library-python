"""RTDE frame parser."""

from __future__ import annotations

from collections.abc import Sequence

from ur_client_library.comm.bin_parser import BinParser
from ur_client_library.exceptions import ParseError
from ur_client_library.rtde.data_package import DataPackage
from ur_client_library.rtde.package_header import PackageHeader, PackageType
from ur_client_library.rtde.rtde_package import (
    ControlPackagePause,
    ControlPackageSetupInputs,
    ControlPackageSetupOutputs,
    ControlPackageStart,
    GetUrcontrolVersion,
    RequestProtocolVersion,
    RTDEPackage,
    TextMessage,
)
from ur_client_library.ur.version_information import VersionInformation


class RTDEParser:
    """Parse one complete RTDE frame according to the negotiated recipe."""

    def __init__(self, recipe: Sequence[str], protocol_version: int = 1) -> None:
        self._recipe = tuple(recipe)
        self.protocol_version = protocol_version

    @property
    def recipe(self) -> tuple[str, ...]:
        return self._recipe

    @property
    def protocol_version(self) -> int:
        return self._protocol_version

    @protocol_version.setter
    def protocol_version(self, value: int) -> None:
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError("RTDE protocol version must be an integer")
        if value not in (1, 2):
            raise ValueError("RTDE protocol version must be 1 or 2")
        self._protocol_version = value

    def parse(self, data: bytes) -> RTDEPackage:
        header = PackageHeader.parse(data)
        if header.package_size != len(data):
            raise ParseError(
                f"RTDE header declares {header.package_size} bytes, received {len(data)}"
            )
        payload = data[PackageHeader.SIZE :]
        parser = BinParser(payload)
        package_type = header.package_type
        if package_type == PackageType.DATA_PACKAGE:
            package: RTDEPackage = DataPackage.from_payload(
                self._recipe,
                payload,
                self._protocol_version,
            )
            return package
        if package_type == PackageType.REQUEST_PROTOCOL_VERSION:
            package = RequestProtocolVersion(parser.read_bool())
        elif package_type == PackageType.GET_URCONTROL_VERSION:
            package = GetUrcontrolVersion(
                VersionInformation(
                    parser.read_uint32(),
                    parser.read_uint32(),
                    parser.read_uint32(),
                    parser.read_uint32(),
                )
            )
        elif package_type == PackageType.TEXT_MESSAGE:
            package = self._parse_text(parser, payload)
        elif package_type == PackageType.CONTROL_PACKAGE_SETUP_INPUTS:
            input_recipe_id = parser.read_uint8()
            package = ControlPackageSetupInputs(
                input_recipe_id,
                self._types(parser.read_remainder_string()),
                payload,
            )
        elif package_type == PackageType.CONTROL_PACKAGE_SETUP_OUTPUTS:
            output_recipe_id = parser.read_uint8() if self._protocol_version == 2 else None
            package = ControlPackageSetupOutputs(
                output_recipe_id,
                self._types(parser.read_remainder_string()),
                self._protocol_version,
                payload,
            )
        elif package_type == PackageType.CONTROL_PACKAGE_START:
            package = ControlPackageStart(parser.read_bool())
        elif package_type == PackageType.CONTROL_PACKAGE_PAUSE:
            package = ControlPackagePause(parser.read_bool())
        else:
            parser.consume()
            package = RTDEPackage(package_type, payload)
        if not parser.empty:
            raise ParseError(
                f"RTDE package type {int(package_type)} has {parser.remaining} trailing bytes"
            )
        return package

    def _parse_text(self, parser: BinParser, payload: bytes) -> TextMessage:
        if self._protocol_version == 2:
            message = parser.read_string()
            source = parser.read_string()
            warning_level = parser.read_uint8()
            return TextMessage(
                message,
                source=source,
                warning_level=warning_level,
                protocol_version=2,
                payload=payload,
            )
        message_type = parser.read_uint8()
        return TextMessage(
            parser.read_remainder_string(),
            message_type=message_type,
            protocol_version=1,
            payload=payload,
        )

    @staticmethod
    def _types(value: str) -> tuple[str, ...]:
        return tuple(value.split(",")) if value else ()
