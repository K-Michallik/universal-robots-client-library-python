"""RTDE protocol-version request packages."""

from ur_client_library.rtde.rtde_package import (
    RequestProtocolVersion,
    serialize_protocol_version_request,
)


class RequestProtocolVersionRequest:
    @staticmethod
    def serialize(version: int) -> bytes:
        return serialize_protocol_version_request(version)


__all__ = ["RequestProtocolVersion", "RequestProtocolVersionRequest"]
