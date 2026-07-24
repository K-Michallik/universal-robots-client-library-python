"""RTDE controller-version request packages."""

from ur_client_library.rtde.rtde_package import (
    GetUrcontrolVersion,
    serialize_get_urcontrol_version_request,
)


class GetUrcontrolVersionRequest:
    @staticmethod
    def serialize() -> bytes:
        return serialize_get_urcontrol_version_request()


__all__ = ["GetUrcontrolVersion", "GetUrcontrolVersionRequest"]
