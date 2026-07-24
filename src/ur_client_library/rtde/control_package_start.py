"""RTDE start-control packages."""

from ur_client_library.rtde.rtde_package import ControlPackageStart, serialize_start_request


class ControlPackageStartRequest:
    @staticmethod
    def serialize() -> bytes:
        return serialize_start_request()


__all__ = ["ControlPackageStart", "ControlPackageStartRequest"]
