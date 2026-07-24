"""RTDE pause-control packages."""

from ur_client_library.rtde.rtde_package import ControlPackagePause, serialize_pause_request


class ControlPackagePauseRequest:
    @staticmethod
    def serialize() -> bytes:
        return serialize_pause_request()


__all__ = ["ControlPackagePause", "ControlPackagePauseRequest"]
