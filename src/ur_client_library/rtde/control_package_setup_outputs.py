"""RTDE output-recipe setup packages."""

from collections.abc import Sequence

from ur_client_library.rtde.rtde_package import (
    ControlPackageSetupOutputs,
    serialize_setup_outputs_request,
)


class ControlPackageSetupOutputsRequest:
    @staticmethod
    def serialize(
        recipe: Sequence[str],
        *,
        protocol_version: int,
        frequency: float = 0.0,
    ) -> bytes:
        return serialize_setup_outputs_request(
            tuple(recipe),
            protocol_version=protocol_version,
            frequency=frequency,
        )


__all__ = ["ControlPackageSetupOutputs", "ControlPackageSetupOutputsRequest"]
