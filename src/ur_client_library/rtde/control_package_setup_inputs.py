"""RTDE input-recipe setup packages."""

from collections.abc import Sequence

from ur_client_library.rtde.rtde_package import (
    ControlPackageSetupInputs,
    serialize_setup_inputs_request,
)


class ControlPackageSetupInputsRequest:
    @staticmethod
    def serialize(recipe: Sequence[str]) -> bytes:
        return serialize_setup_inputs_request(tuple(recipe))


__all__ = ["ControlPackageSetupInputs", "ControlPackageSetupInputsRequest"]
