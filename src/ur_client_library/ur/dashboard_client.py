"""Facade and shared value types for Universal Robots dashboard clients."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, cast

from ur_client_library.ur.version_information import VersionInformation


class DashboardClientPolicy(str, Enum):
    """Explicitly select the controller's dashboard transport."""

    G5 = "g5"
    POLYSCOPE_X = "polyscope_x"


DashboardBackend = DashboardClientPolicy
ClientPolicy = DashboardClientPolicy


@dataclass(frozen=True, slots=True)
class ProgramInformation:
    """Metadata returned by the PolyScope X programs endpoint."""

    created_date: int
    description: str
    last_modified_date: int
    last_saved_date: int
    name: str
    program_state: str


@dataclass(frozen=True, slots=True)
class ProgramState:
    """Current program state and, when available, program name."""

    state: str
    program_name: str | None = None


@dataclass(frozen=True, slots=True)
class ProgramSaveState:
    """Whether the loaded G5 program is saved."""

    saved: bool
    program_name: str


@dataclass(frozen=True, slots=True)
class DashboardResponse:
    """Successful command response.

    Rejected commands raise :class:`DashboardCommandError`; consequently a
    returned response is normally successful. ``ok`` remains available to make
    the result self-describing and to mirror Robot API response handling.
    """

    ok: bool = True
    message: str = ""
    data: dict[str, object] = field(default_factory=dict)
    status_code: int | None = None


class DashboardClient:
    """Transport-selecting dashboard facade.

    ``backend`` is mandatory: transport selection never probes the controller
    and construction performs no network I/O.
    """

    ClientPolicy = DashboardClientPolicy

    def __init__(
        self,
        host: str,
        backend: DashboardClientPolicy | str,
        **backend_options: Any,
    ) -> None:
        if not isinstance(host, str) or not host.strip():
            raise ValueError("host must be a non-empty string")

        policy = self._parse_policy(backend)
        implementation: Any
        if policy is DashboardClientPolicy.G5:
            from ur_client_library.ur.dashboard_g5 import DashboardClientG5

            implementation = DashboardClientG5(host, **backend_options)
        else:
            from ur_client_library.ur.dashboard_x import DashboardClientX

            implementation = DashboardClientX(host, **backend_options)

        self._policy = policy
        self._implementation: Any = implementation

    @staticmethod
    def _parse_policy(backend: DashboardClientPolicy | str) -> DashboardClientPolicy:
        if isinstance(backend, DashboardClientPolicy):
            return backend
        if not isinstance(backend, str):
            raise TypeError("backend must be a DashboardClientPolicy or string")
        normalized = backend.strip().lower().replace("-", "_")
        if normalized == "x":
            normalized = DashboardClientPolicy.POLYSCOPE_X.value
        try:
            return DashboardClientPolicy(normalized)
        except ValueError as exc:
            choices = ", ".join(policy.value for policy in DashboardClientPolicy)
            raise ValueError(
                f"unknown dashboard backend {backend!r}; expected one of {choices}"
            ) from exc

    @property
    def policy(self) -> DashboardClientPolicy:
        return self._policy

    @property
    def implementation(self) -> object:
        """The selected backend, primarily for diagnostics and testing."""

        return self._implementation

    @property
    def is_connected(self) -> bool:
        return bool(self._implementation.is_connected)

    @property
    def version(self) -> VersionInformation:
        """The G5 PolyScope version or PolyScope X Robot API version."""

        return cast(VersionInformation, self._implementation.version)

    @property
    def polyscope_version(self) -> VersionInformation:
        """The legacy PolyScope version.

        PolyScope X does not expose this query and raises
        :class:`NotImplementedError`.
        """

        return cast(VersionInformation, self._implementation.polyscope_version)

    @property
    def receive_timeout(self) -> float:
        return float(self._implementation.receive_timeout)

    @receive_timeout.setter
    def receive_timeout(self, timeout: float) -> None:
        self._implementation.receive_timeout = timeout

    def set_receive_timeout(self, timeout: float) -> None:
        self.receive_timeout = timeout

    def get_configured_receive_timeout(self) -> float:
        return self.receive_timeout

    def connect(
        self,
        *,
        max_attempts: int = 0,
        retry_interval: float = 10.0,
    ) -> DashboardClient:
        self._implementation.connect(
            max_attempts=max_attempts,
            retry_interval=retry_interval,
        )
        return self

    def close(self) -> None:
        self._implementation.close()

    disconnect = close

    def __enter__(self) -> DashboardClient:
        return self.connect()

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: object,
    ) -> None:
        self.close()

    def __getattr__(self, name: str) -> Any:
        # Command methods intentionally live on the implementations. Delegating
        # here keeps the facade's supported surface identical to the selected
        # backend without duplicating dozens of pass-through methods.
        return getattr(self._implementation, name)


def _download_result(path: str | Path, response: DashboardResponse) -> DashboardResponse:
    """Attach a normalized local path to a successful download response."""

    data = dict(response.data)
    data["path"] = Path(path)
    return DashboardResponse(
        ok=response.ok,
        message=response.message,
        data=data,
        status_code=response.status_code,
    )


__all__ = [
    "ClientPolicy",
    "DashboardBackend",
    "DashboardClient",
    "DashboardClientPolicy",
    "DashboardResponse",
    "ProgramInformation",
    "ProgramSaveState",
    "ProgramState",
]
