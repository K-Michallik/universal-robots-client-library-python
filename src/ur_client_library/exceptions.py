"""Exception hierarchy for :mod:`ur_client_library`."""

from __future__ import annotations

import builtins
from collections.abc import Sequence
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ur_client_library.ur.datatypes import RobotMode, SafetyMode
    from ur_client_library.ur.version_information import VersionInformation


class URCLException(Exception):
    """Base class for all library-defined exceptions."""


class LifecycleError(URCLException):
    """An operation is invalid in the object's current lifecycle state."""


class URCLConnectionError(URCLException, builtins.ConnectionError):
    """A robot connection could not be established or was lost."""


class StreamNotConnectedError(URCLConnectionError):
    """A stream operation was attempted without an active connection."""


class ProtocolError(URCLException):
    """A peer violated the expected wire protocol."""


class ParseError(ProtocolError):
    """A received package could not be parsed safely."""


class UnexpectedResponseError(ProtocolError):
    """A peer returned a syntactically valid but unexpected response."""

    def __init__(self, message: str, *, status: int | None = None) -> None:
        self.status = status
        suffix = "" if status is None else f" (status code: {status})"
        super().__init__(f"{message}{suffix}")


class URCLTimeoutError(URCLException, TimeoutError):
    """An operation exceeded its configured timeout."""

    def __init__(self, message: str, timeout: float | None = None) -> None:
        self.timeout = timeout
        suffix = "" if timeout is None else f" (configured timeout: {timeout:g} sec)"
        super().__init__(f"{message}{suffix}")


class WorkerThreadError(URCLException):
    """A background worker failed."""

    def __init__(self, worker: str, cause: BaseException) -> None:
        self.worker = worker
        self.cause = cause
        super().__init__(f"{worker} worker failed: {cause}")


class VersionMismatchError(URCLException):
    """The connected controller does not satisfy a version requirement."""

    def __init__(
        self,
        message: str,
        required: VersionInformation,
        actual: VersionInformation,
    ) -> None:
        self.required = required
        self.actual = actual
        super().__init__(f"{message} (required version: {required}, actual version: {actual})")


class ToolCommunicationUnavailableError(VersionMismatchError):
    """Tool communication is unavailable for the controller version."""


class IncompatibleRobotVersionError(VersionMismatchError):
    """A requested feature is unavailable on the connected controller."""


class InvalidRangeError(URCLException, ValueError):
    """A value lies outside the protocol's accepted range."""


class MissingArgumentError(URCLException, ValueError):
    """A required command argument was omitted."""


class UnsupportedMotionTypeError(URCLException):
    """A motion target or motion type is unsupported."""


class UnknownVariableError(URCLException, KeyError):
    """A named protocol variable is unknown."""


class RTDEInvalidKeyError(URCLException, KeyError):
    """An RTDE package key is not part of its recipe."""


class RTDEInputConflictError(URCLException):
    """An RTDE input is already controlled by another client."""

    def __init__(self, key: str) -> None:
        self.key = key
        super().__init__(
            f"Variable {key!r} is controlled by another RTDE client. The input recipe "
            "cannot be used as configured. A fieldbus can claim all outputs."
        )


class CommandError(URCLException):
    """A robot command failed or was rejected."""


class DashboardCommandError(CommandError):
    """A dashboard command was rejected."""

    def __init__(self, command: str, response: str, *, status: int | None = None) -> None:
        self.command = command
        self.response = response
        self.status = status
        suffix = "" if status is None else f" (status code: {status})"
        super().__init__(f"Dashboard command {command!r} was rejected: {response}{suffix}")


class ScriptSyntaxError(URCLException, ValueError):
    """A script template contains invalid syntax."""


class ScriptIncludeError(ScriptSyntaxError):
    """A script include is rooted or resolves outside the top-level script directory."""


class ScriptFileError(URCLException, builtins.OSError):
    """A script template or one of its includes could not be resolved or read."""


class ScriptExecutionError(CommandError):
    """A URScript program failed while executing."""


class RobotRuntimeError(ScriptExecutionError):
    """The controller reported a runtime exception."""


class ReadOnlyInterfaceError(ScriptExecutionError):
    """The controller rejected a script on a read-only interface."""


class RobotErrorCodeError(ScriptExecutionError):
    """The controller emitted an error-code message during script execution."""


class RobotModeError(CommandError):
    """The robot is in the wrong mode for an operation."""

    def __init__(self, operation: str, required: RobotMode, actual: RobotMode) -> None:
        self.operation = operation
        self.required = required
        self.actual = actual
        super().__init__(
            f"Robot mode for {operation!r} must be {required.name} ({required.value}); "
            f"got {actual.name} ({actual.value})"
        )


class SafetyModeError(CommandError):
    """The robot is in the wrong safety mode for an operation."""

    def __init__(
        self,
        operation: str,
        allowed: Sequence[SafetyMode],
        actual: SafetyMode,
    ) -> None:
        self.operation = operation
        self.allowed = tuple(allowed)
        self.actual = actual
        expected = ", ".join(f"{mode.name} ({mode.value})" for mode in allowed)
        super().__init__(
            f"Safety mode for {operation!r} must be one of {expected}; "
            f"got {actual.name} ({actual.value})"
        )
