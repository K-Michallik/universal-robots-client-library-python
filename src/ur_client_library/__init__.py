"""Pure-Python Universal Robots client library."""

from importlib.metadata import PackageNotFoundError, version

from ur_client_library.example_robot_wrapper import ExampleRobotWrapper
from ur_client_library.exceptions import (
    CommandError,
    LifecycleError,
    ProtocolError,
    URCLConnectionError,
    URCLException,
    URCLTimeoutError,
)
from ur_client_library.logging import configure_logging
from ur_client_library.primary.client import PrimaryClient
from ur_client_library.types import MotionTarget, Pose, Q
from ur_client_library.ur.dashboard_client import DashboardClient, DashboardClientPolicy
from ur_client_library.ur.datatypes import (
    RobotMode,
    RobotSeries,
    RobotType,
    SafetyMode,
    SafetyStatus,
)
from ur_client_library.ur.instruction_executor import InstructionExecutor
from ur_client_library.ur.tool_communication import Parity, ToolCommSetup, ToolVoltage
from ur_client_library.ur.ur_driver import UrDriver, UrDriverConfiguration
from ur_client_library.ur.version_information import VersionInformation

try:
    __version__ = version("universal-robots-client-library-python")
except PackageNotFoundError:
    __version__ = "0.1.0"

__all__ = [
    "CommandError",
    "DashboardClient",
    "DashboardClientPolicy",
    "ExampleRobotWrapper",
    "InstructionExecutor",
    "LifecycleError",
    "MotionTarget",
    "Parity",
    "Pose",
    "PrimaryClient",
    "ProtocolError",
    "Q",
    "RobotMode",
    "RobotSeries",
    "RobotType",
    "SafetyMode",
    "SafetyStatus",
    "ToolCommSetup",
    "ToolVoltage",
    "URCLConnectionError",
    "URCLException",
    "URCLTimeoutError",
    "UrDriver",
    "UrDriverConfiguration",
    "VersionInformation",
    "__version__",
    "configure_logging",
]
