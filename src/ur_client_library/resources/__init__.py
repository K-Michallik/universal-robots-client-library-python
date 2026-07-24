"""Packaged external-control protocol resources."""

from __future__ import annotations

import math
from contextlib import AbstractContextManager
from importlib.resources import as_file, files
from numbers import Real
from pathlib import Path

from ur_client_library.control.reverse_interface import (
    DEFAULT_REVERSE_PORT,
    DEFAULT_SCRIPT_COMMAND_PORT,
    DEFAULT_TRAJECTORY_PORT,
    MULT_JOINTSTATE,
    MULT_TIME,
)
from ur_client_library.control.script_reader import ScriptData
from ur_client_library.ur.tool_communication import ToolCommSetup
from ur_client_library.ur.version_information import VersionInformation


def _validate_servoj_lookahead_time(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError("servoj_lookahead_time must be a real number")
    result = float(value)
    if not math.isfinite(result) or not 0.03 <= result <= 0.2:
        raise ValueError("servoj_lookahead_time must be in range [0.03, 0.2]")
    return result


def external_control_script() -> str:
    return files(__package__).joinpath("external_control.urscript").read_text()


def external_control_script_path() -> AbstractContextManager[Path]:
    return as_file(files(__package__).joinpath("external_control.urscript"))


def external_control_script_data(
    *,
    server_ip: str,
    robot_software_version: VersionInformation,
    reverse_port: int = DEFAULT_REVERSE_PORT,
    trajectory_port: int = DEFAULT_TRAJECTORY_PORT,
    script_command_port: int = DEFAULT_SCRIPT_COMMAND_PORT,
    servoj_lookahead_time: float = 0.03,
    servoj_gain: int = 2000,
    tool_comm_setup: ToolCommSetup | None = None,
) -> ScriptData:
    """Build the current substitution dictionary used by ``external_control.urscript``."""
    for name, port in (
        ("reverse_port", reverse_port),
        ("trajectory_port", trajectory_port),
        ("script_command_port", script_command_port),
    ):
        if isinstance(port, bool) or not isinstance(port, int):
            raise TypeError(f"{name} must be an integer")
        if not 0 <= port <= 65535:
            raise ValueError(f"{name} must be in range [0, 65535]")
    lookahead_time = _validate_servoj_lookahead_time(servoj_lookahead_time)
    if isinstance(servoj_gain, bool) or not isinstance(servoj_gain, int):
        raise TypeError("servoj_gain must be an integer")
    if not 100 <= servoj_gain <= 2000:
        raise ValueError("servoj_gain must be in range [100, 2000]")
    begin_replace = (
        "" if tool_comm_setup is None else tool_comm_setup.to_urscript(robot_software_version)
    )
    return {
        "BEGIN_REPLACE": begin_replace,
        "JOINT_STATE_REPLACE": str(MULT_JOINTSTATE),
        "TIME_REPLACE": str(MULT_TIME),
        "SERVO_J_REPLACE": (
            f"lookahead_time={lookahead_time:g}, gain={servoj_gain}"
        ),
        "SERVER_IP_REPLACE": server_ip,
        "SERVER_PORT_REPLACE": str(reverse_port),
        "TRAJECTORY_SERVER_PORT_REPLACE": str(trajectory_port),
        "SCRIPT_COMMAND_SERVER_PORT_REPLACE": str(script_command_port),
        "ROBOT_SOFTWARE_VERSION": robot_software_version,
    }
