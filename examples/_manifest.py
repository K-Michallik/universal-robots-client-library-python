"""Manifest for the 17 CMake-built upstream examples."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class ExampleSpec:
    """A runnable Python example and its upstream source."""

    module: str
    source: str
    description: str
    duration: float = 1.0


EXAMPLES = (
    # Run force mode before any example can leave the robot in a singular
    # configuration for the upstream force-mode task frame.
    ExampleSpec(
        "examples.force_mode_example",
        "force_mode_example.cpp",
        "Force mode",
        duration=2.0,
    ),
    ExampleSpec(
        "examples.full_driver",
        "full_driver.cpp",
        "UrDriver joint-servo loop",
        duration=20.0,
    ),
    ExampleSpec(
        "examples.primary_pipeline",
        "primary_pipeline.cpp",
        "Primary-interface producer/consumer pipeline",
    ),
    ExampleSpec(
        "examples.send_script",
        "send_script.cpp",
        "Primary-interface script sending",
        duration=15.0,
    ),
    ExampleSpec(
        "examples.primary_pipeline_calibration",
        "primary_pipeline_calibration.cpp",
        "Primary-interface calibration status",
        duration=3.0,
    ),
    ExampleSpec(
        "examples.rtde_client",
        "rtde_client.cpp",
        "Standalone RTDE client",
        duration=3.0,
    ),
    ExampleSpec(
        "examples.spline_example",
        "spline_example.cpp",
        "Cubic and quintic splines",
        duration=35.0,
    ),
    ExampleSpec(
        "examples.tool_contact_example",
        "tool_contact_example.cpp",
        "Tool-contact detection",
        duration=2.0,
    ),
    ExampleSpec("examples.freedrive_example", "freedrive_example.cpp", "Freedrive control"),
    ExampleSpec("examples.script_sender", "script_sender.cpp", "Standalone script sender"),
    ExampleSpec(
        "examples.script_command_interface",
        "script_command_interface.cpp",
        "Script-command interface",
        duration=28.0,
    ),
    ExampleSpec(
        "examples.direct_torque_control",
        "direct_torque_control.cpp",
        "Direct torque control",
        duration=10.0,
    ),
    ExampleSpec(
        "examples.trajectory_point_interface",
        "trajectory_point_interface.cpp",
        "Trajectory point interface",
        duration=60.0,
    ),
    ExampleSpec(
        "examples.trajectory_streaming",
        "trajectory_streaming.cpp",
        "Trajectory streaming",
        duration=8.0,
    ),
    ExampleSpec(
        "examples.instruction_executor",
        "instruction_executor.cpp",
        "High-level instruction executor",
        duration=120.0,
    ),
    ExampleSpec(
        "examples.external_fts_through_rtde",
        "external_fts_through_rtde.cpp",
        "External force/torque RTDE input",
        duration=2.0,
    ),
    # Dashboard power cycles can leave a simulator powered off when a
    # controller-specific transition is unavailable, so isolate it last.
    ExampleSpec(
        "examples.dashboard_example",
        "dashboard_example.cpp",
        "Dashboard client",
        duration=10.0,
    ),
)

MODULES = tuple(example.module for example in EXAMPLES)
