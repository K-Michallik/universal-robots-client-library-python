from __future__ import annotations

import importlib
from pathlib import Path

import pytest
from examples import external_fts_through_rtde, spline_example, trajectory_streaming
from examples._common import (
    EXAMPLE_SKIPPED,
    ExampleSkipped,
    confirm_start,
    recipe_path,
    run_cli,
)
from examples._manifest import EXAMPLES, MODULES

EXPECTED_SOURCES = {
    "dashboard_example.cpp",
    "direct_torque_control.cpp",
    "external_fts_through_rtde.cpp",
    "force_mode_example.cpp",
    "freedrive_example.cpp",
    "full_driver.cpp",
    "instruction_executor.cpp",
    "primary_pipeline.cpp",
    "primary_pipeline_calibration.cpp",
    "rtde_client.cpp",
    "script_command_interface.cpp",
    "script_sender.cpp",
    "send_script.cpp",
    "spline_example.cpp",
    "tool_contact_example.cpp",
    "trajectory_point_interface.cpp",
    "trajectory_streaming.cpp",
}


def test_manifest_tracks_exactly_the_official_examples() -> None:
    assert len(EXAMPLES) == 17
    assert len(set(MODULES)) == 17
    assert {example.source for example in EXAMPLES} == EXPECTED_SOURCES
    assert all("tcp_z_move" not in module for module in MODULES)
    assert EXAMPLES[0].module == "examples.force_mode_example"


@pytest.mark.parametrize("module_name", MODULES)
def test_examples_import_and_accept_uniform_runner_arguments(module_name: str) -> None:
    module = importlib.import_module(module_name)
    args = module.parse_args(
        [
            "--robot-ip",
            "192.0.2.1",
            "--duration",
            "0.25",
            "--simulator",
            "--yes",
        ]
    )

    assert args.robot_ip == "192.0.2.1"
    assert args.duration == 0.25
    assert args.simulator is True
    assert args.yes is True
    assert callable(module.run)
    assert callable(module.main)


@pytest.mark.parametrize(
    "recipe_name",
    (
        "rtde_output_recipe.txt",
        "rtde_input_recipe.txt",
        "rtde_input_only_ft.txt",
    ),
)
def test_recipe_paths_are_independent_of_working_directory(recipe_name: str) -> None:
    path = recipe_path(recipe_name)
    assert path.is_file()
    assert path.parent.name == "resources"


def test_confirmation_refuses_noninteractive_physical_execution(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class NonInteractiveInput:
        def isatty(self) -> bool:
            return False

    monkeypatch.setattr("examples._common.sys.stdin", NonInteractiveInput())
    assert not confirm_start(
        "192.0.2.1",
        warning="test warning",
        yes=False,
        simulator=False,
    )


def test_confirmation_allows_explicit_simulator_mode() -> None:
    assert confirm_start(
        "192.0.2.1",
        warning="test warning",
        yes=False,
        simulator=True,
    )


def test_run_cli_maps_skips_to_dedicated_exit_code() -> None:
    def skip() -> None:
        raise ExampleSkipped("unsupported")

    assert run_cli(skip) == EXAMPLE_SKIPPED


def test_spline_points_always_include_velocity_and_acceleration() -> None:
    positions, velocities, accelerations = spline_example._build_spline_points((0.0,) * 6)

    assert len(positions) == len(velocities) == len(accelerations) == 5
    assert all(len(point) == 6 for point in positions)
    assert any(point[5] != 0.0 for point in velocities)
    assert any(point[5] != 0.0 for point in accelerations)


def test_quintic_stream_sampling_has_stationary_endpoints() -> None:
    start = (0.0,) * 6
    finish = (1.0,) * 6

    first = trajectory_streaming.sample_quintic_hermite(start, finish, 0.0, 2.0)
    last = trajectory_streaming.sample_quintic_hermite(start, finish, 1.0, 2.0)

    assert first.positions == start
    assert first.velocities == (0.0,) * 6
    assert first.accelerations == (0.0,) * 6
    assert last.positions == finish
    assert last.velocities == (0.0,) * 6
    assert last.accelerations == (0.0,) * 6


def test_simulator_force_torque_sequence_is_deterministic() -> None:
    assert external_fts_through_rtde._simulator_wrench(0.0) == (0.0,) * 6
    assert external_fts_through_rtde._simulator_wrench(0.25) == (10.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert external_fts_through_rtde._simulator_wrench(0.5) == (-10.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def test_recipes_are_shipped_inside_examples_tree() -> None:
    examples_directory = Path(importlib.import_module("examples").__file__).parent
    assert recipe_path("rtde_output_recipe.txt").is_relative_to(examples_directory)
