from __future__ import annotations

import shlex
import subprocess
from itertools import pairwise
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = ROOT / "scripts" / "start_ursim.sh"
CITADEL_FIXTURE = "tests/resources/dockerursim/citadelDB/10.14.0"


def launch_dry_run(
    model: str,
    version: str,
    *arguments: str,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ("bash", str(LAUNCHER), "-t", "-m", model, "-v", version, *arguments),
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )


@pytest.mark.parametrize(
    ("model", "version", "expected"),
    (
        ("ur5e", "5.9.4", ("ursim_e-series:5.9.4", "ROBOT_MODEL=UR5")),
        (
            "ur5e",
            "10.7.0",
            ("ursim_polyscopex:10.7.0", "ROBOT_TYPE=UR5", "ROBOT_TYPE_CONTROLLER=UR5"),
        ),
        ("ur7e", "10.13.0", ("ursim_polyscopex:10.13.0", "ROBOT_TYPE=UR7e")),
    ),
)
def test_launcher_normalizes_controller_models(
    model: str,
    version: str,
    expected: tuple[str, ...],
) -> None:
    result = launch_dry_run(model, version)

    assert result.returncode == 0, result.stderr
    assert all(fragment in result.stdout for fragment in expected)


def test_launcher_rejects_incompatible_model_and_series() -> None:
    result = launch_dry_run("ur5", "10.13.0")

    assert result.returncode == 2
    assert "requires an e-Series model name" in result.stderr


def _volume_mounts(command_line: str) -> dict[str, str]:
    arguments = shlex.split(command_line)
    mounts: dict[str, str] = {}
    for flag, value in pairwise(arguments):
        if flag == "-v":
            source, _, target = value.rpartition(":")
            mounts[target] = source
    return mounts


def test_launcher_mounts_citadel_database_for_polyscope_x(tmp_path: Path) -> None:
    citadel = tmp_path / "citadelDB"
    citadel.mkdir()

    result = launch_dry_run("ur7e", "10.14.0", "-c", str(citadel))

    assert result.returncode == 0, result.stderr
    assert _volume_mounts(result.stdout)["/citadelDB"] == str(citadel.resolve())


def test_launcher_resolves_relative_citadel_database_folder() -> None:
    result = launch_dry_run("ur7e", "10.14.0", "-c", CITADEL_FIXTURE)

    assert result.returncode == 0, result.stderr
    assert _volume_mounts(result.stdout)["/citadelDB"] == str((ROOT / CITADEL_FIXTURE).resolve())


def test_launcher_omits_citadel_database_without_flag() -> None:
    result = launch_dry_run("ur7e", "10.14.0")

    assert result.returncode == 0, result.stderr
    assert "/citadelDB" not in _volume_mounts(result.stdout)


def test_launcher_rejects_missing_citadel_database_folder(tmp_path: Path) -> None:
    missing = tmp_path / "missing"

    result = launch_dry_run("ur7e", "10.14.0", "-c", str(missing))

    assert result.returncode == 2
    assert "Citadel database folder does not exist" in result.stderr
    assert result.stdout == ""
    assert not missing.exists()


def test_launcher_ignores_citadel_database_outside_polyscope_x(tmp_path: Path) -> None:
    # Upstream accepts -c for every series and only mounts it for PolyScope X.
    result = launch_dry_run("ur5e", "5.9.4", "-c", str(tmp_path))

    assert result.returncode == 0, result.stderr
    assert "only applies to PolyScope X" in result.stderr
    assert "/citadelDB" not in _volume_mounts(result.stdout)


def test_launcher_creates_and_mounts_nested_program_folder(tmp_path: Path) -> None:
    programs = tmp_path / "polyscopex" / "10.14.0" / "ur7e"

    result = launch_dry_run("ur7e", "10.14.0", "-p", str(programs))

    assert result.returncode == 0, result.stderr
    assert programs.is_dir()
    mounts = _volume_mounts(result.stdout)
    assert mounts["/ur/bin/backend/applications"] == str(programs.resolve())


@pytest.mark.parametrize(
    ("model", "version"),
    (("ur5e", "10.12.0"), ("ur7e", "10.13.0"), ("ur7e", "10.14.0")),
)
def test_versioned_polyscope_x_fixtures_are_mounted(model: str, version: str) -> None:
    programs = f"tests/resources/dockerursim/programs/polyscopex/{version}/{model}"
    arguments = ["-p", programs]
    citadel = ROOT / "tests" / "resources" / "dockerursim" / "citadelDB" / version
    if citadel.is_dir():
        arguments += ["-c", str(citadel)]

    result = launch_dry_run(model, version, *arguments)

    assert result.returncode == 0, result.stderr
    assert (ROOT / programs / "00000000000.xd").is_file()
    mounts = _volume_mounts(result.stdout)
    assert mounts["/ur/bin/backend/applications"] == str((ROOT / programs).resolve())
    assert ("/citadelDB" in mounts) == (version != "10.12.0")
