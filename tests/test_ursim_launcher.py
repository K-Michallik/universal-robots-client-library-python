from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = ROOT / "scripts" / "start_ursim.sh"


def launch_dry_run(model: str, version: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ("bash", str(LAUNCHER), "-t", "-m", model, "-v", version),
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
