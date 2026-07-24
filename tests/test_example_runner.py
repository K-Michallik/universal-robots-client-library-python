from __future__ import annotations

import subprocess

import pytest
import run_examples
from examples._common import EXAMPLE_SKIPPED
from examples._manifest import ExampleSpec


def test_bulk_runner_requires_explicit_simulator_acknowledgement() -> None:
    with pytest.raises(SystemExit) as exc_info:
        run_examples.parse_args([])
    assert exc_info.value.code == 2


def test_bulk_runner_rejects_unbounded_duration() -> None:
    with pytest.raises(SystemExit) as exc_info:
        run_examples.parse_args(["--simulator", "--duration", "0"])
    assert exc_info.value.code == 2


def test_runner_reports_pass_and_skip(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    specs = (
        ExampleSpec("examples.one", "one.cpp", "one"),
        ExampleSpec("examples.two", "two.cpp", "two"),
    )
    return_codes = iter((0, EXAMPLE_SKIPPED))
    commands: list[list[str]] = []

    def fake_run(command: list[str], timeout: float) -> int:
        assert timeout == 120.0
        commands.append(command)
        return next(return_codes)

    monkeypatch.setattr(run_examples, "EXAMPLES", specs)
    monkeypatch.setattr(run_examples, "MODULES", tuple(spec.module for spec in specs))
    monkeypatch.setattr(run_examples, "_verify_ursim_target", lambda _ip: True)
    monkeypatch.setattr(run_examples, "_run_process", fake_run)
    monkeypatch.setattr(run_examples.time, "sleep", lambda _seconds: None)

    result = run_examples.main(
        [
            "--simulator",
            "--robot-ip",
            "192.0.2.1",
            "--duration",
            "0.25",
            "--settle-time",
            "0",
        ]
    )

    assert result == 0
    assert len(commands) == 2
    assert all("--simulator" in command and "--yes" in command for command in commands)
    assert "Example summary: 1 passed, 1 skipped" in capsys.readouterr().out


def test_runner_stops_on_first_failure(monkeypatch: pytest.MonkeyPatch) -> None:
    specs = (
        ExampleSpec("examples.one", "one.cpp", "one"),
        ExampleSpec("examples.two", "two.cpp", "two"),
    )
    calls = 0

    def fake_run(command: list[str], timeout: float) -> int:
        nonlocal calls
        del command, timeout
        calls += 1
        return 9

    monkeypatch.setattr(run_examples, "EXAMPLES", specs)
    monkeypatch.setattr(run_examples, "MODULES", tuple(spec.module for spec in specs))
    monkeypatch.setattr(run_examples, "_verify_ursim_target", lambda _ip: True)
    monkeypatch.setattr(run_examples, "_run_process", fake_run)

    result = run_examples.main(["--simulator", "--duration", "1"])

    assert result == 9
    assert calls == 1


def test_runner_rejects_unverified_target(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(run_examples, "_verify_ursim_target", lambda _ip: False)

    result = run_examples.main(["--simulator", "--duration", "1"])

    assert result == 2


def test_docker_verification_requires_official_ursim_image_and_matching_ip(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    responses = iter(
        (
            subprocess.CompletedProcess(("docker", "ps"), 0, stdout="abc123\n"),
            subprocess.CompletedProcess(
                ("docker", "inspect"),
                0,
                stdout="universalrobots/ursim_e-series:5.9.4|192.168.56.101\n",
            ),
        )
    )

    def fake_run(
        command: tuple[str, ...],
        *,
        check: bool,
        capture_output: bool,
        text: bool,
    ) -> subprocess.CompletedProcess[str]:
        del command, check, capture_output, text
        return next(responses)

    monkeypatch.setattr(run_examples.subprocess, "run", fake_run)

    assert run_examples._verify_ursim_target("192.168.56.101")
