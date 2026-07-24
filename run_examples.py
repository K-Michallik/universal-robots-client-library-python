#!/usr/bin/env python3
"""Run the official Python examples sequentially against URSim."""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from collections.abc import Sequence
from pathlib import Path

from examples._common import DEFAULT_ROBOT_IP, EXAMPLE_SKIPPED, duration_value
from examples._manifest import EXAMPLES, MODULES

ROOT = Path(__file__).resolve().parent


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run every official Python example against a simulator."
    )
    parser.add_argument(
        "--robot-ip",
        default=DEFAULT_ROBOT_IP,
        help=f"URSim IP address (default: {DEFAULT_ROBOT_IP})",
    )
    parser.add_argument(
        "--duration",
        type=duration_value,
        default=None,
        help="override every manifest duration with this positive number of seconds",
    )
    parser.add_argument(
        "--process-timeout",
        type=duration_value,
        default=None,
        help="hard wall timeout per process; defaults to duration plus a startup allowance",
    )
    parser.add_argument(
        "--settle-time",
        type=duration_value,
        default=1.0,
        help="delay between examples in seconds (default: 1)",
    )
    parser.add_argument(
        "--only",
        action="append",
        choices=MODULES,
        help="run only this module; may be supplied more than once",
    )
    parser.add_argument(
        "--simulator",
        action="store_true",
        help="required acknowledgement that the target is URSim",
    )
    parser.add_argument(
        "--allow-unverified-simulator",
        action="store_true",
        help="allow a remote URSim IP that cannot be verified through the local Docker engine",
    )
    args = parser.parse_args(argv)
    if not args.simulator:
        parser.error("bulk example execution is restricted to URSim; pass --simulator")
    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be greater than zero for bulk execution")
    if args.process_timeout is not None and args.process_timeout <= 0:
        parser.error("--process-timeout must be greater than zero")
    return args


def _environment() -> dict[str, str]:
    environment = os.environ.copy()
    src = str(ROOT / "src")
    existing = environment.get("PYTHONPATH")
    environment["PYTHONPATH"] = src if not existing else f"{src}{os.pathsep}{existing}"
    return environment


def _verify_ursim_target(robot_ip: str) -> bool:
    """Verify that Docker owns the target IP through an official URSim image."""
    try:
        running = subprocess.run(
            ("docker", "ps", "-q"),
            check=True,
            capture_output=True,
            text=True,
        )
        container_ids = running.stdout.split()
        if not container_ids:
            return False
        inspected = subprocess.run(
            (
                "docker",
                "inspect",
                "--format",
                "{{.Config.Image}}|{{range .NetworkSettings.Networks}}{{.IPAddress}} {{end}}",
                *container_ids,
            ),
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.SubprocessError):
        return False

    for line in inspected.stdout.splitlines():
        image, separator, addresses = line.partition("|")
        if (
            separator
            and image.startswith("universalrobots/ursim_")
            and robot_ip in addresses.split()
        ):
            return True
    return False


def _interrupt_process(process: subprocess.Popen[bytes]) -> None:
    """Give an example time to run its cleanup before forcing termination."""
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=10.0)
        return
    except subprocess.TimeoutExpired:
        process.terminate()
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5.0)


def _run_process(command: Sequence[str], timeout: float) -> int:
    process = subprocess.Popen(
        command,
        cwd=ROOT,
        env=_environment(),
        start_new_session=True,
    )
    try:
        return process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        print(
            f"Example exceeded its {timeout:g}-second process timeout; interrupting it.",
            file=sys.stderr,
        )
        _interrupt_process(process)
        return 124


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if not args.allow_unverified_simulator and not _verify_ursim_target(args.robot_ip):
        print(
            "Refusing bulk motion: the target IP is not owned by a running local "
            "universalrobots/ursim_* Docker container. For a deliberately managed remote "
            "URSim instance, also pass --allow-unverified-simulator.",
            file=sys.stderr,
        )
        return 2

    selected = set(args.only or MODULES)
    passed = 0
    skipped = 0

    for spec in EXAMPLES:
        if spec.module not in selected:
            continue
        duration = spec.duration if args.duration is None else args.duration
        process_timeout = (
            max(120.0, duration + 60.0)
            if args.process_timeout is None
            else args.process_timeout
        )
        print(
            f"\n#### Executing {spec.module} ({spec.source}, duration {duration:g}s)",
            flush=True,
        )
        command = [
            sys.executable,
            "-m",
            spec.module,
            "--robot-ip",
            args.robot_ip,
            "--duration",
            str(duration),
            "--simulator",
            "--yes",
        ]
        return_code = _run_process(command, process_timeout)
        if return_code == EXAMPLE_SKIPPED:
            skipped += 1
            print(f"#### Skipped {spec.module}", flush=True)
        elif return_code != 0:
            print(
                f"Execution of {spec.module} failed with exit status {return_code}.",
                file=sys.stderr,
            )
            return return_code
        else:
            passed += 1
            print(f"#### Passed {spec.module}", flush=True)
        if args.settle_time:
            time.sleep(args.settle_time)

    print(f"\nExample summary: {passed} passed, {skipped} skipped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
