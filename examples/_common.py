"""Shared command-line, safety, and lifecycle helpers for examples."""

from __future__ import annotations

import argparse
import logging
import sys
import time
from collections.abc import Callable, Sequence
from contextlib import suppress
from pathlib import Path
from typing import Protocol

from ur_client_library import ExampleRobotWrapper, URCLException, configure_logging

DEFAULT_ROBOT_IP = ExampleRobotWrapper.DEFAULT_ROBOT_IP
EXAMPLE_SKIPPED = 77
RESOURCE_DIR = Path(__file__).resolve().parent / "resources"


class DriverWithStop(Protocol):
    """Narrow cleanup surface shared by the motion examples."""

    def stop_control(self) -> None: ...


class ExampleSkipped(RuntimeError):
    """The controller does not support an example."""


def duration_value(value: str) -> float:
    """Parse a non-negative duration; zero means run until interrupted."""
    duration = float(value)
    if duration < 0:
        raise argparse.ArgumentTypeError("duration must be non-negative")
    return duration


def positive_int(value: str) -> int:
    """Parse a strictly positive integer."""
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return parsed


def create_parser(
    description: str,
    *,
    default_duration: float = 0.0,
    include_robot_ip: bool = True,
) -> argparse.ArgumentParser:
    """Create the uniform parser used by the official example ports."""
    parser = argparse.ArgumentParser(description=description)
    if include_robot_ip:
        parser.add_argument(
            "--robot-ip",
            default=DEFAULT_ROBOT_IP,
            help=f"robot IP address (default: {DEFAULT_ROBOT_IP})",
        )
    parser.add_argument(
        "--duration",
        type=duration_value,
        default=default_duration,
        help="maximum run time in seconds; zero means until interrupted",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="skip the interactive safety confirmation",
    )
    parser.add_argument(
        "--simulator",
        action="store_true",
        help="declare that the target is URSim and bypass physical-robot confirmation",
    )
    return parser


def recipe_path(name: str) -> Path:
    """Return an example recipe independent of the current directory."""
    path = RESOURCE_DIR / name
    if not path.is_file():
        raise FileNotFoundError(f"example recipe does not exist: {path}")
    return path


def confirm_start(
    robot_ip: str,
    *,
    warning: str,
    yes: bool,
    simulator: bool,
) -> bool:
    """Require explicit approval before an example changes robot state."""
    if yes or simulator:
        return True
    print(f"\nWARNING: {warning}\n")
    if not sys.stdin.isatty():
        print("Refusing to start without an interactive terminal, --yes, or --simulator.")
        return False
    confirmation = input(f"Type the robot IP ({robot_ip}) to continue: ").strip()
    return confirmation == robot_ip


def run_cli(action: Callable[[], None]) -> int:
    """Translate example exceptions into consistent process exit codes."""
    configure_logging(logging.INFO)
    try:
        action()
    except ExampleSkipped as exc:
        print(f"SKIPPED: {exc}")
        return EXAMPLE_SKIPPED
    except KeyboardInterrupt:
        print("\nInterrupted; closing robot connections.")
        return 130
    except (URCLException, OSError) as exc:
        print(f"URCL failure: {exc}", file=sys.stderr)
        return 1
    except Exception:
        logging.exception("Unexpected example failure")
        return 1
    return 0


def run_until(duration: float, action: Callable[[], None], *, period: float = 0.01) -> None:
    """Run an action periodically for a bounded or interruptible duration."""
    started = time.monotonic()
    while duration == 0 or time.monotonic() - started < duration:
        action()
        if period > 0:
            time.sleep(period)


def wait_for_duration(duration: float, *, period: float = 0.1) -> None:
    """Wait for a bounded or interruptible duration."""
    run_until(duration, lambda: None, period=period)


def safe_stop_control(driver: DriverWithStop | None) -> None:
    """Best-effort external-control shutdown for example cleanup."""
    if driver is not None:
        with suppress(Exception):
            driver.stop_control()


def as_six_vector(value: object, name: str) -> tuple[float, float, float, float, float, float]:
    """Validate and normalize an RTDE six-vector used by an example."""
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 6:
        raise TypeError(f"{name} must be a six-element RTDE vector")
    return (
        float(value[0]),
        float(value[1]),
        float(value[2]),
        float(value[3]),
        float(value[4]),
        float(value[5]),
    )
