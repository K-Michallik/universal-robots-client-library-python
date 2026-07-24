# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright 2025 Universal Robots A/S
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# -- END LICENSE BLOCK ------------------------------------------------

"""Send an artificial external force/torque vector through RTDE."""

from __future__ import annotations

import argparse
import select
import sys
import termios
import time
import tty
from collections.abc import Iterator, Sequence
from contextlib import ExitStack, contextmanager, suppress

from examples._common import (
    ExampleSkipped,
    as_six_vector,
    confirm_start,
    create_parser,
    recipe_path,
    run_cli,
    safe_stop_control,
)
from ur_client_library import ExampleRobotWrapper, VersionInformation
from ur_client_library.rtde import RTDEWriter
from ur_client_library.ur.ur_driver import UrDriver

ZERO_WRENCH = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
SIMULATOR_DEFAULT_DURATION = 2.0


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = create_parser("Inject an artificial force/torque sensor vector through RTDE.")
    return parser.parse_args(argv)


def _supports_external_ft(version: VersionInformation) -> bool:
    if version.major >= 5:
        return True
    return version.major == 3 and version >= VersionInformation.from_string("3.14.1")


def _poll_key() -> str | None:
    if not sys.stdin.isatty():
        return None
    readable, _, _ = select.select((sys.stdin,), (), (), 0.0)
    if not readable:
        return None
    return sys.stdin.read(1)


@contextmanager
def _single_key_input(enabled: bool) -> Iterator[None]:
    if not enabled or not sys.stdin.isatty():
        yield
        return
    descriptor = sys.stdin.fileno()
    previous = termios.tcgetattr(descriptor)
    tty.setcbreak(descriptor)
    try:
        yield
    finally:
        termios.tcsetattr(descriptor, termios.TCSADRAIN, previous)


def _apply_key(wrench: list[float], key: str) -> bool:
    axes = {"x": 0, "y": 1, "z": 2, "a": 3, "b": 4, "c": 5}
    lower = key.lower()
    if lower in axes:
        direction = 1.0 if key.islower() else -1.0
        wrench[axes[lower]] += direction * 10.0
    elif key == "0":
        wrench[:] = ZERO_WRENCH
    return key != "q"


def _simulator_wrench(elapsed: float) -> tuple[float, ...]:
    """Produce a repeatable pulse on each axis without reading stdin."""
    phase = int(elapsed / 0.25) % 13
    if phase == 0:
        return ZERO_WRENCH
    axis = (phase - 1) // 2
    sign = 1.0 if phase % 2 else -1.0
    return tuple(sign * 10.0 if index == axis else 0.0 for index in range(6))


def _send_zero_wrench(writer: RTDEWriter) -> None:
    with suppress(Exception):
        writer.send_external_force_torque(ZERO_WRENCH)
        time.sleep(0.05)


def _disable_ft_input(driver: UrDriver) -> None:
    with suppress(Exception):
        driver.ft_rtde_input_enable(False)


def run(args: argparse.Namespace) -> None:
    driver: UrDriver | None = None
    try:
        with ExampleRobotWrapper(
            robot_ip=args.robot_ip,
            output_recipe=recipe_path("rtde_output_recipe.txt"),
            input_recipe=recipe_path("rtde_input_only_ft.txt"),
            headless_mode=True,
            autostart_program="external_control.urp",
        ) as robot, ExitStack() as cleanup:
            driver = robot.driver
            cleanup.callback(safe_stop_control, driver)
            if not robot.is_healthy:
                raise RuntimeError("Robot initialization failed")
            if not _supports_external_ft(driver.version):
                raise ExampleSkipped(
                    "external force/torque input requires PolyScope 3.14.1 or newer; "
                    f"robot has {driver.version}"
                )

            driver.ft_rtde_input_enable(True)
            cleanup.callback(_disable_ft_input, driver)
            robot.start_rtde_communication(consume_data=True)
            writer = driver.rtde_writer
            cleanup.callback(_send_zero_wrench, writer)

            duration = args.duration
            if args.simulator and duration == 0:
                duration = SIMULATOR_DEFAULT_DURATION
                print(
                    f"Simulator mode selected without --duration; running {duration:.0f} seconds."
                )
            wrench = [0.0] * 6
            running = True
            started = time.monotonic()
            last_report = started - 1.0
            if not args.simulator:
                print(
                    "Press x/y/z or a/b/c to add +10 on an axis; use uppercase for -10, "
                    "0 to reset, and q to quit."
                )

            with _single_key_input(not args.simulator):
                while running:
                    elapsed = time.monotonic() - started
                    if duration > 0 and elapsed >= duration:
                        break
                    package = driver.get_data_package(timeout=0.2)
                    if package is None:
                        continue

                    if args.simulator:
                        outgoing = _simulator_wrench(elapsed)
                    else:
                        key = _poll_key()
                        if key is not None:
                            running = _apply_key(wrench, key)
                            print(f"Artificial FT input: {tuple(wrench)}")
                        outgoing = tuple(wrench)
                    if running:
                        writer.send_external_force_torque(outgoing)

                    now = time.monotonic()
                    if now - last_report >= 1.0:
                        measured = as_six_vector(package["actual_TCP_force"], "actual_TCP_force")
                        print(f"Force-torque reported by robot: {measured}")
                        last_report = now
    finally:
        safe_stop_control(driver)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if not confirm_start(
        args.robot_ip,
        warning=(
            "This example releases the brakes and replaces the controller's force/torque sensor "
            "input with RTDE data. Incorrect values can affect force-sensitive robot behavior."
        ),
        yes=args.yes,
        simulator=args.simulator,
    ):
        print("Example cancelled.")
        return 2
    return run_cli(lambda: run(args))


if __name__ == "__main__":
    raise SystemExit(main())
