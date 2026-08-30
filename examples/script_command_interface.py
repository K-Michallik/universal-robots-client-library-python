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

"""Exercise one-shot commands while maintaining external-control keepalives."""

from __future__ import annotations

import argparse
import logging
import threading
import time
from collections.abc import Callable, Sequence
from contextlib import suppress

from examples._common import (
    ExampleSkipped,
    as_six_vector,
    confirm_start,
    create_parser,
    recipe_path,
    run_cli,
    run_until,
    safe_stop_control,
)
from ur_client_library import ExampleRobotWrapper, LifecycleError, UrDriver, VersionInformation

_LOGGER = logging.getLogger(__name__)
_COMMAND_DELAY = 3.0
_MINIMUM_FRICTION_SCALE_VERSION_G5 = VersionInformation.from_string("5.25.1")
_MINIMUM_FRICTION_SCALE_VERSION_X = VersionInformation.from_string("10.12.1")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = create_parser("Exercise the external-control script-command interface.")
    parser.add_argument(
        "--headless",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="send the external-control program headlessly (default: enabled)",
    )
    return parser.parse_args(argv)


def _drain_script_commands(driver: UrDriver, seconds: float = 1.0) -> None:
    """Keep the external-control program running long enough to apply pending commands."""
    period = max(0.001, 1.0 / driver.control_frequency)
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        driver.write_keepalive()
        time.sleep(period)


def _read_tcp_offset(
    driver: UrDriver,
    attempts: int = 15,
) -> tuple[float, float, float, float, float, float] | None:
    """Return the controller's current TCP offset, or None if it stays unavailable."""
    # This example does not run the RTDE background reader, so read the
    # package directly; fall back to the background accessor for callers that
    # do have it running.
    for _ in range(attempts):
        with suppress(Exception):
            return as_six_vector(driver.get_data_package_blocking()["tcp_offset"], "tcp_offset")
        with suppress(Exception):
            package = driver.get_data_package(timeout=1.0)
            if package is not None:
                return as_six_vector(package["tcp_offset"], "tcp_offset")
        # Neither reader is available until the driver finishes bringing RTDE
        # up, which happens shortly after the program starts.
        time.sleep(0.2)
    return None


def run(args: argparse.Namespace) -> None:
    """Connect to the robot and cycle through script commands."""
    with ExampleRobotWrapper(
        robot_ip=args.robot_ip,
        output_recipe=recipe_path("rtde_output_recipe.txt"),
        input_recipe=recipe_path("rtde_input_recipe.txt"),
        headless_mode=args.headless,
        autostart_program="external_control.urp",
    ) as robot:
        driver = robot.driver
        stop_requested = threading.Event()
        failures: list[Exception] = []
        worker: threading.Thread | None = None
        supports_friction_scales = False
        # This example changes the TCP offset and the payload, which are
        # controller-wide settings that outlive the program. Capture the
        # incoming TCP offset so the finally block can put it back; leaving a
        # tool offset behind shifts every later Cartesian target and can push
        # it outside the robot's workspace.
        original_tcp_offset: tuple[float, ...] | None = None
        try:
            if driver.version.major < 5:
                raise ExampleSkipped("This script-command sequence requires an e-Series controller")

            supports_friction_scales = _supports_friction_scales(driver.version)
            if not supports_friction_scales:
                _LOGGER.info(
                    "Friction scales are unavailable on PolyScope %s; those commands are skipped",
                    driver.version,
                )

            # Needed only to read back the TCP offset this example overwrites.
            robot.start_rtde_communication(consume_data=True)
            original_tcp_offset = _read_tcp_offset(driver)
            if original_tcp_offset is None:
                _LOGGER.warning(
                    "Could not read the current TCP offset; it will be reset to zero afterwards"
                )
                original_tcp_offset = (0.0,) * 6

            worker = threading.Thread(
                target=_send_script_commands,
                args=(driver, supports_friction_scales, stop_requested, failures),
                name="urcl-example-script-commands",
                daemon=True,
            )
            worker.start()

            def keepalive() -> None:
                if failures:
                    raise failures[0]
                if not robot.is_healthy:
                    raise LifecycleError("The external-control program stopped")
                driver.write_keepalive()

            run_until(
                args.duration,
                keepalive,
                period=max(0.001, 1.0 / driver.control_frequency),
            )
            if failures:
                raise failures[0]
            _LOGGER.info("Duration reached")
        finally:
            stop_requested.set()
            if worker is not None:
                worker.join()
            with suppress(Exception):
                driver.end_tool_contact()
            with suppress(Exception):
                driver.set_tool_voltage(0)
            if supports_friction_scales:
                with suppress(Exception):
                    driver.set_friction_scales((1.0,) * 6, (1.0,) * 6)
            if original_tcp_offset is not None:
                with suppress(Exception):
                    driver.set_tcp_offset(original_tcp_offset)
            with suppress(Exception):
                driver.set_payload(0.0, (0.0, 0.0, 0.0))
            # The script command interface is asynchronous: the URScript reads
            # one command per control cycle. Keep the external-control program
            # alive briefly so it consumes the restore commands above before
            # safe_stop_control tears the program down.
            with suppress(Exception):
                _drain_script_commands(driver)
            safe_stop_control(driver)
        if failures:
            raise failures[0]


def _supports_friction_scales(version: VersionInformation) -> bool:
    return (
        version.major == 5 and version >= _MINIMUM_FRICTION_SCALE_VERSION_G5
    ) or (version.major >= 10 and version >= _MINIMUM_FRICTION_SCALE_VERSION_X)


def _send_script_commands(
    driver: UrDriver,
    supports_friction_scales: bool,
    stop_requested: threading.Event,
    failures: list[Exception],
) -> None:
    try:
        commands = _script_commands(driver, supports_friction_scales)
        while not stop_requested.is_set():
            for message, command in commands:
                if stop_requested.wait(_COMMAND_DELAY):
                    return
                _LOGGER.info(message)
                command()
    except Exception as exc:
        failures.append(exc)
        stop_requested.set()
    finally:
        _LOGGER.info("Script command thread finished")


def _script_commands(
    driver: UrDriver,
    supports_friction_scales: bool,
) -> list[tuple[str, Callable[[], None]]]:
    commands: list[tuple[str, Callable[[], None]]] = [
        ("Setting tool voltage to 24 V", lambda: driver.set_tool_voltage(24)),
        ("Enabling tool contact mode", driver.start_tool_contact),
    ]
    if supports_friction_scales:
        commands.append(
            (
                "Setting friction scales to zero",
                lambda: driver.set_friction_scales((0.0,) * 6, (0.0,) * 6),
            )
        )
    commands.extend(
        [
            ("Setting tool voltage to 0 V", lambda: driver.set_tool_voltage(0)),
            ("Zeroing the force/torque sensor", driver.zero_ft_sensor),
            ("Disabling tool contact mode", driver.end_tool_contact),
            (
                "Setting TCP offset to [0, 0, 0.1, 0, 0, 0]",
                lambda: driver.set_tcp_offset((0.0, 0.0, 0.1, 0.0, 0.0, 0.0)),
            ),
            (
                "Setting a 2 kg target payload",
                lambda: driver.set_target_payload(
                    2.0,
                    (0.0, 0.0, 0.1),
                    (0.003, 0.003, 0.003, 0.0, 0.0, 0.0),
                    0.5,
                ),
            ),
        ]
    )
    if supports_friction_scales:
        commands.append(
            (
                "Setting nonzero friction scales",
                lambda: driver.set_friction_scales(
                    (0.9, 0.9, 0.8, 0.9, 0.9, 0.9),
                    (0.8, 0.8, 0.7, 0.8, 0.8, 0.8),
                ),
            )
        )
    return commands


def main(argv: Sequence[str] | None = None) -> int:
    """Run the command-line example."""
    args = parse_args(argv)
    if not confirm_start(
        args.robot_ip,
        warning=(
            "This example stops the current program, releases the brakes, and repeatedly "
            "changes tool voltage, contact mode, TCP, payload, and friction settings."
        ),
        yes=args.yes,
        simulator=args.simulator,
    ):
        print("Example cancelled.")
        return 2
    return run_cli(lambda: run(args))


if __name__ == "__main__":
    raise SystemExit(main())
