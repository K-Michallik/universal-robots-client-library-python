# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright 2022 Universal Robots A/S
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

"""Move downward until the tool-contact detector reports a result."""

from __future__ import annotations

import argparse
import logging
import threading
import time
from collections.abc import Sequence
from contextlib import suppress

from examples._common import (
    ExampleSkipped,
    confirm_start,
    create_parser,
    recipe_path,
    run_cli,
    safe_stop_control,
)
from ur_client_library import ExampleRobotWrapper
from ur_client_library.comm.control_mode import ControlMode
from ur_client_library.control.script_command_interface import ToolContactResult
from ur_client_library.ur.robot_receive_timeout import RobotReceiveTimeout

_LOGGER = logging.getLogger(__name__)
_TCP_SPEED = (0.0, 0.0, -0.02, 0.0, 0.0, 0.0)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse command-line arguments."""
    return create_parser(
        "Move the TCP downward at 0.02 m/s until tool contact is detected."
    ).parse_args(argv)


def run(args: argparse.Namespace) -> None:
    """Connect to the robot and execute tool-contact detection."""
    with ExampleRobotWrapper(
        robot_ip=args.robot_ip,
        output_recipe=recipe_path("rtde_output_recipe.txt"),
        input_recipe=recipe_path("rtde_input_recipe.txt"),
        headless_mode=True,
        autostart_program="external_control.urp",
    ) as robot:
        driver = robot.driver
        result_ready = threading.Event()
        result: list[ToolContactResult] = []

        def handle_tool_contact(contact_result: ToolContactResult) -> None:
            result[:] = [contact_result]
            print(f"Tool contact result: {contact_result.name.lower()}")
            result_ready.set()

        unsubscribe = None
        tool_contact_started = False
        try:
            if driver.version.major < 5:
                raise ExampleSkipped("Tool contact requires an e-Series controller")

            unsubscribe = driver.subscribe_tool_contact_result(handle_tool_contact)
            driver.start_tool_contact()
            tool_contact_started = True
            receive_timeout = RobotReceiveTimeout.from_milliseconds(
                args.receive_timeout_ms or 100
            )
            deadline = time.monotonic() + args.duration if args.duration else None
            period = 1.0 / driver.control_frequency

            while not result_ready.is_set() and (
                deadline is None or time.monotonic() < deadline
            ):
                driver.write_joint_command(
                    _TCP_SPEED,
                    ControlMode.SPEEDL,
                    receive_timeout,
                )
                time.sleep(period)

            if result_ready.is_set():
                _LOGGER.info("Tool contact detector finished with %s", result[-1].name.lower())
            else:
                _LOGGER.info("Timed out before reaching tool contact")
        finally:
            if unsubscribe is not None:
                with suppress(Exception):
                    unsubscribe()
            if tool_contact_started:
                with suppress(Exception):
                    driver.end_tool_contact()
            safe_stop_control(driver)


def main(argv: Sequence[str] | None = None) -> int:
    """Run the command-line example."""
    args = parse_args(argv)
    if not confirm_start(
        args.robot_ip,
        warning=(
            "This example stops the current program, releases the brakes, and moves the TCP "
            "downward until contact is detected. Verify the tool, surface, and speed first."
        ),
        yes=args.yes,
        simulator=args.simulator,
    ):
        print("Example cancelled.")
        return 2
    return run_cli(lambda: run(args))


if __name__ == "__main__":
    raise SystemExit(main())
