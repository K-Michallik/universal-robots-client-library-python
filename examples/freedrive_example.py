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

"""Put the robot into freedrive mode while sending required keepalives."""

from __future__ import annotations

import argparse
import logging
from collections.abc import Sequence
from contextlib import suppress

from examples._common import (
    confirm_start,
    create_parser,
    recipe_path,
    run_cli,
    run_until,
    safe_stop_control,
)
from ur_client_library import ExampleRobotWrapper
from ur_client_library.control.reverse_interface import FreedriveControlMessage

_LOGGER = logging.getLogger(__name__)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse command-line arguments."""
    return create_parser("Enable freedrive mode while the example is running.").parse_args(argv)


def run(args: argparse.Namespace) -> None:
    """Connect to the robot and maintain freedrive mode."""
    with ExampleRobotWrapper(
        robot_ip=args.robot_ip,
        output_recipe=recipe_path("rtde_output_recipe.txt"),
        input_recipe=recipe_path("rtde_input_recipe.txt"),
        headless_mode=True,
        autostart_program="external_control.urp",
    ) as robot:
        driver = robot.driver
        freedrive_started = False
        try:
            _LOGGER.info("Starting freedrive mode")
            driver.write_freedrive_control_message(FreedriveControlMessage.START)
            freedrive_started = True
            run_until(
                args.duration,
                lambda: driver.write_freedrive_control_message(FreedriveControlMessage.NOOP),
                period=0.002,
            )
        finally:
            if freedrive_started:
                _LOGGER.info("Stopping freedrive mode")
                with suppress(Exception):
                    driver.write_freedrive_control_message(FreedriveControlMessage.STOP)
            safe_stop_control(driver)


def main(argv: Sequence[str] | None = None) -> int:
    """Run the command-line example."""
    args = parse_args(argv)
    if not confirm_start(
        args.robot_ip,
        warning=(
            "This example stops the current program, releases the brakes, and enables "
            "freedrive. Support the robot and keep clear of pinch points."
        ),
        yes=args.yes,
        simulator=args.simulator,
    ):
        print("Example cancelled.")
        return 2
    return run_cli(lambda: run(args))


if __name__ == "__main__":
    raise SystemExit(main())
