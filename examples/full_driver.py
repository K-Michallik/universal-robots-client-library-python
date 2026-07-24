# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright 2020 FZI Forschungszentrum Informatik
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# -- END LICENSE BLOCK ------------------------------------------------

"""Run the upstream open-loop joint-servo example."""

from __future__ import annotations

import argparse
import logging
import time
from collections.abc import Sequence

from examples._common import (
    as_six_vector,
    confirm_start,
    create_parser,
    recipe_path,
    run_cli,
    safe_stop_control,
)
from ur_client_library import ExampleRobotWrapper
from ur_client_library.comm.control_mode import ControlMode
from ur_client_library.ur.robot_receive_timeout import RobotReceiveTimeout

_LOGGER = logging.getLogger(__name__)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse command-line arguments."""
    return create_parser(
        "Move wrist joint 3 from its current position to 2 rad and back to 0 rad."
    ).parse_args(argv)


def run(args: argparse.Namespace) -> None:
    """Connect to the robot and execute the joint-servo loop."""
    with ExampleRobotWrapper(
        robot_ip=args.robot_ip,
        output_recipe=recipe_path("rtde_output_recipe.txt"),
        input_recipe=recipe_path("rtde_input_recipe.txt"),
        headless_mode=True,
        autostart_program="external_control.urp",
    ) as robot:
        driver = robot.driver
        try:
            _LOGGER.info("Robot version: %s", driver.version)
            increment_constant = 0.002 if driver.version.major < 5 else 0.0005
            receive_timeout = RobotReceiveTimeout.from_milliseconds(100)
            joint_target: list[float] | None = None
            passed_positive_part = False
            passed_negative_part = False
            deadline = time.monotonic() + args.duration if args.duration else None

            # Background consumption keeps the RTDE socket drained while
            # bounded polling lets --duration stop the example promptly.
            robot.start_rtde_communication(consume_data=True)
            _LOGGER.info("Starting robot movement")

            while not (passed_positive_part and passed_negative_part):
                if deadline is not None and time.monotonic() >= deadline:
                    _LOGGER.info("Duration reached before the movement completed")
                    break

                package = driver.get_data_package(timeout=_rtde_timeout(deadline))
                if package is None:
                    _LOGGER.warning("Could not get a fresh RTDE data package")
                    continue

                joint_positions = as_six_vector(package["actual_q"], "actual_q")
                if joint_target is None:
                    joint_target = list(joint_positions)

                if not passed_positive_part:
                    increment = increment_constant
                    if joint_positions[5] >= 2.0:
                        passed_positive_part = True
                else:
                    increment = -increment_constant
                    if joint_positions[5] <= 0.0:
                        passed_negative_part = True

                joint_target[5] += increment
                driver.write_joint_command(
                    joint_target,
                    ControlMode.SERVOJ,
                    receive_timeout,
                )
                _LOGGER.debug("RTDE package: %r", package)

            _LOGGER.info("Movement done")
        finally:
            safe_stop_control(driver)


def _rtde_timeout(deadline: float | None) -> float:
    if deadline is None:
        return 0.1
    return max(0.0, min(0.1, deadline - time.monotonic()))


def main(argv: Sequence[str] | None = None) -> int:
    """Run the command-line example."""
    args = parse_args(argv)
    if not confirm_start(
        args.robot_ip,
        warning=(
            "This example stops the current program, releases the brakes, and commands "
            "wrist joint 3. Keep the robot workspace clear."
        ),
        yes=args.yes,
        simulator=args.simulator,
    ):
        print("Example cancelled.")
        return 2
    return run_cli(lambda: run(args))


if __name__ == "__main__":
    raise SystemExit(main())
