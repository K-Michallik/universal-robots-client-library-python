# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright 2023 Universal Robots A/S
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

"""Execute the upstream cubic and quintic wrist-joint splines."""

from __future__ import annotations

import argparse
import logging
import threading
import time
from collections.abc import Sequence
from contextlib import suppress

from examples._common import (
    as_six_vector,
    confirm_start,
    create_parser,
    recipe_path,
    run_cli,
    safe_stop_control,
)
from ur_client_library import ExampleRobotWrapper, UrDriver
from ur_client_library.control.reverse_interface import TrajectoryControlMessage
from ur_client_library.control.trajectory_point_interface import TrajectoryResult

_LOGGER = logging.getLogger(__name__)
_SPLINE_POSITIONS = (4.15364583e-03, 4.15364583e-03, -1.74088542e-02, 1.44817708e-02, 0.0)
_SPLINE_VELOCITIES = (
    -2.01015625e-01,
    4.82031250e-02,
    1.72812500e-01,
    -3.49453125e-01,
    8.50000000e-02,
)
_SPLINE_ACCELERATIONS = (
    2.55885417e00,
    -4.97395833e-01,
    1.71276042e00,
    -5.36458333e-02,
    -2.69817708e00,
)
_SPLINE_TIMES = (1.0, 4.0, 8.001, 12.5, 4.0)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse command-line arguments."""
    return create_parser("Run cubic and quintic splines on wrist joint 3.").parse_args(argv)


def run(args: argparse.Namespace) -> None:
    """Connect to the robot and execute both spline variants."""
    with ExampleRobotWrapper(
        robot_ip=args.robot_ip,
        output_recipe=recipe_path("rtde_output_recipe.txt"),
        input_recipe=recipe_path("rtde_input_recipe.txt"),
        headless_mode=True,
        autostart_program="external_control.urp",
    ) as robot:
        driver = robot.driver
        trajectory_done = threading.Event()
        latest_result: list[TrajectoryResult] = []

        def handle_trajectory_result(result: TrajectoryResult) -> None:
            latest_result[:] = [result]
            print(f"Trajectory report: {result.name.lower()}")
            trajectory_done.set()

        unsubscribe = None
        try:
            unsubscribe = driver.subscribe_trajectory_done(handle_trajectory_result)
            deadline = time.monotonic() + args.duration if args.duration else None

            # Background consumption keeps the RTDE socket drained while
            # bounded polling lets --duration stop the example promptly.
            robot.start_rtde_communication(consume_data=True)
            joint_positions = _wait_for_joint_positions(driver, deadline)
            if joint_positions is None:
                _LOGGER.info("Duration reached before the first RTDE sample")
                return

            driver.write_trajectory_control_message(TrajectoryControlMessage.NOOP)
            positions, velocities, accelerations = _build_spline_points(joint_positions)

            completed = _send_and_wait(
                driver,
                positions,
                velocities,
                None,
                trajectory_done,
                latest_result,
                deadline,
            )
            if not completed:
                return
            _LOGGER.info("Cubic movement done")

            completed = _send_and_wait(
                driver,
                positions,
                velocities,
                accelerations,
                trajectory_done,
                latest_result,
                deadline,
            )
            if completed:
                _LOGGER.info("Quintic movement done")
                driver.write_trajectory_control_message(TrajectoryControlMessage.NOOP)
        finally:
            if unsubscribe is not None:
                with suppress(Exception):
                    unsubscribe()
            safe_stop_control(driver)


def _wait_for_joint_positions(
    driver: UrDriver, deadline: float | None
) -> tuple[float, float, float, float, float, float] | None:
    while deadline is None or time.monotonic() < deadline:
        package = driver.get_data_package(timeout=_rtde_timeout(deadline))
        if package is not None:
            return as_six_vector(package["actual_q"], "actual_q")
        _LOGGER.warning("Could not get a fresh RTDE data package")
    return None


def _build_spline_points(
    joint_positions: Sequence[float],
) -> tuple[
    list[tuple[float, ...]],
    list[tuple[float, ...]],
    list[tuple[float, ...]],
]:
    positions: list[tuple[float, ...]] = []
    velocities: list[tuple[float, ...]] = []
    accelerations: list[tuple[float, ...]] = []
    for position, velocity, acceleration in zip(
        _SPLINE_POSITIONS,
        _SPLINE_VELOCITIES,
        _SPLINE_ACCELERATIONS,
        strict=True,
    ):
        point = list(joint_positions)
        point[5] = position
        positions.append(tuple(point))

        velocity_point = [0.0] * 6
        velocity_point[5] = velocity
        velocities.append(tuple(velocity_point))

        acceleration_point = [0.0] * 6
        acceleration_point[5] = acceleration
        accelerations.append(tuple(acceleration_point))
    return positions, velocities, accelerations


def _send_and_wait(
    driver: UrDriver,
    positions: Sequence[Sequence[float]],
    velocities: Sequence[Sequence[float]],
    accelerations: Sequence[Sequence[float]] | None,
    trajectory_done: threading.Event,
    latest_result: list[TrajectoryResult],
    deadline: float | None,
) -> bool:
    if deadline is not None and time.monotonic() >= deadline:
        return False

    trajectory_done.clear()
    latest_result.clear()
    driver.write_trajectory_control_message(
        TrajectoryControlMessage.START,
        len(positions),
    )
    for index, (position, velocity, goal_time) in enumerate(
        zip(positions, velocities, _SPLINE_TIMES, strict=True)
    ):
        acceleration = None if accelerations is None else accelerations[index]
        driver.write_trajectory_spline_point(
            position,
            velocity,
            acceleration,
            goal_time=goal_time,
        )

    while not trajectory_done.is_set():
        if deadline is not None and time.monotonic() >= deadline:
            _LOGGER.info("Duration reached while a trajectory was running")
            with suppress(Exception):
                driver.write_trajectory_control_message(TrajectoryControlMessage.CANCEL)
            return False

        package = driver.get_data_package(timeout=_rtde_timeout(deadline))
        if package is None:
            _LOGGER.warning("Could not get a fresh RTDE data package")
            continue
        as_six_vector(package["actual_q"], "actual_q")
        driver.write_trajectory_control_message(TrajectoryControlMessage.NOOP)

    if latest_result and latest_result[-1] is not TrajectoryResult.SUCCESS:
        _LOGGER.warning("Trajectory finished with result %s", latest_result[-1].name.lower())
    return True


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
            "This example stops the current program, releases the brakes, and executes "
            "two wrist-joint trajectories. Keep the robot workspace clear."
        ),
        yes=args.yes,
        simulator=args.simulator,
    ):
        print("Example cancelled.")
        return 2
    return run_cli(lambda: run(args))


if __name__ == "__main__":
    raise SystemExit(main())
