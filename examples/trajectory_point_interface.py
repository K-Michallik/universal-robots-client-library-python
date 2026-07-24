# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright 2024 Universal Robots A/S
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

"""Execute finite MoveJ, MoveL, and spline trajectories."""

from __future__ import annotations

import argparse
import threading
import time
from collections.abc import Callable, Sequence
from contextlib import ExitStack
from typing import cast

from examples._common import confirm_start, create_parser, recipe_path, run_cli, safe_stop_control
from ur_client_library import ExampleRobotWrapper, Pose, Q
from ur_client_library.control import (
    MotionPrimitive,
    MoveJPrimitive,
    MoveLPrimitive,
    ReverseInterface,
    SplinePrimitive,
    TrajectoryPointInterface,
    TrajectoryResult,
    TrajectoryStream,
    trajectory_result_to_string,
)
from ur_client_library.ur.ur_driver import UrDriver

ZERO = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


class _DriverTrajectoryAdapter:
    """Expose UrDriver's trajectory methods under TrajectoryStream's protocol."""

    def __init__(self, driver: UrDriver) -> None:
        self._driver = driver

    def subscribe_trajectory_result(
        self, callback: Callable[[TrajectoryResult], None]
    ) -> Callable[[], None]:
        return self._driver.subscribe_trajectory_done(callback)

    def write_motion_primitive(self, primitive: MotionPrimitive) -> None:
        self._driver.write_motion_primitive(primitive)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = create_parser("Execute the upstream finite trajectory-point demonstrations.")
    return parser.parse_args(argv)


def _make_stream(driver: UrDriver) -> TrajectoryStream:
    reverse = cast(ReverseInterface, driver)
    trajectory = cast(TrajectoryPointInterface, _DriverTrajectoryAdapter(driver))
    return TrajectoryStream(reverse, trajectory)


def _execute_trajectory(
    stream: TrajectoryStream,
    primitives: Sequence[MotionPrimitive],
    done: threading.Event,
    results: list[TrajectoryResult],
    deadline: float | None,
) -> bool:
    done.clear()
    results.clear()
    stream.start(len(primitives))
    for primitive in primitives:
        stream.write(primitive)

    while not done.is_set():
        if deadline is not None:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                stream.cancel()
                print("Duration reached; cancelling the active trajectory.")
                return False
            wait_time = min(0.1, remaining)
        else:
            wait_time = 0.1
        if not done.wait(wait_time):
            stream.keepalive()

    result = results[-1]
    print(f"Trajectory completed with result {trajectory_result_to_string(result)}")
    if result is not TrajectoryResult.SUCCESS:
        raise RuntimeError(f"Trajectory failed with result {trajectory_result_to_string(result)}")
    return True


def run(args: argparse.Namespace) -> None:
    driver: UrDriver | None = None
    stream: TrajectoryStream | None = None
    unsubscribe: Callable[[], None] | None = None
    try:
        with ExampleRobotWrapper(
            robot_ip=args.robot_ip,
            output_recipe=recipe_path("rtde_output_recipe.txt"),
            input_recipe=recipe_path("rtde_input_recipe.txt"),
            headless_mode=True,
            autostart_program="external_control.urp",
        ) as robot, ExitStack() as cleanup:
            driver = robot.driver
            cleanup.callback(safe_stop_control, driver)
            if not robot.is_healthy:
                raise RuntimeError("Robot initialization failed")

            done = threading.Event()
            results: list[TrajectoryResult] = []

            def trajectory_done(result: TrajectoryResult) -> None:
                results.append(result)
                done.set()

            stream = _make_stream(driver)
            cleanup.callback(stream.close)
            unsubscribe = driver.subscribe_trajectory_done(trajectory_done)
            cleanup.callback(unsubscribe)
            deadline = time.monotonic() + args.duration if args.duration > 0 else None

            joint_a = Q(-1.57, -1.57, 0.0, 0.0, 0.0, 0.0)
            joint_b = Q(-1.57, -1.6, 1.6, -0.7, 0.7, 0.2)
            print("Running MoveJ trajectory")
            if not _execute_trajectory(
                stream,
                (
                    MoveJPrimitive(joint_a, blend_radius=0.1, duration=5.0),
                    MoveJPrimitive(joint_b, blend_radius=0.1, duration=2.0),
                    MoveJPrimitive(
                        joint_a,
                        blend_radius=0.1,
                        acceleration=2.5,
                        velocity=2.0,
                    ),
                    MoveJPrimitive(
                        joint_b,
                        blend_radius=0.1,
                        acceleration=2.5,
                        velocity=2.3,
                    ),
                ),
                done,
                results,
                deadline,
            ):
                return

            pose_a = Pose(-0.203, 0.263, 0.559, 0.68, -1.083, -2.076)
            pose_b = Pose(-0.203, 0.463, 0.559, 0.68, -1.083, -2.076)
            print("Running MoveL trajectory")
            if not _execute_trajectory(
                stream,
                (
                    MoveLPrimitive(pose_a, duration=5.0),
                    MoveLPrimitive(pose_b, duration=5.0),
                    MoveLPrimitive(pose_a, acceleration=2.5, velocity=2.0),
                    MoveLPrimitive(pose_b, acceleration=2.5, velocity=2.3),
                ),
                done,
                results,
                deadline,
            ):
                return

            print("Running spline trajectory")
            _execute_trajectory(
                stream,
                (
                    SplinePrimitive((-1.57, -1.57, 0.0, 0.0, 0.0, 0.0), ZERO, duration=3.0),
                    SplinePrimitive(
                        (-1.57, -1.57, -1.57, 0.0, 0.0, 0.0),
                        ZERO,
                        duration=3.0,
                    ),
                    SplinePrimitive(
                        (-1.57, -1.57, 0.0, 0.0, 0.0, 0.0),
                        (0.0, 0.0, 1.5, 0.0, 0.0, 0.0),
                        duration=3.0,
                    ),
                    SplinePrimitive(
                        (-1.57, -1.6, 1.6, -0.7, 0.7, 0.2),
                        ZERO,
                        duration=3.0,
                    ),
                ),
                done,
                results,
                deadline,
            )
    finally:
        safe_stop_control(driver)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if not confirm_start(
        args.robot_ip,
        warning=(
            "This example releases the brakes and executes joint, Cartesian, and spline "
            "trajectories through several poses. Verify every target and keep the workspace clear."
        ),
        yes=args.yes,
        simulator=args.simulator,
    ):
        print("Example cancelled.")
        return 2
    return run_cli(lambda: run(args))


if __name__ == "__main__":
    raise SystemExit(main())
