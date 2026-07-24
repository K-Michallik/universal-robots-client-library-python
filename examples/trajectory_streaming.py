# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright 2026 Universal Robots A/S
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

"""Demonstrate open-ended trajectory streaming with a controlled-stop endpoint."""

from __future__ import annotations

import argparse
import threading
import time
from collections.abc import Callable, Sequence
from contextlib import ExitStack, suppress
from dataclasses import dataclass
from typing import cast

from examples._common import confirm_start, create_parser, recipe_path, run_cli, safe_stop_control
from ur_client_library import ExampleRobotWrapper
from ur_client_library.control import (
    MotionPrimitive,
    ReverseInterface,
    SplinePrimitive,
    TrajectoryPointInterface,
    TrajectoryResult,
    TrajectoryStream,
    trajectory_result_to_string,
)
from ur_client_library.ur.ur_driver import UrDriver

POSE_A = (-1.57, -1.57, 0.0, -1.57, 0.0, 0.0)
POSE_B = (-1.57, -1.20, 0.5, -1.57, 0.0, 0.0)
ZERO = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
POINT_COUNT = 200
MOTION_DURATION_S = 2.0


@dataclass(frozen=True, slots=True)
class SampledPoint:
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    accelerations: tuple[float, ...]


class _DriverTrajectoryAdapter:
    def __init__(self, driver: UrDriver) -> None:
        self._driver = driver

    def subscribe_trajectory_result(
        self, callback: Callable[[TrajectoryResult], None]
    ) -> Callable[[], None]:
        return self._driver.subscribe_trajectory_done(callback)

    def write_motion_primitive(self, primitive: MotionPrimitive) -> None:
        self._driver.write_motion_primitive(primitive)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = create_parser("Stream a quintic-Hermite trajectory without a declared point count.")
    return parser.parse_args(argv)


def sample_quintic_hermite(
    start: Sequence[float],
    end: Sequence[float],
    fraction: float,
    duration: float,
) -> SampledPoint:
    if len(start) != 6 or len(end) != 6:
        raise ValueError("start and end must each contain six values")
    if not 0.0 <= fraction <= 1.0:
        raise ValueError("fraction must be in [0, 1]")
    if duration <= 0:
        raise ValueError("duration must be positive")

    s2 = fraction * fraction
    s3 = s2 * fraction
    s4 = s3 * fraction
    s5 = s4 * fraction
    position_blend = 10.0 * s3 - 15.0 * s4 + 6.0 * s5
    velocity_blend = (30.0 * s2 - 60.0 * s3 + 30.0 * s4) / duration
    acceleration_blend = (
        60.0 * fraction - 180.0 * s2 + 120.0 * s3
    ) / (duration * duration)
    deltas = tuple(float(finish) - float(begin) for begin, finish in zip(start, end, strict=True))
    return SampledPoint(
        tuple(
            float(begin) + delta * position_blend
            for begin, delta in zip(start, deltas, strict=True)
        ),
        tuple(delta * velocity_blend for delta in deltas),
        tuple(delta * acceleration_blend for delta in deltas),
    )


def _make_stream(driver: UrDriver) -> TrajectoryStream:
    reverse = cast(ReverseInterface, driver)
    trajectory = cast(TrajectoryPointInterface, _DriverTrajectoryAdapter(driver))
    return TrajectoryStream(reverse, trajectory)


def _cancel_active_stream(stream: TrajectoryStream) -> None:
    if stream.is_streaming:
        with suppress(Exception):
            stream.cancel()


def _wait_for_result(
    stream: TrajectoryStream,
    done: threading.Event,
    results: list[TrajectoryResult],
    deadline: float | None,
) -> bool:
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
            cleanup.callback(_cancel_active_stream, stream)
            unsubscribe = driver.subscribe_trajectory_done(trajectory_done)
            cleanup.callback(unsubscribe)
            deadline = time.monotonic() + args.duration if args.duration > 0 else None

            print("Pre-positioning to pose A through a finite trajectory")
            done.clear()
            results.clear()
            stream.start(1)
            stream.write(SplinePrimitive(POSE_A, ZERO, ZERO, duration=3.0))
            if not _wait_for_result(stream, done, results, deadline):
                return

            points = tuple(
                sample_quintic_hermite(
                    POSE_A,
                    POSE_B,
                    index / POINT_COUNT,
                    MOTION_DURATION_S,
                )
                for index in range(1, POINT_COUNT + 1)
            )
            step_time = MOTION_DURATION_S / POINT_COUNT
            done.clear()
            results.clear()
            started_writing = time.monotonic()
            stream.start_stream()
            for point in points:
                if deadline is not None and time.monotonic() >= deadline:
                    stream.cancel()
                    print("Duration reached while writing trajectory points.")
                    return
                stream.write(
                    SplinePrimitive(
                        point.positions,
                        point.velocities,
                        point.accelerations,
                        duration=step_time,
                    )
                )
            sent_points = stream.end_stream()
            finished_writing = time.monotonic()
            print(
                f"Streamed {sent_points} points in "
                f"{(finished_writing - started_writing) * 1000.0:.0f} ms"
            )
            if sent_points != POINT_COUNT:
                raise RuntimeError(
                    f"TrajectoryStream reported {sent_points} points; expected {POINT_COUNT}"
                )
            _wait_for_result(stream, done, results, deadline)
    finally:
        safe_stop_control(driver)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if not confirm_start(
        args.robot_ip,
        warning=(
            "This example releases the brakes, pre-positions every joint, and streams a "
            "joint-space trajectory. Keep the entire robot workspace clear."
        ),
        yes=args.yes,
        simulator=args.simulator,
    ):
        print("Example cancelled.")
        return 2
    return run_cli(lambda: run(args))


if __name__ == "__main__":
    raise SystemExit(main())
