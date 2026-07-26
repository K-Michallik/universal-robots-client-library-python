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

"""Exercise the high-level InstructionExecutor motion API."""

from __future__ import annotations

import argparse
import threading
import time
from collections.abc import Callable, Sequence
from contextlib import ExitStack, suppress

from examples._common import (
    ExampleSkipped,
    confirm_start,
    create_parser,
    recipe_path,
    run_cli,
    safe_stop_control,
)
from ur_client_library import (
    ExampleRobotWrapper,
    InstructionExecutor,
    Pose,
    Q,
    VersionInformation,
)
from ur_client_library.control import (
    MoveJPrimitive,
    MoveLPrimitive,
    MovePPrimitive,
    OptimoveJPrimitive,
    OptimoveLPrimitive,
)
from ur_client_library.ur.ur_driver import UrDriver

CALIBRATION_CHECKSUM = "calib_12788084448423163542"


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = create_parser("Execute the upstream high-level instruction sequence.")
    return parser.parse_args(argv)


def _supports_optimove(version: VersionInformation) -> bool:
    return (
        version.major == 5
        and version >= VersionInformation.from_string("5.21.0")
    ) or (
        version.major >= 10
        and version >= VersionInformation.from_string("10.8.0")
    )


def _run_bounded(
    executor: InstructionExecutor,
    operation: Callable[[], object],
    deadline: float | None,
) -> bool:
    if deadline is None:
        operation()
        return True

    remaining = deadline - time.monotonic()
    if remaining <= 0:
        return False

    done = threading.Event()
    errors: list[BaseException] = []

    def invoke() -> None:
        try:
            operation()
        except BaseException as exc:
            errors.append(exc)
        finally:
            done.set()

    worker = threading.Thread(target=invoke, name="instruction-example", daemon=True)
    worker.start()
    if not done.wait(remaining):
        if executor.is_trajectory_running:
            with suppress(Exception):
                executor.cancel_motion()
        worker.join(timeout=5.0)
        if worker.is_alive():
            raise RuntimeError("Timed out waiting for the cancelled instruction to stop")
        print("Duration reached; cancelling the active instruction.")
        return False
    worker.join()
    if errors:
        raise errors[0]
    return True


def run(args: argparse.Namespace) -> None:
    driver: UrDriver | None = None
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
            if not driver.check_calibration(CALIBRATION_CHECKSUM):
                print(
                    "WARNING: the robot calibration does not match the upstream example's "
                    "reference checksum. Verify all targets against this robot."
                )
            if not _supports_optimove(driver.version):
                raise ExampleSkipped(
                    "the translated sequence includes OptimoveJ/L and requires PolyScope "
                    f"5.21.0 / 10.8.0 or newer; robot has {driver.version}"
                )

            deadline = time.monotonic() + args.duration if args.duration > 0 else None
            with InstructionExecutor(driver) as executor:
                sequence = (
                    MoveJPrimitive(
                        Q(-1.57, -1.57, 0.0, 0.0, 0.0, 0.0),
                        blend_radius=0.1,
                        duration=5.0,
                    ),
                    MoveJPrimitive(
                        Q(-1.57, -1.6, 1.6, -0.7, 0.7, 0.2),
                        blend_radius=0.1,
                        acceleration=1.4,
                        velocity=2.0,
                    ),
                    MoveLPrimitive(
                        Pose(-0.203, 0.263, 0.559, 0.68, -1.083, -2.076),
                        blend_radius=0.1,
                        duration=2.0,
                    ),
                    MoveLPrimitive(
                        Q(-1.57, -1.6, 1.6, -0.7, 0.7, 0.2),
                        blend_radius=0.1,
                        duration=2.0,
                    ),
                    MovePPrimitive(
                        Pose(-0.203, 0.463, 0.559, 0.68, -1.083, -2.076),
                        blend_radius=0.1,
                        acceleration=0.4,
                        velocity=0.4,
                    ),
                    OptimoveJPrimitive(
                        Q(-1.57, -1.57, 1.6, -0.5, 0.4, 0.3),
                        blend_radius=0.1,
                        acceleration_fraction=0.4,
                        velocity_fraction=0.7,
                    ),
                    OptimoveLPrimitive(
                        Pose(-0.203, 0.263, 0.559, 0.68, -1.083, -2.076),
                        blend_radius=0.1,
                        acceleration_fraction=0.4,
                        velocity_fraction=0.7,
                    ),
                )
                if not _run_bounded(
                    executor,
                    lambda: executor.execute_motion(sequence),
                    deadline,
                ):
                    return

                target_pose = Pose(
                    -0.206,
                    -0.6437,
                    0.202,
                    0.0,
                    3.140,
                    0.0,
                    q_near=Q(-1.7, -4.0, 1.5, -1.0, 1.5, 0.0),
                )
                goal_time = 2.0
                operations: tuple[tuple[str, Callable[[], object]], ...] = (
                    (
                        "MoveJ joint target",
                        lambda: executor.move_j(
                            Q(-1.742, -1.726, -2.214, -0.773, 1.572, -0.171),
                            2.0,
                            2.0,
                        ),
                    ),
                    (
                        "MoveJ pose target",
                        lambda: executor.move_j(
                            Pose(-0.206, -0.6437, 0.202, 0.0, 3.140, 0.0),
                            2.0,
                            2.0,
                        ),
                    ),
                    ("MoveJ pose with q_near", lambda: executor.move_j(target_pose, 2.0, 2.0)),
                    ("MoveJ inline q_near", lambda: executor.move_j(target_pose, 2.0, 2.0)),
                    (
                        "MoveJ goal time",
                        lambda: executor.move_j(
                            Q(-1.57, -1.6, 1.6, -0.7, 0.7, 0.2),
                            0.1,
                            0.1,
                            goal_time,
                        ),
                    ),
                    (
                        "MoveL inferred pose",
                        lambda: executor.move_l(
                            (-0.0203, 0.363, 0.559, 0.68, -1.083, -2.076),
                            1.5,
                            1.5,
                        ),
                    ),
                    (
                        "MoveL pose",
                        lambda: executor.move_l(
                            Pose(-0.0203, 0.363, 0.559, 0.68, -1.083, -2.076),
                            1.5,
                            1.5,
                        ),
                    ),
                    (
                        "MoveL joint target",
                        lambda: executor.move_l(
                            Q(-1.572, -1.686, 1.707, -0.833, 0.782, 0.479),
                            1.5,
                            1.5,
                            goal_time,
                        ),
                    ),
                    (
                        "MoveJ Cartesian target",
                        lambda: executor.move_j(
                            Pose(-0.0203, 0.363, 0.559, 0.68, -1.083, -2.076),
                            0.1,
                            0.1,
                            goal_time,
                        ),
                    ),
                    (
                        "MoveP inferred pose",
                        lambda: executor.move_p(
                            (-0.2, 0.363, 0.559, 0.68, -1.083, -2.076),
                            0.2,
                            0.2,
                        ),
                    ),
                    (
                        "MoveP pose",
                        lambda: executor.move_p(
                            Pose(-0.0203, 0.303, 0.559, 0.68, -1.083, -2.076),
                            0.2,
                            0.2,
                        ),
                    ),
                    (
                        "MoveP joint target",
                        lambda: executor.move_p(
                            Q(-1.57, -1.83, 1.707, -0.833, 0.782, 0.479),
                            0.2,
                            0.2,
                        ),
                    ),
                    (
                        "MoveP circular pre-position",
                        lambda: executor.move_p(
                            Pose(-0.2, 0.5, 0.5, 0.68, -1.083, -2.076),
                            0.2,
                            0.2,
                        ),
                    ),
                    (
                        "MoveC pose target",
                        lambda: executor.move_c(
                            Pose(-0.2, 0.4, 0.4, 0.68, -1.083, -2.076),
                            Pose(-0.2, 0.4, 0.6, 0.68, -1.083, -2.076),
                        ),
                    ),
                    (
                        "MoveC joint target",
                        lambda: executor.move_c(
                            Pose(-0.2, 0.5, 0.5, 0.68, -1.083, -2.076),
                            Q(-1.84, -1.8, 2.7, 4.2, 0.6, -4.0),
                        ),
                    ),
                    (
                        "OptimoveJ joint target",
                        lambda: executor.optimove_j(
                            Q(-1.57, -1.6, 1.6, -0.7, 0.7, 0.2),
                            1.0,
                            1.0,
                        ),
                    ),
                    (
                        "OptimoveL pose target",
                        lambda: executor.optimove_l(
                            Pose(-0.0203, 0.363, 0.559, 0.68, -1.083, -2.076),
                            1.0,
                            1.0,
                        ),
                    ),
                    (
                        "OptimoveL joint target",
                        lambda: executor.optimove_l(
                            Q(-1.572, -1.686, 1.707, -0.833, 0.782, 0.479),
                            1.0,
                            1.0,
                        ),
                    ),
                    (
                        "OptimoveJ pose target",
                        lambda: executor.optimove_j(
                            Pose(-0.0203, 0.363, 0.559, 0.68, -1.083, -2.076),
                            1.0,
                            1.0,
                        ),
                    ),
                )
                for description, operation in operations:
                    print(f"Running {description}")
                    if not _run_bounded(executor, operation, deadline):
                        return
    finally:
        safe_stop_control(driver)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if not confirm_start(
        args.robot_ip,
        warning=(
            "This example releases the brakes and executes a long sequence of joint, Cartesian, "
            "circular, and Optimove commands. Verify every target and keep the workspace clear."
        ),
        yes=args.yes,
        simulator=args.simulator,
    ):
        print("Example cancelled.")
        return 2
    return run_cli(lambda: run(args))


if __name__ == "__main__":
    raise SystemExit(main())
