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

"""Apply the upstream sinusoidal direct-torque demonstration."""

from __future__ import annotations

import argparse
import math
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
    safe_stop_control,
)
from ur_client_library import ExampleRobotWrapper, InstructionExecutor, Q, VersionInformation
from ur_client_library.comm import ControlMode
from ur_client_library.ur.robot_receive_timeout import RobotReceiveTimeout
from ur_client_library.ur.ur_driver import UrDriver

JOINT_INDEX = 5
FREQUENCY_HZ = 0.2
AMPLITUDE_NM = 2.5
START_POSITION_RAD = 0.0
PERIOD_COUNT = 10


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = create_parser(
        "Apply a sinusoidal torque to wrist joint 3 using direct torque control.",
        default_duration=PERIOD_COUNT / FREQUENCY_HZ,
    )
    return parser.parse_args(argv)


def _supports_friction_scales(version: VersionInformation) -> bool:
    return (
        version.major == 5
        and version >= VersionInformation.from_string("5.25.1")
    ) or (
        version.major >= 10
        and version >= VersionInformation.from_string("10.12.1")
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

    worker = threading.Thread(target=invoke, name="torque-preposition", daemon=True)
    worker.start()
    if not done.wait(remaining):
        if executor.is_trajectory_running:
            with suppress(Exception):
                executor.cancel_motion()
        worker.join(timeout=5.0)
        if worker.is_alive():
            raise RuntimeError("Timed out waiting for the cancelled pre-position motion to stop")
        print("Duration reached while pre-positioning; cancelling motion.")
        return False
    worker.join()
    if errors:
        raise errors[0]
    return True


def run(args: argparse.Namespace) -> None:
    with ExampleRobotWrapper(
        robot_ip=args.robot_ip,
        output_recipe=recipe_path("rtde_output_recipe.txt"),
        input_recipe=recipe_path("rtde_input_recipe.txt"),
        headless_mode=True,
        autostart_program="external_control.urp",
    ) as robot:
        driver: UrDriver = robot.driver
        try:
            if not robot.is_healthy:
                raise RuntimeError("Robot initialization failed")
            if not _supports_friction_scales(driver.version):
                raise ExampleSkipped(
                    "direct torque with friction scales requires PolyScope "
                    f"5.25.1 / 10.12.1 or newer; robot has {driver.version}"
                )

            viscous_scale = [0.0] * 6
            coulomb_scale = [0.0] * 6
            viscous_scale[JOINT_INDEX] = 1.0
            coulomb_scale[JOINT_INDEX] = 1.0
            driver.set_friction_scales(viscous_scale, coulomb_scale)

            robot.start_rtde_communication(consume_data=True)
            package = driver.get_data_package(timeout=2.0)
            if package is None:
                raise TimeoutError("Timed out waiting for the initial RTDE package")
            joint_positions = list(as_six_vector(package["actual_q"], "actual_q"))
            joint_positions[JOINT_INDEX] = START_POSITION_RAD

            deadline = time.monotonic() + args.duration if args.duration > 0 else None
            with InstructionExecutor(driver) as executor:
                if not _run_bounded(
                    executor,
                    lambda: executor.optimove_j(Q(joint_positions)),
                    deadline,
                ):
                    return

            timestep = 1.0 / driver.control_frequency
            maximum_control_time = PERIOD_COUNT / FREQUENCY_HZ
            receive_timeout = RobotReceiveTimeout.from_milliseconds(100)
            target_torques = [0.0] * 6
            control_time = 0.0

            while control_time < maximum_control_time:
                if deadline is not None and time.monotonic() >= deadline:
                    print("Duration reached; stopping torque control.")
                    break
                if driver.get_data_package(timeout=0.2) is None:
                    continue

                control_time += timestep
                target_torques[JOINT_INDEX] = AMPLITUDE_NM * math.sin(
                    2.0 * math.pi * FREQUENCY_HZ * control_time
                )
                driver.write_joint_command(
                    target_torques,
                    ControlMode.TORQUE,
                    receive_timeout,
                )
        finally:
            with suppress(Exception):
                driver.set_friction_scales((1.0,) * 6, (1.0,) * 6)
            safe_stop_control(driver)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if not confirm_start(
        args.robot_ip,
        warning=(
            "This example releases the brakes, starts external control, moves wrist joint 3 "
            "to zero radians, and applies open-loop torque. Keep the entire robot workspace clear."
        ),
        yes=args.yes,
        simulator=args.simulator,
    ):
        print("Example cancelled.")
        return 2
    return run_cli(lambda: run(args))


if __name__ == "__main__":
    raise SystemExit(main())
