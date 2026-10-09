# Copyright 2022 Universal Robots A/S
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
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

"""Exercise the dashboard commands used by the upstream C++ example."""

from __future__ import annotations

import argparse
import time
from collections.abc import Sequence
from contextlib import suppress

from ur_client_library import DashboardClient, DashboardClientPolicy, PrimaryClient
from ur_client_library.exceptions import DashboardCommandError, VersionMismatchError

from ._common import ExampleSkipped, confirm_start, create_parser, run_cli


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse command-line arguments without contacting a robot."""

    parser = create_parser("Exercise the Universal Robots dashboard client")
    return parser.parse_args(argv)


def run(args: argparse.Namespace) -> None:
    """Run the dashboard command sequence against one controller."""

    with PrimaryClient(args.robot_ip) as primary:
        version = primary.wait_for_version()

    if version.major >= 10 and (version.major, version.minor) < (10, 12):
        raise ExampleSkipped(
            "the dashboard example requires PolyScope 10.12.0 or newer on PolyScope X"
        )

    policy = (
        DashboardClientPolicy.G5
        if version.major < 10
        else DashboardClientPolicy.POLYSCOPE_X
    )
    dashboard = DashboardClient(args.robot_ip, policy)
    dashboard.set_receive_timeout(10.0)
    try:
        with dashboard:
            powered = False
            program_started = False
            try:
                remote_control = version.major >= 5 and dashboard.is_in_remote_control()

                if version.major < 10:
                    dashboard.power_off()
                    print(f"PolyScope version: {dashboard.polyscope_version}")
                    dashboard.close_safety_popup()
                elif remote_control:
                    # As upstream, ignore the result: the Robot API rejects
                    # powering off an already powered-off robot (HTTP 409).
                    with suppress(DashboardCommandError):
                        dashboard.power_off()

                if version.major >= 10 and not remote_control:
                    print("PolyScope X is not in remote control; no state-changing commands sent.")
                    return

                dashboard.power_on()
                powered = True
                dashboard.brake_release()
                program_name = "wait_program.urp" if version.major < 10 else "wait_program"
                try:
                    dashboard.load_program(program_name)
                except DashboardCommandError as exc:
                    raise ExampleSkipped(
                        f"dashboard prerequisite {program_name!r} is not installed"
                    ) from exc
                time.sleep(1.0)
                dashboard.play()
                program_started = True
                dashboard.pause()
                if version.major >= 10:
                    dashboard.resume()
                else:
                    dashboard.play()
                dashboard.stop()
                program_started = False
                dashboard.power_off()
                powered = False

                if version.major < 10:
                    dashboard.save_log()
                    print(f"Program state: {dashboard.send_and_receive('programState')}")
                    dashboard.send_request("power off", "Powering off")
                    print("Power off command success: true")
            finally:
                if program_started:
                    with suppress(Exception):
                        dashboard.stop()
                if powered:
                    with suppress(Exception):
                        dashboard.power_off()
    except DashboardCommandError as exc:
        if not args.simulator:
            raise
        raise ExampleSkipped(f"URSim could not complete a dashboard transition: {exc}") from exc
    except (NotImplementedError, VersionMismatchError) as exc:
        raise ExampleSkipped(f"dashboard operation is unsupported: {exc}") from exc


def main(argv: Sequence[str] | None = None) -> int:
    """Run the example as a command-line program."""

    args = parse_args(argv)
    if not confirm_start(
        args.robot_ip,
        warning=(
            "This example powers the robot on and off and loads, starts, pauses, "
            "resumes, and stops a program."
        ),
        yes=args.yes,
        simulator=args.simulator,
    ):
        print("Example cancelled.")
        return 2
    return run_cli(lambda: run(args))


if __name__ == "__main__":
    raise SystemExit(main())
