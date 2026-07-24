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

"""Read RTDE state while demonstrating speed-slider input writes."""

from __future__ import annotations

import argparse
import math
import sys
import time
from collections.abc import Sequence
from contextlib import suppress

from ur_client_library.rtde import RTDEClient

from ._common import confirm_start, create_parser, recipe_path, run_cli

TARGET_SPEED_FRACTION = "target_speed_fraction"
RTDE_FREQUENCY = 50.0


def _print_fraction(fraction: float, label: str, width: int = 20) -> None:
    display_fraction = min(1.0, max(0.0, fraction))
    filled = math.ceil(display_fraction * width)
    empty = math.floor((1.0 - display_fraction) * width)
    sys.stdout.write(f"\r{label}: [{'#' * filled}{'-' * empty}]")
    sys.stdout.flush()


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse command-line arguments without connecting to RTDE."""

    parser = create_parser("Read RTDE data and vary the robot speed slider")
    return parser.parse_args(argv)


def run(args: argparse.Namespace) -> None:
    """Run the RTDE read/write loop for the requested duration."""

    client = RTDEClient(
        args.robot_ip,
        str(recipe_path("rtde_output_recipe.txt")),
        str(recipe_path("rtde_input_recipe.txt")),
        target_frequency=RTDE_FREQUENCY,
    )
    with client:
        client.start(background=True)
        speed_slider_fraction = 1.0
        speed_slider_increment = 0.01
        deadline = time.monotonic() + args.duration if args.duration else None
        try:
            while deadline is None or time.monotonic() < deadline:
                timeout = 1.0 if deadline is None else max(0.0, deadline - time.monotonic())
                data_package = client.get_data_package(timeout=timeout)
                if data_package is None:
                    continue
                target_value = data_package[TARGET_SPEED_FRACTION]
                if isinstance(target_value, bool) or not isinstance(target_value, (int, float)):
                    raise TypeError(f"{TARGET_SPEED_FRACTION} is not a scalar RTDE value")
                target_speed_fraction = float(target_value)
                _print_fraction(target_speed_fraction, TARGET_SPEED_FRACTION)

                next_fraction = speed_slider_fraction + speed_slider_increment
                if next_fraction > 1.0 or next_fraction < 0.0:
                    speed_slider_increment *= -1.0
                speed_slider_fraction += speed_slider_increment
                client.writer.send_speed_slider(speed_slider_fraction)
        finally:
            with suppress(Exception):
                client.writer.send_speed_slider(1.0)
                time.sleep(0.05)
            print()


def main(argv: Sequence[str] | None = None) -> int:
    """Run the example as a command-line program."""

    args = parse_args(argv)
    if not confirm_start(
        args.robot_ip,
        warning="This example continuously changes the robot speed slider between 0% and 100%.",
        yes=args.yes,
        simulator=args.simulator,
    ):
        print("Example cancelled.")
        return 2
    return run_cli(lambda: run(args))


if __name__ == "__main__":
    raise SystemExit(main())
