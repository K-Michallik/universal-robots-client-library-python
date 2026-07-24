# Copyright 2022 Universal Robots A/S
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

"""Read and report the controller's Primary-interface calibration status."""

from __future__ import annotations

import argparse
import threading
from collections.abc import Sequence

from ur_client_library.comm import Pipeline, URProducer, URStream
from ur_client_library.primary import (
    UR_PRIMARY_PORT,
    KinematicsInfo,
    PackageHeader,
    PrimaryPackage,
    PrimaryParser,
)

from ._common import ExampleSkipped, create_parser, run_cli


class CalibrationConsumer:
    """Capture the first kinematics package delivered by a pipeline."""

    LINEARIZED = 2

    def __init__(self) -> None:
        self._status: int | None = None
        self._received = threading.Event()

    def consume(self, product: PrimaryPackage) -> bool:
        if isinstance(product, KinematicsInfo):
            print(product)
            self._status = product.calibration_status
            self._received.set()
        return True

    def on_timeout(self) -> None:
        """The pipeline has no timeout work for this consumer."""

    def wait(self, timeout: float) -> bool:
        return self._received.wait(None if timeout == 0 else timeout)

    @property
    def is_calibrated(self) -> bool:
        return self._status == self.LINEARIZED


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse command-line arguments without opening the Primary interface."""

    parser = create_parser("Report the controller's calibration status")
    return parser.parse_args(argv)


def run(args: argparse.Namespace) -> None:
    """Wait for calibration data and report whether it is valid."""

    stream = URStream(
        args.robot_ip,
        UR_PRIMARY_PORT,
        length_size=PackageHeader.SIZE,
        length_decoder=PackageHeader.get_package_length,
        max_packet_size=16 * 1024 * 1024,
    )
    producer = URProducer(stream, PrimaryParser())
    consumer = CalibrationConsumer()
    pipeline = Pipeline(producer, consumer, name="Primary calibration pipeline")

    with pipeline:
        received = consumer.wait(args.duration)

    if not received:
        raise ExampleSkipped(
            f"no calibration status was received within {args.duration:g} seconds"
        )
    if consumer.is_calibrated:
        print(f"The robot on IP {args.robot_ip} is calibrated.")
    else:
        print(
            f"The robot controller on IP {args.robot_ip} does not have a valid calibration."
        )
        print("Remember to turn on the robot to get calibration stored on the robot.")


def main(argv: Sequence[str] | None = None) -> int:
    """Run the example as a command-line program."""

    args = parse_args(argv)
    return run_cli(lambda: run(args))


if __name__ == "__main__":
    raise SystemExit(main())
