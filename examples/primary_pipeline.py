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

"""Print packages received through a composed Primary-interface pipeline."""

from __future__ import annotations

import argparse
from collections.abc import Sequence

from ur_client_library.comm import Pipeline, URProducer, URStream
from ur_client_library.primary import (
    UR_PRIMARY_PORT,
    PackageHeader,
    PrimaryPackage,
    PrimaryParser,
)

from ._common import create_parser, run_cli, wait_for_duration


class PrintingConsumer:
    """Minimal consumer matching the Python pipeline's consumer protocol."""

    def consume(self, product: PrimaryPackage) -> bool:
        print(product)
        return True

    def on_timeout(self) -> None:
        """The pipeline has no timeout work for this consumer."""


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse command-line arguments without opening the Primary interface."""

    parser = create_parser("Print Primary-interface packages through a pipeline")
    return parser.parse_args(argv)


def run(args: argparse.Namespace) -> None:
    """Connect and print Primary packages for the requested duration."""

    stream = URStream(
        args.robot_ip,
        UR_PRIMARY_PORT,
        length_size=PackageHeader.SIZE,
        length_decoder=PackageHeader.get_package_length,
        max_packet_size=16 * 1024 * 1024,
    )
    producer = URProducer(stream, PrimaryParser())
    pipeline = Pipeline(producer, PrintingConsumer(), name="Primary pipeline")
    with pipeline:
        wait_for_duration(args.duration)


def main(argv: Sequence[str] | None = None) -> int:
    """Run the example as a command-line program."""

    args = parse_args(argv)
    return run_cli(lambda: run(args))


if __name__ == "__main__":
    raise SystemExit(main())
