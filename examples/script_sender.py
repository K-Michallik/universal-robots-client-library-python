# Copyright 2024 Universal Robots A/S
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

"""Serve a fixed URScript program to clients requesting ``request_program``."""

from __future__ import annotations

import argparse
from collections.abc import Sequence

from ur_client_library.control import ScriptSender

from ._common import create_parser, run_cli, wait_for_duration

PORT = 12345
PROGRAM = 'textmsg("Hello, World!")'


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse command-line arguments without binding the server port."""

    parser = create_parser("Serve a fixed URScript program on TCP port 12345")
    return parser.parse_args(argv)


def run(args: argparse.Namespace) -> None:
    """Serve the example program for the requested duration."""

    with ScriptSender(PROGRAM, port=PORT) as sender:
        if args.duration > 0:
            print(
                f"Waiting for incoming requests on port {sender.port} "
                f"for {args.duration:g} seconds"
            )
        else:
            print(f"Waiting for incoming requests on port {sender.port} indefinitely")
        wait_for_duration(args.duration, period=0.5)

    if args.duration > 0:
        print("Timeout reached")


def main(argv: Sequence[str] | None = None) -> int:
    """Run the example as a command-line program."""

    args = parse_args(argv)
    return run_cli(lambda: run(args))


if __name__ == "__main__":
    raise SystemExit(main())
