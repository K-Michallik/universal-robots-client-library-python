"""Send blocking and non-blocking URScript through the Primary interface."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from contextlib import suppress

from ur_client_library import PrimaryClient, URCLException
from ur_client_library.exceptions import RobotRuntimeError

from ._common import confirm_start, create_parser, run_cli

FULLY_DEFINED_SCRIPT = """
# This is a fully defined script, function definition and all.
# Comments and whitespace-only lines are stripped before sending.
def example_fun():
  movej([0,-1.2,1.2,-0.1,1.57,0])
  sleep(0.1)
  current_pose = get_target_tcp_pose()
  relative_move = p[0,-0.1,0,0,0,0]
  movel(pose_trans(current_pose, relative_move), t=1)
end
"""

SECONDARY_SCRIPT = """
sec sec_script():
  textmsg("Named secondary program")
end
"""

BAD_SCRIPT_CODE = """
def bad_code():
  current_pose = get_target_tcp_pose()
  movel(current_pos)
end
"""


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse command-line arguments without connecting to the Primary interface."""

    parser = create_parser("Send URScript through the Primary interface")
    return parser.parse_args(argv)


def run(args: argparse.Namespace) -> None:
    """Execute the script sequence from the upstream example."""

    with PrimaryClient(args.robot_ip) as client:
        client.command_brake_release()
        if not client.safety_mode_allows_execution():
            raise URCLException(
                "Robot is not in a safety state where script execution is possible"
            )

        client.send_script_blocking(FULLY_DEFINED_SCRIPT)
        client.send_script_blocking('textmsg("Successful program execution")')
        client.send_script_blocking(
            'textmsg("hello")',
            "cool_function_name",
            start_timeout=0,
        )
        client.send_script_blocking(SECONDARY_SCRIPT)

        print("Sending bad script code...")
        try:
            client.send_script_blocking(BAD_SCRIPT_CODE)
        except RobotRuntimeError as exc:
            print("Caught expected runtime exception from send_script_blocking:")
            print(exc)

        unmonitored_motion_sent = False
        try:
            print("Executing motion without feedback")
            client.send_script("movej([0.1,-0.9,0.9,0,0,0])")
            unmonitored_motion_sent = True
            print("Sending bad script code without feedback...")
            client.send_script(BAD_SCRIPT_CODE)
            print("Bad code sent to robot successfully? true")
        finally:
            if unmonitored_motion_sent:
                with suppress(Exception):
                    client.command_stop(timeout=2.0)


def main(argv: Sequence[str] | None = None) -> int:
    """Run the example as a command-line program."""

    args = parse_args(argv)
    if not confirm_start(
        args.robot_ip,
        warning="This example releases the brakes and executes several robot motions.",
        yes=args.yes,
        simulator=args.simulator,
    ):
        print("Example cancelled.")
        return 2
    return run_cli(lambda: run(args))


if __name__ == "__main__":
    raise SystemExit(main())
