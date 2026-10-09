#!/usr/bin/env python3
"""Log controller-side program and robot-state events from the Primary interface.

CI runs this in the background while the examples execute. An example that
loses its external-control connection only sees the socket drop; the
controller's own account of why the program ended (PROGRAM_XXX_STOPPED, the
script's textmsg output, runtime exceptions, robot or safety mode changes)
arrives on the Primary interface and is recorded here with wall-clock
timestamps that line up with the example output.
"""

from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
from datetime import datetime

from ur_client_library.primary.client import PrimaryClient
from ur_client_library.primary.packages import (
    ErrorCodeMessage,
    KeyMessage,
    PrimaryPackage,
    RobotModeData,
    RuntimeExceptionMessage,
    SafetyModeMessage,
    TextMessage,
)


def _stamp() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


class _Printer:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._last_mode: tuple[object, ...] | None = None

    def emit(self, text: str) -> None:
        with self._lock:
            print(f"[primary {_stamp()}] {text}", flush=True)

    def __call__(self, package: PrimaryPackage) -> None:
        if isinstance(package, RobotModeData):
            mode = (
                package.robot_mode,
                package.is_program_running,
                package.is_program_paused,
                package.is_protective_stopped,
                package.is_emergency_stopped,
                package.is_robot_power_on,
            )
            if mode != self._last_mode:
                self._last_mode = mode
                self.emit(
                    f"mode={getattr(package.robot_mode, 'name', package.robot_mode)}"
                    f" program_running={package.is_program_running}"
                    f" paused={package.is_program_paused}"
                    f" pstop={package.is_protective_stopped}"
                    f" estop={package.is_emergency_stopped}"
                    f" power={package.is_robot_power_on}"
                )
        elif isinstance(package, KeyMessage):
            self.emit(f"key {package.title} {package.text!r} code={package.message_code}")
        elif isinstance(package, TextMessage):
            self.emit(f"text {package.text!r}")
        elif isinstance(package, RuntimeExceptionMessage):
            self.emit(
                f"runtime-exception line={package.line_number}:{package.column_number}"
                f" {package.text!r}"
            )
        elif isinstance(package, ErrorCodeMessage):
            level = getattr(package.report_level, "name", package.report_level)
            self.emit(f"error {package.code} level={level} {package.text!r}")
        elif isinstance(package, SafetyModeMessage):
            self.emit(f"safety {package!r}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-ip", required=True)
    args = parser.parse_args()

    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    printer = _Printer()
    while not stop.is_set():
        client = PrimaryClient(args.robot_ip)
        try:
            client.subscribe(printer)
            client.connect()
            client.start()
            printer.emit("connected")
            while not stop.is_set() and client.is_running:
                stop.wait(0.5)
        except Exception as exc:  # keep monitoring across controller restarts
            printer.emit(f"monitor error: {exc!r}")
        finally:
            client.close()
        if not stop.is_set():
            time.sleep(1.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
