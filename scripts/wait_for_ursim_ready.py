#!/usr/bin/env python3
"""Block until a PolyScope X URSim is ready to accept robot-control connections.

PolyScope X leaves the safety configuration unconfirmed on first boot. The
confirmation is normally driven by the PolyScope X web frontend, so CI used to
load the UI in a headless browser and sleep. That is racy in both directions:
the browser may confirm long after the sleep expires, and its confirmation
power-cycles the robot, which aborts an already-running external-control
program. Driving the same REST call directly makes the step deterministic.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request

SERVICES = "/universal-robots/polyscopex-services/polyscopex-services"
UNCONTROLLABLE_MODES = frozenset({"CONFIRM_SAFETY", "NO_CONTROLLER", "BOOTING"})


def _request(url: str, method: str = "GET", timeout: float = 5.0):
    request = urllib.request.Request(url, method=method)
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            body = response.read()
            return json.loads(body) if body else {}
    except (urllib.error.URLError, OSError, ValueError):
        return None


def _bootstrapper_finished(base: str) -> bool:
    status = _request(f"{base}/universal-robots/bootstrapper/status")
    return bool(status) and status.get("isFinished") is True


def _safety_confirmed(base: str) -> bool:
    return bool((_request(f"{base}{SERVICES}/rest-api/safety/crc") or {}).get("crc"))


def _robot_controllable(base: str) -> bool:
    robot = (_request(f"{base}{SERVICES}/rest-api/robot/state") or {}).get("robot") or {}
    mode = robot.get("robotMode")
    if mode is None or mode in UNCONTROLLABLE_MODES:
        return False
    return robot.get("safetyMode") == "NORMAL"


def _await(label: str, predicate, deadline: float) -> None:
    started = time.monotonic()
    while not predicate():
        if time.monotonic() >= deadline:
            raise TimeoutError(label)
        time.sleep(2.0)
    print(f"[ursim-ready] {label} after {time.monotonic() - started:.1f}s", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--timeout", type=float, default=600.0)
    args = parser.parse_args()

    base = f"http://{args.robot_ip}"
    deadline = time.monotonic() + args.timeout
    try:
        _await("bootstrapper finished", lambda: _bootstrapper_finished(base), deadline)
        if not _safety_confirmed(base):
            if _request(f"{base}{SERVICES}/rest-api/safety/confirm", method="POST") is None:
                print("[ursim-ready] safety confirmation request failed", file=sys.stderr)
                return 1
            _await("safety configuration confirmed", lambda: _safety_confirmed(base), deadline)
        else:
            print("[ursim-ready] safety configuration already confirmed", flush=True)
        _await("robot controllable", lambda: _robot_controllable(base), deadline)
    except TimeoutError as exc:
        print(f"[ursim-ready] TIMEOUT waiting for {exc.args[0]}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
