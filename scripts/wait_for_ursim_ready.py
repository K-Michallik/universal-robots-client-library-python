#!/usr/bin/env python3
"""Block until a PolyScope X URSim is ready to accept robot-control connections.

PolyScope X leaves the safety configuration unconfirmed on first boot. The
confirmation is normally driven by the PolyScope X web frontend, so CI used to
load the UI in a headless browser and sleep. That is racy in both directions:
the browser may confirm long after the sleep expires, and its confirmation
power-cycles the robot, which aborts an already-running external-control
program. Driving the same REST call directly makes the step deterministic.

The Robot API program endpoints (/program/v1/loaded and friends) are served
through the web frontend, though, and hang until the client times out when no
UI session is open. Jobs that load programs therefore open the UI again, but
only after this script has confirmed safety, and pass --frontend-loaded to
wait until the UI has finished starting. The endpoints answer (412, "No
program loaded") well before that, but the starting UI still loads its last
program and can power the robot off, which aborts an external-control program
started in that window.

"Controllable" is not yet "quiet": PolyScope X keeps starting URCap containers
for several seconds afterwards, and the resulting CPU contention stalls the
realtime path long enough to trip a servo example's receive timeout. The last
gate, after the frontend wait when there is one, therefore waits for a steady
RTDE stream. A run that will be followed by opening the UI passes
--no-steady-rtde and leaves that gate to the --frontend-loaded run, since the
starting UI can stall the stream again.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request
from collections.abc import Callable

SERVICES = "/universal-robots/polyscopex-services/polyscopex-services"
ROBOT_API = "/universal-robots/robot-api"
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


def _loaded_program(base: str) -> str | None:
    # The starting frontend ends by loading its last program, so a 200 with a
    # name marks it as done; 412 (no program yet) and timeouts mean not yet.
    payload = _request(f"{base}{ROBOT_API}/program/v1/loaded")
    name = (payload or {}).get("name")
    return name if isinstance(name, str) and name else None


def _await(label: str, predicate, deadline: float) -> None:
    started = time.monotonic()
    while not predicate():
        if time.monotonic() >= deadline:
            raise TimeoutError(label)
        time.sleep(2.0)
    print(f"[ursim-ready] {label} after {time.monotonic() - started:.1f}s", flush=True)


# The servo-style examples halt the robot program once the gap between two
# commands exceeds their receive timeout (at most 200 ms in realtime modes).
# Right after boot, PolyScope X is still starting URCap containers, and the
# CPU contention on a shared runner stalls the realtime path for up to ~200 ms
# for several seconds. Hold off until the RTDE stream has been steady for a
# while, using half the tightest receive timeout as the stall threshold.
STEADY_WINDOW = 10.0
STEADY_MAX_GAP = 0.1
STEADY_TIMEOUT = 120.0
RTDE_FREQUENCY = 500.0


def await_steady_stream(
    next_arrival: Callable[[], float],
    *,
    window: float,
    max_gap: float,
    deadline: float,
) -> float:
    """Consume package arrival times until they have been gap-free for ``window``.

    ``next_arrival`` blocks for the next package and returns its arrival time on a
    monotonic clock. Returns the largest gap seen. Raises ``TimeoutError`` with
    that gap when ``deadline`` (same clock) passes first.
    """
    first = next_arrival()
    steady_since = previous = first
    worst = 0.0
    while True:
        arrival = next_arrival()
        gap = arrival - previous
        previous = arrival
        worst = max(worst, gap)
        if gap > max_gap:
            steady_since = arrival
        elif arrival - steady_since >= window:
            return worst
        if arrival >= deadline:
            raise TimeoutError(worst)


def _await_steady_rtde(robot_ip: str) -> None:
    from ur_client_library.rtde.rtde_client import RTDEClient

    client = RTDEClient(robot_ip, ("timestamp",), target_frequency=RTDE_FREQUENCY)
    started = time.monotonic()
    with client:
        client.start(background=False)

        def next_arrival() -> float:
            client.read_data_package()
            return time.monotonic()

        try:
            worst = await_steady_stream(
                next_arrival,
                window=STEADY_WINDOW,
                max_gap=STEADY_MAX_GAP,
                deadline=started + STEADY_TIMEOUT,
            )
        except TimeoutError as exc:
            # Best effort: an always-jittery host should not block CI outright.
            print(
                f"[ursim-ready] WARNING: RTDE stream not steady after {STEADY_TIMEOUT:.0f}s "
                f"(largest gap {exc.args[0] * 1000:.0f} ms); continuing",
                file=sys.stderr,
                flush=True,
            )
            return
    print(
        f"[ursim-ready] RTDE stream steady for {STEADY_WINDOW:.0f}s after "
        f"{time.monotonic() - started:.1f}s (largest gap {worst * 1000:.0f} ms)",
        flush=True,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--timeout", type=float, default=600.0)
    parser.add_argument(
        "--frontend-loaded",
        action="store_true",
        help=(
            "also wait until an open PolyScope X web frontend has finished "
            "starting, which it signals by loading a program"
        ),
    )
    parser.add_argument(
        "--steady-rtde",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "finish by waiting until the RTDE stream has had no gap over "
            f"{STEADY_MAX_GAP * 1000:.0f} ms for {STEADY_WINDOW:.0f} s, continuing with a "
            f"warning after {STEADY_TIMEOUT:.0f} s. It runs after the --frontend-loaded "
            "wait; skip it when the UI is opened afterwards"
        ),
    )
    args = parser.parse_args(argv)

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
        if args.frontend_loaded:
            _await(
                "web frontend loaded a program",
                lambda: _loaded_program(base) is not None,
                deadline,
            )
            print(f"[ursim-ready] loaded program: {_loaded_program(base)}", flush=True)
            _await("robot controllable", lambda: _robot_controllable(base), deadline)
        if args.steady_rtde:
            _await_steady_rtde(args.robot_ip)
    except TimeoutError as exc:
        print(f"[ursim-ready] TIMEOUT waiting for {exc.args[0]}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
