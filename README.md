# Universal Robots Client Library Python

A synchronous, pure-Python client library for Universal Robots interfaces.
The implementation translates the active behavior of URCL 2.15.2 while
presenting Pythonic lifecycle, naming, typing, and error semantics.

> **Status:** the URCL 2.15.2 source-parity pass is complete. The project is a
> `0.1.0` alpha: focused unit/protocol tests and a Docker URSim example matrix
> cover the translated surface, while physical-hardware validation remains
> limited. One out-of-band PolyScope 5 smoke test established a connection and
> read RTDE data without sending a motion command; it is not part of CI.

## Scope

- TCP communication and producer/consumer pipelines
- RTDE negotiation, recipes, data packages, reconnect, and asynchronous writes
- Primary-interface state and robot-message parsing
- Reverse, trajectory, script-command, and script-sender protocols
- G5 dashboard TCP and PolyScope X Robot API clients
- `UrDriver`, `InstructionExecutor`, and `ExampleRobotWrapper`
- Python ports of all 17 CMake-built upstream examples

The supported controller scope for the runnable examples and URSim matrix is
PolyScope 5/e-Series and PolyScope X. CB3 example validation is intentionally
excluded because that controller generation is no longer supported.

The source baseline is
[UniversalRobots/Universal_Robots_Client_Library](https://github.com/UniversalRobots/Universal_Robots_Client_Library)
2.15.2 at commit `7747b683a68ca5f23d33fde4405e24e55d390b85`. Changes on upstream `master` after the
2.15.2 tag are not part of the baseline; see `docs/parity.md`.

## Installation

Python 3.10 or newer is required.

```bash
python3 -m pip install .
```

For development:

```bash
python3 -m pip install -e ".[dev]"
pytest
ruff check src tests examples run_examples.py
mypy
```



## Examples

The source checkout and sdist include safety-gated ports of all 17 official
`C++` examples, shared RTDE recipes, and a bulk runner that verifies a local
Docker URSim target by default. These repository examples are not installed
into the library-only wheel. Start with [`docs/examples.md`](docs/examples.md)
for the `C++` mapping, prerequisites, commands, controller-version skips, and
URSim limitations.

## API conventions

- Constructors validate and store configuration without opening sockets.
- `connect()` and `start()` perform network and worker-thread setup.
- `close()` is bounded and idempotent.
- Network clients support context managers that connect on entry and close on
exit.
- Failed operations raise typed exceptions. A polling read returns `None` only
when its caller-provided timeout expires.
- RTDE data-package reads expose `RTDEValue` values; high-level driver,
wrapper, and instruction APIs return their concrete package, client, writer,
error, and trajectory types.
- `UrDriver.get_data_package_blocking()` waits for and returns a `DataPackage`;
it does not accept a timeout.
- Public vectors are immutable tuples; inputs accept validated sequences.
- Subscription APIs return idempotent unsubscribe callables.

## Timing and safety

Pure CPython cannot provide the hard real-time guarantees of the `C++` library.
The implementation preserves protocol behavior and attempts best-effort FIFO
scheduling where useful, but applications must account for interpreter,
garbage-collection, and operating-system scheduling jitter at 125 Hz and
500 Hz.

This software controls industrial robots. Validate commands and safety
behavior in URSim before operating physical hardware.

Control servers listen on all interfaces by default so the robot can connect
back to the host. Restrict host firewall rules to trusted robot networks.
Connection retry count `0` means retry indefinitely, matching upstream; use a
finite limit when a wrong address must fail promptly.

The trajectory packet format and packaged `external_control.urscript` are one
versioned wire protocol. Do not pair this package's trajectory serializer with
an older external-control script.

## Known upstream limitations

Primary-interface parsing is intentionally partial. Unknown packages remain
available as raw bytes. PolyScope X dashboard operations that do not have an
upstream implementation raise `NotImplementedError`.

`ExampleRobotWrapper` is an unstable convenience for examples and supervised
robot setup, not a production driver facade. Like upstream, it owns a Primary
connection in addition to the one owned by `UrDriver`.

See `docs/parity.md` for the source-to-Python implementation ledger,
`docs/api-mapping.md` for lifecycle and naming differences, and
`docs/lifecycle-and-safety.md` for worker, retry, network-exposure, and timing
details. See `docs/examples.md` for runnable example guidance.

## License

Apache-2.0, with upstream BSD-licensed portions and attributions documented in
`NOTICE`.