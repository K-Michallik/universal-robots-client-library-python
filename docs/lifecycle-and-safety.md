# Lifecycle, threading, and safety

This package preserves URCL 2.14.0 protocol behavior while exposing explicit
Python lifecycle methods.

## Object lifecycle

Constructors validate and retain configuration without opening sockets.
`connect()` establishes transports and performs protocol negotiation;
`start()` begins protocol activity where the interface has a distinct started
state. `close()` is idempotent.

Network clients are context managers. Entering connects and exiting closes:

```python
from ur_client_library.rtde import RTDEClient

with RTDEClient("192.168.56.101", output_recipe=("timestamp",)) as client:
    client.start()
    package = client.get_data_package(timeout=1.0)
```

Calling an operation in the wrong state raises `LifecycleError`. Polling reads
return `None` only when their caller-supplied timeout expires; a connection or
worker failure raises.

## Worker threads and shutdown

Socket servers, pipelines, RTDE readers/writers, and reconnect paths use
private worker threads. Worker failures are retained and raised by the next
relevant public operation as `WorkerThreadError`.

Callbacks run on worker threads and are called without internal state locks
held. A callback must return quickly and must not call a blocking shutdown on
the same worker. Offload expensive work to an application-owned queue or
executor.

Shutdown is bounded and best effort. A worker blocked in operating-system I/O
may outlive `close()` briefly; the library logs that condition instead of
blocking an application forever. Long-running services that repeatedly
connect and disconnect should monitor worker errors and verify cleanup before
creating replacement clients.

## Retry and timeout defaults

For compatibility with upstream URCL, a connection-attempt count of `0` means
retry indefinitely. Use a finite value when configuration errors or an
unreachable robot must fail promptly.

Socket receive timeouts, robot-side reverse-socket receive timeouts, RTDE
polling timeouts, and dashboard HTTP timeouts are distinct settings. Do not
substitute one for another.

## Listening interfaces and network boundaries

Reverse, trajectory, script-command, and script-sender servers listen on
`0.0.0.0` by default so a robot can connect back to the host. This exposes
their ports on every host interface. Run the driver on a trusted robot network
and restrict firewall access to expected controller addresses.

Dashboard, Primary, and RTDE protocols are not credentials or encryption
boundaries. Do not expose robot controller ports to untrusted networks.

## Driver and wrapper behavior

`UrDriverConfiguration.headless_mode=False` expects an external-control
URCap/URCapX program to request the bundled script. Set `headless_mode=True`
only when the application should upload and start the generated program
through the Primary interface.

`ExampleRobotWrapper` is an unstable, opinionated helper for examples and
supervised setup. It may clear a protective stop, release brakes, start a
program, and owns a Primary connection in addition to `UrDriver`'s Primary
connection. Applications that need explicit ownership or reconnect behavior
should compose `PrimaryClient`, `DashboardClient`, and `UrDriver` directly.

## Timing and wire compatibility

Pure CPython does not provide C++ hard-real-time guarantees. RTDE at 125 Hz or
500 Hz is sensitive to interpreter, garbage-collection, kernel, and competing
process jitter. Best-effort FIFO scheduling can reduce host scheduling delays
but does not make Python a hard-real-time runtime.

The current 21-word trajectory layout, microsecond `MULT_TIME` scaling, and
bundled `external_control.urscript` form one wire-protocol revision. Keep them
in lockstep. Pairing this serializer with a pre-2.12 script can misinterpret
motion targets and parameters.

Validate protocol, cleanup, and command behavior in URSim before supervised
physical operation. URSim does not validate collision clearance, torque,
contact, force response, or real-time deadlines.
