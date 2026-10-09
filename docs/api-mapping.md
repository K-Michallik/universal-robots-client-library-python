# `C++` to Python API policy

The Python package preserves wire behavior and state transitions, not `C++` call
shapes.

## Naming and ownership

- `C++` namespaces map to Python subpackages.
- Classes use PascalCase; methods, properties, and arguments use snake_case.
- Constructors only validate and store configuration.
- `connect()` opens client transports; `start()` starts protocol activity.
- `close()` is idempotent and bounded. Network clients are context managers.
- `UrDriver` does not own a dashboard client.

## Values and return types

- `vector6d_t` joint values map to `Q`.
- Cartesian targets map to `Pose`; `MotionTarget` is `Q | Pose`.
- Public sequence inputs are validated for exact length and range.
- Returned vectors are immutable tuples.
- `C++` output parameters become return values.
- `DataPackage` mapping reads return `RTDEValue`, the union of supported RTDE
  scalar and fixed-tuple values. Writes still accept objects and validate them
  against the recipe field at runtime.
- Polling reads return `DataPackage | None`.
  `UrDriver.get_data_package_blocking()` takes no timeout and returns
  `DataPackage` or raises.
- High-level driver properties and operations expose `RTDEWriter`, Primary
  client contracts, `ErrorCode`, `MotionPrimitive`, `TrajectoryResult`,
  `TrajectoryControlMessage`, and tool/trajectory enums instead of `object`.
- Successful commands return `None` or a meaningful typed response.

## Representative method mapping

| C++ URCL | Python |
| --- | --- |
| `RTDEClient::init(...)` | `RTDEClient.connect(...)` |
| `RTDEClient::start(true)` | `RTDEClient.start(background=True)` |
| `getDataPackage(output, timeout)` | `get_data_package(timeout) -> DataPackage \| None` |
| `getDataPackageBlocking(output)` | `get_data_package_blocking() -> DataPackage` |
| `DataPackage::getData(key, output)` | `package[key]` |
| `DataPackage::setData(key, value)` | `package[key] = value` |
| `PrimaryClient::getErrorCodes()` | `primary.pop_error_codes()` |
| `registerTrajectoryDoneCallback(callback)` | `subscribe_trajectory_done(callback) -> unsubscribe` |
| `writeMotionPrimitive(shared_ptr)` | `write_motion_primitive(primitive)` |
| `DashboardClient(host, policy)` | `DashboardClient(host, backend=...)` |

The dashboard backend is mandatory in Python:

```python
from ur_client_library import DashboardClient, DashboardClientPolicy

dashboard = DashboardClient(
    "192.168.56.101",
    backend=DashboardClientPolicy.G5,
)
```

This avoids guessing whether a controller exposes the G5 TCP dashboard or the
PolyScope X Robot API.

Injected `UrDriver` factories use narrow component protocols. Test doubles and
alternate implementations must provide the same canonical Python method names
and signatures as the built-in RTDE, Primary, reverse, trajectory,
script-command, reader, and sender components.

## Errors

- Transport failures raise `URCLConnectionError`, which subclasses both
  `URCLException` and Python's built-in `ConnectionError`.
- Invalid lifecycle operations raise `LifecycleError`.
- Malformed packets raise `ProtocolError` or `ParseError`.
- Rejected commands raise a typed `CommandError`.
- Unknown RTDE keys raise `RTDEInvalidKeyError`, also a `KeyError`.
- Unsupported PolyScope X calls raise `NotImplementedError`.

Important specialized errors include:

- `URCLTimeoutError` for configured operation deadlines;
- `WorkerThreadError` when a retained worker failure reaches a public call;
- `UnexpectedResponseError` for syntactically valid protocol surprises;
- `VersionMismatchError` and its robot-feature subclasses;
- `RTDEInputConflictError` for inputs owned by another client;
- `DashboardCommandError` for rejected dashboard operations;
- `ScriptSyntaxError` (with `ScriptIncludeError` for rejected include paths),
  `ScriptFileError` for unreadable script templates, and the
  `ScriptExecutionError` hierarchy;
- `RobotModeError` and `SafetyModeError` for invalid command preconditions.

Import common high-level classes and errors from `ur_client_library`. Protocol
building blocks remain in their explicit subpackages:

```python
from ur_client_library import UrDriver, URCLConnectionError
from ur_client_library.rtde import RTDEClient
from ur_client_library.control import ReverseInterface
from ur_client_library.comm import Pipeline
```

## Dashboard backend differences

The Python backends preserve upstream's intentionally unequal feature sets.

- G5 does not implement `resume()` (use `play()`), program listing, upload,
  update, or download.
- PolyScope X does not implement installation loading, popup/logging helpers,
  shutdown, legacy version/model/serial queries, safety-status query,
  operational-mode mutation, user roles, flight/support reports, log saving,
  or raw TCP/regex/retry helpers.
- PolyScope X program list/upload/update/download operations require Robot API
  3.1.4 (PolyScope 10.12.0) or newer.

Unsupported operations raise `NotImplementedError` rather than silently
falling back to a different transport.

## Concurrency

- Reader, writer, and server threads are private implementation details.
- User callbacks run on worker threads and are never called while an internal
  state lock is held.
- Subscriptions return an idempotent zero-argument unsubscribe callable.
- A worker failure is retained and raised by the next relevant public call.
- Blocking and background RTDE reads are mutually exclusive.

See `lifecycle-and-safety.md` for shutdown, retry, callback-thread, network
binding, and real-time caveats.
