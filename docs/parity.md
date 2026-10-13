# URCL 2.15.2 parity ledger

Source of truth: upstream tag 2.15.2, commit
`7747b683a68ca5f23d33fde4405e24e55d390b85`, package version 2.15.2. The previous baseline was
`91be9eeac2028b0a5ee3654b5d12508cebefc1e6` (2.14.0 plus two maintenance
commits); the 2.14.1 to 2.15.2 delta is recorded under
[Upstream 2.14.1 to 2.15.2](#upstream-2141-to-2152).

The translation plan stated that the checkout contains 49 `.cpp` files. The
checkout actually contains 48. All 48 are listed below; implementation-bearing
headers and the bundled URScript are tracked separately.

Legend: `[x]` implemented, `[~]` partially implemented, `[ ]` not yet
implemented, `[-]` deliberately omitted.

## Foundations

- [x] `src/types.cpp` → `types.py` (`Q.to_string()`, `Pose.to_string()`)
- [x] `src/helpers.cpp` → `helpers.py` (`string_from_motion_target()`)
- [x] `src/ur/version_information.cpp` → `ur/version_information.py`
- [x] `src/ur/robot_receive_timeout.cpp` → `ur/robot_receive_timeout.py`
- [x] `src/log.cpp` → `logging.py` standard-library integration
- [x] `src/default_log_handler.cpp` → opt-in `configure_logging()`
- [-] `src/ur/calibration_checker.cpp` → standalone deprecated consumer omitted;
  hashing and `PrimaryClient.check_calibration()` retained

## Communication

- [x] `src/comm/tcp_socket.cpp` → `comm/tcp_socket.py`
- [x] `src/comm/tcp_server.cpp` → `comm/tcp_server.py`

Header implementations:

- [x] `comm/bin_parser.h` → `comm/bin_parser.py`
- [x] `comm/package_serializer.h` → `comm/package_serializer.py`
- [x] `comm/stream.h` → `comm/stream.py` (`reconnect()`, `stop_requested`)
- [x] `comm/producer.h` → `comm/producer.py` (auto-reconnect never reopens a
  deliberately closed stream; back-off interruptible in 0.1 s slices)
- [x] `comm/pipeline.h` → `comm/pipeline.py`
- [x] `comm/parser.h`, `package.h` → `comm/parser.py`, `comm/package.py`
- [x] `comm/control_mode.h` → `comm/control_mode.py`
- [-] `comm/shell_consumer.h` → runnable/example-only behavior omitted
- [-] bundled lock-free queue → standard-library bounded queues

## RTDE

- [x] `src/rtde/rtde_package.cpp` → `rtde/rtde_package.py`
- [x] `src/rtde/data_package.cpp` → `rtde/data_package.py`
- [x] `src/rtde/text_message.cpp` → `rtde/text_message.py`
- [x] `src/rtde/request_protocol_version.cpp` →
  `rtde/request_protocol_version.py`
- [x] `src/rtde/get_urcontrol_version.cpp` →
  `rtde/get_urcontrol_version.py`
- [x] `src/rtde/control_package_setup_outputs.cpp` →
  `rtde/control_package_setup_outputs.py`
- [x] `src/rtde/control_package_setup_inputs.cpp` →
  `rtde/control_package_setup_inputs.py`
- [x] `src/rtde/control_package_start.cpp` →
  `rtde/control_package_start.py`
- [x] `src/rtde/control_package_pause.cpp` →
  `rtde/control_package_pause.py`
- [x] `src/rtde/rtde_parser.cpp` → `rtde/rtde_parser.py`
- [x] `src/rtde/rtde_client.cpp` → `rtde/rtde_client.py`
- [x] `src/rtde/rtde_writer.cpp` → `rtde/rtde_writer.py`

Header implementations:

- [x] `rtde/package_header.h`, `rtde/rtde_package.h`
- [x] `rtde/data_package.h`
- [x] `rtde/rtde_parser.h`
- [x] `rtde/rtde_client.h`
- [x] `rtde/rtde_writer.h`
- [x] full field-type registry in `rtde/rtde_package.h`

## Primary interface

Primary parsing is intentionally partial. The package models listed here are
implemented, while unmodeled state/message subpackages remain available as raw
bytes in lenient mode. This matches the supported Python surface and is not a
claim that every upstream Primary subpackage has a structured parser.

- [x] `src/primary/primary_package.cpp` → `primary/packages.py`
- [x] `src/primary/robot_state.cpp` → `primary/packages.py`
- [x] `src/primary/robot_message.cpp` → `primary/packages.py`
- [x] `src/primary/robot_state/robot_mode_data.cpp` → `primary/packages.py`
- [x] `src/primary/robot_state/kinematics_info.cpp` → `primary/packages.py`
- [x] `src/primary/robot_state/configuration_data.cpp` → `primary/packages.py`
- [x] `src/primary/robot_state/masterboard_data.cpp` → `primary/packages.py`
- [x] `src/primary/robot_message/version_message.cpp` → `primary/packages.py`
- [x] `src/primary/robot_message/text_message.cpp` → `primary/packages.py`
- [x] `src/primary/robot_message/runtime_exception_message.cpp` →
  `primary/packages.py`
- [x] `src/primary/robot_message/key_message.cpp` → `primary/packages.py`
- [x] `src/primary/robot_message/error_code_message.cpp` →
  `primary/packages.py` (`ErrorCode.description` /
  `ErrorCodeMessage.description`: override, exact, code-only, then bare
  `C<code>A<arg>`)
- [x] `src/primary/robot_message/safety_mode_message.cpp` →
  `primary/packages.py`
- [x] `src/primary/primary_client.cpp` → `primary/client.py`

Header implementations:

- [~] `primary/primary_parser.h` → `primary/parser.py` (structured subset with
  raw fallback)
- [x] `primary/primary_consumer.h` → private snapshot consumer; error codes
  are logged with upstream's report-level mapping
- [-] `primary/abstract_primary_consumer.h` visitor API → one-package protocol
- [x] package and subpackage enum headers
- [x] `ur/error_code_texts.h` (generated, JSON v40.121.0) →
  `ur/_error_code_texts.py`, generated by `scripts/generate_error_codes.py`
  from the URCL header at a git ref (local JSON plus overlay also supported)
- [x] `src/ur/error_code_overrides.cpp` → `ur/error_codes.py` (C100 robot-mode
  text)
- [-] `check_error_codes` CMake target and nightly workflow (no download step)

## External control

- [x] `src/control/reverse_interface.cpp` → `control/reverse_interface.py`
- [x] `src/control/trajectory_point_interface.cpp` →
  `control/trajectory_point_interface.py` (`MULT_VEL_ACC` spline encoding,
  `vel_acc_multiplier`)
- [x] `src/control/script_command_interface.cpp` →
  `control/script_command_interface.py`
- [x] `src/control/script_reader.cpp` → `control/script_reader.py` (include
  root confinement through symlinks, depth limit 32, `is_variable_registered`)
- [x] `src/control/script_sender.cpp` → `control/script_sender.py`
- [x] `src/control/motion_primitives.cpp` → `control/motion_primitives.py`
- [x] `src/ur/tool_communication.cpp` → `ur/tool_communication.py`
- [x] `resources/external_control.urscript` →
  `resources/external_control.urscript`, byte-identical to 2.15.2

Implementation-bearing headers for reverse/trajectory/script callbacks,
configuration structs, enums, and validators are included in the mapped
modules.

## Dashboard and high-level API

- [x] `src/ur/dashboard_client.cpp` → `ur/dashboard_client.py`
- [x] `src/ur/dashboard_client_implementation_g5.cpp` →
  `ur/dashboard_g5.py`
- [x] `src/ur/dashboard_client_implementation_x.cpp` →
  `ur/dashboard_x.py`
- [x] `src/ur/ur_driver.cpp` → `ur/ur_driver.py`
- [x] `src/ur/instruction_executor.cpp` → `ur/instruction_executor.py`
- [x] `src/example_robot_wrapper.cpp` → `example_robot_wrapper.py`

All implementation-bearing headers for these sources are part of the parity
review. Deprecated constructor overloads and `C++` `WithResponse` method
duplication are deliberately omitted.

Known upstream code/documentation conflicts are resolved in favor of executed
code: RTDE initialization attempts must be greater than zero, and the
positions-only spline overload is omitted because the low-level implementation
always rejects its null velocity argument.

## Examples

All 17 executables declared by upstream `examples/CMakeLists.txt` have
safety-gated Python ports under `examples/`, with the three RTDE recipes copied
alongside them. The mapping and controller prerequisites are documented in
`docs/examples.md`.

The omitted `C++` `ShellConsumer` is replaced only inside the Primary pipeline
example by a Python-native printing consumer. The spline example supplies
explicit velocities, and callback/command handling follows the canonical
Python subscription and exception contracts.

Example support and the Docker matrix target PolyScope 5/e-Series and
PolyScope X. CB3 is intentionally outside the supported example-validation
scope even where translated protocol code retains historical compatibility.

## Upstream 2.14.1 to 2.15.2

| Upstream change | Release | Python status |
| --- | --- | --- |
| #545 BinParser out-of-bounds read | 2.14.1 | Not affected; `BinParser` bounds-checks every read |
| #546 script_reader hardening | 2.14.1 | Ported; path rejections raise `ScriptIncludeError`, filesystem failures `ScriptFileError` (an `OSError`). Paths resolve strictly first, so symlink loops fail on every Python as with `weakly_canonical` |
| 7df9f19 `RTDEInvalidKeyException` keys | 2.14.1 | Ported as `RTDEInvalidKeyError.invalid_keys`; unavailable outputs fail `connect()` without retry |
| #519 teardown during reconnect | 2.14.1 | Intent ported (`TCPSocket.reconnect()`, `stop_requested`, interruptible back-off); the C++ socket state machine, `unique_fd`, and `poll` TCP server are not needed |
| #559 spline vel/acc precision | 2.15.0 | Ported as one wire revision: script, serializer, `VEL_ACC_REPLACE`, legacy-script fallback |
| #558 MotionTarget string helpers | 2.15.0 | Ported |
| #549 Robot API 10.14 | 2.15.0 | Ported with a per-command version table |
| #553 PolyScope X remote control in CI | 2.15.0 | Ported to `scripts/start_ursim.sh -c` and the examples workflow; remote-control entries open the web frontend in headless Chrome (upstream opens it for every PolyScope X entry), and the dashboard example ignores the initial power-off error as upstream does |
| #564 error code texts | 2.15.1 | Ported |
| #565 nullable program-list fields | 2.15.1 | Ported |
| #540, #543, #554, #556, #569 | 2.14.1 to 2.15.2 | Build, URCap, or test-harness only |

Deliberate differences: PolyScope X commands do not auto-connect (they raise
`LifecycleError`), HTTP and filesystem failures raise instead of returning
`ok=false`, and out-of-range spline velocities raise `InvalidRangeError`
instead of returning false.

Not in the baseline (upstream `master` after 2.15.2): RTDE protocol v3 and
read properties, robot-reported RTDE field types, the Primary hardware-info
message, control box and tool flange types, g-series robot types, configurable
connect timeouts, `ExampleRobotWrapper(reverse_ip=...)`, the `rtde_roundtrip`
and `rtde_robot_properties` examples, and error-code texts v42.

## Validation scope

The automated offline suite covers the translated unit/protocol surface,
including typed high-level contracts, failed-connect cleanup, example imports,
uniform CLI behavior, recipe lookup, pure trajectory generation, safety
refusal, and bounded-runner behavior.

The Docker URSim workflow separately exercises the supported PolyScope
5/e-Series and PolyScope X example matrix, with explicit skips for known
version, geometry, remote-mode, and program prerequisites. This integration
matrix is not part of the offline pytest claim.

One manual, out-of-band PolyScope 5 smoke test established a connection and
read RTDE data without sending a motion command. That check is not
reproducible in CI and is not counted as automated coverage. The complete
upstream test suite, physical motion validation, timing benchmarks, and
hard-real-time claims remain outside the completed validation.

`tests/coverage.md` records the automated, integration, manual, and deferred
validation scopes.
