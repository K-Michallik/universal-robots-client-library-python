# URCL 2.14.0 parity ledger

Source of truth: upstream commit
`91be9eeac2028b0a5ee3654b5d12508cebefc1e6`, package version 2.14.0.

The translation plan stated that the checkout contains 49 `.cpp` files. The
checkout actually contains 48. All 48 are listed below; implementation-bearing
headers and the bundled URScript are tracked separately.

Legend: `[x]` implemented, `[~]` partially implemented, `[ ]` not yet
implemented, `[-]` deliberately omitted.

## Foundations

- [x] `src/types.cpp` → `types.py`
- [x] `src/helpers.cpp` → `helpers.py`
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
- [x] `comm/stream.h` → `comm/stream.py`
- [x] `comm/producer.h` → `comm/producer.py`
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
  `primary/packages.py`
- [x] `src/primary/robot_message/safety_mode_message.cpp` →
  `primary/packages.py`
- [x] `src/primary/primary_client.cpp` → `primary/client.py`

Header implementations:

- [~] `primary/primary_parser.h` → `primary/parser.py` (structured subset with
  raw fallback)
- [x] `primary/primary_consumer.h` → private snapshot consumer
- [-] `primary/abstract_primary_consumer.h` visitor API → one-package protocol
- [x] package and subpackage enum headers

## External control

- [x] `src/control/reverse_interface.cpp` → `control/reverse_interface.py`
- [x] `src/control/trajectory_point_interface.cpp` →
  `control/trajectory_point_interface.py`
- [x] `src/control/script_command_interface.cpp` →
  `control/script_command_interface.py`
- [x] `src/control/script_reader.cpp` → `control/script_reader.py`
- [x] `src/control/script_sender.cpp` → `control/script_sender.py`
- [x] `src/control/motion_primitives.cpp` → `control/motion_primitives.py`
- [x] `src/ur/tool_communication.cpp` → `ur/tool_communication.py`
- [x] `resources/external_control.urscript` →
  `resources/external_control.urscript` unchanged

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
