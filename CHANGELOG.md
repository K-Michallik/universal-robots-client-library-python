# Changelog

## Unreleased

Source baseline moves from URCL 2.14.0 (`91be9ee`) to 2.15.2.

### Breaking

- An RTDE output that the controller reports as `NOT_FOUND` (with
  `ignore_unavailable_outputs=False`) makes `connect()` raise
  `RTDEInvalidKeyError` on the first attempt, instead of `URCLConnectionError`
  after retries.
- Script includes must be relative and resolve inside the top-level script's
  directory; absolute and `../` escapes raise `ScriptIncludeError`.
- Custom URScripts that adopt `{{VEL_ACC_REPLACE}}` must decode spline
  velocities and accelerations with `MULT_velacc`. Scripts without the
  placeholder keep the legacy encoding and log a warning.

### Added

- PolyScope X 10.14 (Robot API 5.0.107) dashboard commands: `popup`,
  `close_popup`, `close_safety_popup`, `shutdown`, `add_to_log`,
  `get_polyscope_version`, `get_robot_model`, `get_serial_number`,
  `generate_flight_report`, and `download_support_files`.
- UR error-code texts (JSON v40.121.0): `ErrorCode.description`,
  `ur_client_library.ur.error_codes`, and `scripts/generate_error_codes.py`.
- `RTDEInvalidKeyError.invalid_keys`, `TCPSocket.reconnect()`,
  `URStream.reconnect()`, `stop_requested`, `MULT_VEL_ACC`,
  `vel_acc_multiplier` accessors, `ScriptReader.is_variable_registered`,
  `ScriptIncludeError`, `ScriptFileError`, `Q.to_string()`,
  `Pose.to_string()`, and `string_from_motion_target()`.
- `scripts/start_ursim.sh -c` mounts a PolyScope X citadel database; the
  URSim matrix adds PolyScope X 10.14.0 and runs 10.13/10.14 in remote control.
- The remote-control URSim entries open the PolyScope X web frontend, which
  serves the Robot API program endpoints, in headless Chrome;
  `scripts/wait_for_ursim_ready.py --frontend-loaded` holds the examples back
  until the frontend has finished starting.

### Changed

- CI runs the Docker URSim matrix weekly and on demand
  (`gh workflow run examples.yml --ref <branch>`) instead of on every push and
  nightly; pushes and pull requests run the offline and package jobs, skip
  documentation-only changes, and cancel superseded runs.
- Spline velocities and accelerations are encoded with a 1e8 multiplier; the
  bundled `external_control.urscript` matches 2.15.2.
- PolyScope X commands are gated per command on the Robot API version; any 2xx
  response counts as success.
- The Primary consumer logs error codes by report level, and
  `RobotErrorCodeError` messages include the error description.
- `str()` of `Q`, `Pose`, and `RTDEInvalidKeyError` now gives the upstream
  text or the plain message; `repr()` is unchanged.

### Fixed

- The PolyScope X program list tolerates null `createdDate`, `description`,
  and `programState`.
- A failed `RTDEClient.connect()` closes its transport and returns to
  `UNINITIALIZED`.
- `URProducer` auto-reconnect no longer reopens a stream closed during back-off.
- In remote control, the dashboard example ignores PolyScope X rejecting the
  initial power-off of an already powered-off robot, as upstream does.

### Security

- Script includes are confined to the script directory, also through
  symlinks, and limited to 32 levels. Paths resolve strictly, so a symlink
  loop raises `ScriptFileError` on every supported Python, including 3.13.
- The Primary reader rejects package lengths above 65535 bytes before reading.

## 0.1.0

Initial private alpha.

- Translate the active URCL 2.14.0 protocol and high-level client surface to a
  synchronous, pure-Python package.
- Provide Pythonic lifecycle, value, callback, and typed-exception contracts.
- Include focused offline protocol tests, Docker URSim integration, and ports
  of the 17 upstream CMake examples.
- Preserve documented upstream limitations for partial Primary parsing and
  backend-specific dashboard operations.

Source parity is complete for the declared baseline; physical motion,
hard-real-time behavior, and production deployment remain unvalidated.
