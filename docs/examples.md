# Python examples

The `examples` package ports the 17 executables built by the upstream URCL
2.14.0 `examples/CMakeLists.txt`. The ports preserve protocol behavior while
using Python lifecycle, naming, exception, and callback conventions.

## Running an example

Install the project in editable mode and run examples as modules from the
repository root:

```bash
python3 -m pip install -e ".[dev]"
python3 -m examples.rtde_client --robot-ip 192.168.56.101 --duration 10
python3 -m examples.full_driver --robot-ip 192.168.56.101 --duration 10
```

Zero duration means run until interrupted. Examples that change robot state or
command motion require an interactive confirmation. `--yes` bypasses that
confirmation for a deliberate physical-robot run. `--simulator` declares that
the target is URSim and bypasses physical confirmation.

Most external-control examples stop the current program, may clear a
protective stop, release the brakes, and start the bundled headless URScript.
PolyScope 5 and PolyScope X controllers must permit remote operation. Pure
CPython does not provide `C++` hard-real-time guarantees at 125 Hz or 500 Hz.

The supported example and URSim validation scope starts with PolyScope 5
(e-Series) and includes PolyScope X. CB3 is intentionally excluded because
that controller generation is no longer supported by this project. Historical
CB3-compatible behavior retained in the translated library is not an example
support or validation claim.

## Upstream mapping

Protocol and controller-state examples:

- `dashboard_example.cpp` → `examples.dashboard_example`: selects the G5 or
  PolyScope X backend, cycles power and program state, and demonstrates raw G5
  requests. It requires `wait_program.urp` on PolyScope 5 or `wait_program`
  on PolyScope X.
- `primary_pipeline.cpp` → `examples.primary_pipeline`: composes the Primary
  stream, parser, producer, and pipeline. A local printing consumer replaces
  the deliberately omitted `C++` `ShellConsumer`.
- `primary_pipeline_calibration.cpp` →
  `examples.primary_pipeline_calibration`: waits for and reports kinematics
  calibration status.
- `rtde_client.cpp` → `examples.rtde_client`: reads RTDE state and writes the
  speed slider through a standalone RTDE client.
- `send_script.cpp` → `examples.send_script`: sends blocking and non-blocking
  URScript through the Primary interface.
- `script_sender.cpp` → `examples.script_sender`: serves a fixed URScript on
  TCP port 12345.

External-control and motion examples:

- `full_driver.cpp` → `examples.full_driver`
- `spline_example.cpp` → `examples.spline_example`
- `tool_contact_example.cpp` → `examples.tool_contact_example`
- `freedrive_example.cpp` → `examples.freedrive_example`
- `force_mode_example.cpp` → `examples.force_mode_example`
- `script_command_interface.cpp` → `examples.script_command_interface`
- `direct_torque_control.cpp` → `examples.direct_torque_control`
- `trajectory_point_interface.cpp` →
  `examples.trajectory_point_interface`
- `trajectory_streaming.cpp` → `examples.trajectory_streaming`
- `instruction_executor.cpp` → `examples.instruction_executor`
- `external_fts_through_rtde.cpp` →
  `examples.external_fts_through_rtde`

The spline port supplies explicit velocities because the upstream
positions-only overload is not executable. Trajectory streaming composes the
Python `TrajectoryStream` with the canonical `UrDriver` trajectory methods.
Controller-dependent friction, Optimove, torque, dashboard, and external-FT
features exit as explicit skips on unsupported versions.

Important version gates are:

- PolyScope X dashboard state control: 10.12.0 or newer, and only in
  remote control; in local control the dashboard example reads the control
  mode, reports it, and sends no state-changing commands
- PolyScope X popup, system-log, shutdown, flight-report, support-file, and
  robot-information (PolyScope version, model, serial number) dashboard
  commands: upstream URCL 2.15 gates them on PolyScope X 10.14.0
  (Robot API 5.0.107). Popups, log entries, shutdown, and flight reports
  additionally require remote control. The Python PolyScope X backend does
  not implement these commands yet and raises `NotImplementedError` for them
- instruction example: UR5-family geometry, with Optimove requiring
  5.21.0 / 10.8.0 or newer
- friction scales and direct torque: 5.25.1 / 10.12.1 or newer
- external FT input: supported PolyScope 5/X controllers
- tool-contact and script-command demonstrations: e-Series/PolyScope X

Cleanup is best effort. The dashboard example stops its program and powers off
after failures; force/tool modes are ended before external control closes; the
script-command examples disable tool contact and tool voltage and return
friction scales to `1.0`. The script-command tour cannot snapshot the prior TCP
or payload through this API, so reapply application-specific TCP/payload
configuration after running it on hardware.

## Bounded URSim runner

Bulk execution is restricted to a declared simulator:

```bash
python3 run_examples.py \
  --simulator \
  --robot-ip 192.168.56.101
```

The runner executes all official examples sequentially, waits between
reconnects, continues past explicit version/prerequisite skips, and stops at
the first failure. Each manifest entry has a duration long enough to reach its
demonstrated behavior, and every child also has a hard wall timeout with an
interrupt-first cleanup path. By default the runner verifies that the target IP
belongs to a local `universalrobots/ursim_*` Docker container. A deliberately
managed remote simulator additionally requires
`--allow-unverified-simulator`.

Force mode runs before examples that can leave the shared simulator in a
singular configuration for its fixed task frame. This prevents one example's
terminal pose from invalidating the next example.

`scripts/start_ursim.sh` launches representative Docker controllers. The
repository's `.github/workflows/examples.yml` workflow runs offline checks on
every push and pull request, and a separate PolyScope 5/e-Series and
PolyScope X integration matrix weekly and on demand. The matrix costs about 50
runner-minutes per run, so trigger it for a branch only when a change needs it:
`gh workflow run examples.yml --ref <branch>`. Documentation-only changes skip
the workflow, and a newer push cancels the older run. Before
starting examples on PolyScope X, CI waits until the robot is controllable,
until the web frontend has finished starting in remote-control entries, and
then until the RTDE stream has been free of realtime stalls for 10 s.
PolyScope X entries remain visible but non-blocking because
their timing is sensitive to shared-runner load; reproduce failures locally
before treating them as library regressions. CB3 is not part of that matrix.
Local and CI matrix runs can produce expected skips for controller-version,
robot-geometry, remote-mode, and program prerequisites.

The PolyScope X entries mirror the upstream URCL 2.15.2 CI matrix:

| Controller | Model | Program folder (`-p`) | Citadel DB (`-c`) | Control mode |
| --- | --- | --- | --- | --- |
| 10.11.0 | `ur5e` | `programs/polyscopex` | none | local |
| 10.12.0 | `ur5e` | `programs/polyscopex/10.12.0/ur5e` | none | local |
| 10.13.0 | `ur7e` | `programs/polyscopex/10.13.0/ur7e` | `citadelDB/10.13.0` | remote |
| 10.14.0 | `ur7e` | `programs/polyscopex/10.14.0/ur7e` | `citadelDB/10.14.0` | remote |

Paths are relative to `tests/resources/dockerursim/`. PolyScope X 10.13.0
and newer persists its settings in a citadel database; the launcher's `-c`
option mounts a preconditioned copy at `/citadelDB` so the controller boots
in remote control. The workflow confirms the control mode before running the
examples, and the dashboard example then exercises its state-changing path.
Whether every other example behaves the same in remote control has not been
verified yet, which is one more reason these entries stay advisory.

URSim validates connection, negotiation, command serialization, lifecycle, and
cleanup. It does not validate physical torque response, contact detection,
force behavior, collision clearance, or timing guarantees. Those behaviors
still require a separately supervised physical-robot validation pass.
