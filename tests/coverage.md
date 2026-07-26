# Validation scope

This is a behavioral validation ledger, not a measured line-coverage report.

## Automated offline tests

- Semantic `Q`, `Pose`, and motion-target normalization
- Fixed-length numeric validation
- Boolean parsing, string splitting, robot type/series conversion
- `C++` half-away-from-zero rounding
- 64-bit libstdc++ string hashing and calibration-hash construction
- Version parsing, ordering, formatting, and validation
- Robot receive-timeout control-mode bounds
- TCP framing, exact reads/writes, retry, pipeline, and queue behavior
- RTDE package parsing, negotiation, recipe setup, client state, and writer
- Typed RTDE scalar/vector reads, missing-key mapping semantics, polling reads,
  and timeout-free blocking reads
- Primary package parsing, strict/lenient fallback, snapshots, commands, and
  blocking script feedback
- Reverse, trajectory, script-command, ScriptReader, ScriptSender, and tool
  communication wire protocols
- G5 dashboard TCP and PolyScope X HTTP transports
- `UrDriver`, `InstructionExecutor`, and `ExampleRobotWrapper` composition,
  typed result paths, lifecycle, cleanup, callback, and default-factory
  behavior
- Deterministically synchronized bounded-pipeline overflow and cleanup
- Failed high-level connection rollback and retry
- Exact 17-example manifest, import-safe uniform CLIs, bundled recipe lookup,
  physical-run safety refusal, simulator skip handling, and runner failure
  propagation
- Pure spline, quintic-stream, and deterministic external-FT command
  generation used by the runnable examples
- Docker-target verification, per-example duration metadata, and dry-run
  controller/model normalization for the URSim launcher

## Docker URSim integration

The GitHub workflow and local Docker runner exercise bounded execution across
the configured PolyScope 5/e-Series and PolyScope X matrix. PolyScope 5 jobs
remain required; timing-sensitive PolyScope X jobs are reported but
non-blocking and should be reproduced locally before triage. Known
controller-version, geometry, remote-mode, and program prerequisites produce
explicit skips. These jobs require Docker and are separate from the offline
pytest suite.

## Manual hardware validation

One out-of-band PolyScope 5 smoke test established a connection and read RTDE
data without sending a motion command. It is not stored as an automated test,
does not run in CI, and does not validate motion or timing.

## Deferred

- Physical-robot motion validation
- Timing/throughput benchmarks at 125 Hz and 500 Hz
- Platform matrix outside the active Linux/Python environment
- Exhaustive copies of every upstream `C++` unit test
- PolyScope X integration requiring authenticated Robot API access
- End-to-end external-control URCap/URCapX tests

## Out of scope

- CB3 example validation; that controller generation is no longer supported
