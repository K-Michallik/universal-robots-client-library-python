---
name: port-upstream-urcl-module
description: Ports active C++ URCL module behavior into the mirrored Python package with Pythonic lifecycle contracts, protocol tests, and parity documentation. Use when implementing or updating RTDE, Primary, communication, control, dashboard, driver, executor, resource, or example functionality from upstream.
---

# Port an Upstream URCL Module

## Establish the source contract

1. Confirm that the upstream revision being used matches the Python repository's
   recorded baseline. If upstream moved, use `sync-upstream-urcl` first.
2. Locate the ledger entry in `docs/parity.md`.
3. Read the complete matching C++ source and implementation-bearing headers.
   Also inspect enums, serializers, resources, tests, examples, and call sites
   that define observable behavior.
4. Separate active supported behavior from compatibility-only deprecated APIs.
   Port behavior, not unused C++ overloads or ownership shapes.

## Implement in the mirrored Python area

- Place code under the corresponding `src/ur_client_library/comm`, `rtde`,
  `primary`, `control`, or `ur` subpackage.
- Follow `docs/api-mapping.md` and the project lifecycle rule: lightweight
  constructors, explicit connect/start/close, typed returns and exceptions,
  immutable returned vectors, and unsubscribe callables.
- Preserve wire formats, bounds, retry/timeout meanings, state transitions,
  callbacks, version gates, and cleanup ordering exactly.
- Reuse existing validators, serializers, transports, protocols, and exception
  classes before adding abstractions or dependencies.
- Keep user callbacks outside internal locks and retain worker failures for
  public operations.
- For trajectory changes, update serialization and
  `resources/external_control.urscript` as one protocol revision.
- For RTDE reference behavior, independently adapt only what is needed and
  preserve BSD attribution; do not add the sibling RTDE repository as a
  dependency.
- Preserve explicit G5/PolyScope X backend differences and unsupported
  operations.

## Prove parity

1. Add focused offline tests near the existing test module for the protocol
   area. Prefer deterministic byte vectors, injectable transports, and local
   fake servers over timing-dependent integration.
2. Cover success, invalid input, timeout/disconnect, lifecycle, version-gate,
   and cleanup behavior applicable to the change.
3. Update `docs/parity.md`. Update `docs/api-mapping.md`,
   `docs/examples.md`, `tests/coverage.md`, and
   `docs/lifecycle-and-safety.md` when their claims change.
4. Run `validate-urcl-python`.
5. Run the smallest relevant `run-ursim-integration` entry when the change
   affects controller negotiation, commands, resources, lifecycle, or
   examples. Record expected skips separately.

Do not advance the advertised upstream baseline until all affected active
behavior has been reconciled and the applicable validation layers pass.
