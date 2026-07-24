# Future improvements

These items are intentionally deferred to preserve URCL 2.14.0 behavior during
the first Python translation.

## Share the Primary connection in ExampleRobotWrapper

The upstream `ExampleRobotWrapper` creates a `PrimaryClient` to inspect and
initialize the robot before it constructs `UrDriver`. `UrDriver` then creates
and owns a second `PrimaryClient`. The Python implementation preserves those
two connections for equivalent startup ordering, reconnect behavior, and
lifecycle ownership.

A future major-version change may inject the wrapper's Primary client into
`UrDriver` so the two objects share one connection and worker pipeline. That
change requires:

- an explicit ownership contract for closing and reconnecting the shared
  client;
- preserving the wrapper's pre-driver protective-stop and brake-release
  sequence;
- defining how worker failures and subscriptions are shared;
- integration tests for startup, reconnect, local/remote mode transitions, and
  shutdown;
- a documented compatibility change because connection count and failure
  ordering are externally observable.

## Complete the public typing sweep

The integration cleanup gives RTDE values and the high-level driver, wrapper,
and instruction paths precise contracts. It deliberately does not redesign the
dynamic Dashboard facade or the extensible Primary transport/consumer seams.
A full package sweep should be staged as a separate compatibility project.

### Dashboard facade and backend configuration

- Replace dynamic `__getattr__` command forwarding with an explicit,
  statically typed facade for the common G5 and PolyScope X operations.
- Model policy/backend construction with typed configuration objects instead
  of open-ended argument forwarding.
- Keep backend-specific unsupported operations explicit. PolyScope X methods
  that are absent upstream must continue to raise `NotImplementedError`.
- Decide whether backend-only operations belong on separate typed views rather
  than widening the common facade.

This affects attribute discovery and may expose existing backend signature
differences, so it requires a deprecation window rather than a mechanical
annotation-only change.

### Primary transport and consumer contracts

- Replace the remaining `object`-based transport extension points with narrow
  `Protocol`s for connect/read/write/close behavior.
- Define a typed consumer/snapshot protocol for parsed Primary packages while
  retaining lenient raw-package fallback.
- Separate public extension contracts from private compatibility adapters so
  transport errors and lifecycle guarantees remain clear.

Stronger protocols will reject some currently accepted duck-typed extensions
at static-check time and may reveal runtime implementations that rely on
alternate method names.

### Aliases, deprecations, and contract fixtures

- Inventory legacy `C++`-style aliases and alternate Python spellings.
- Select one canonical name and signature for each public operation.
- Preserve approved aliases as documented wrappers for at least one minor
  release, emit deprecation warnings where appropriate, and remove them only
  in a major release.
- Add strict static contract fixtures for public imports, factory callables,
  callbacks, overloads, and representative third-party fakes. Run those
  fixtures on every supported Python version in CI.

### Staged migration

1. Publish the intended protocols and a compatibility inventory without
   changing runtime dispatch.
2. Add explicit Dashboard methods and typed Primary extension seams while
   retaining documented aliases.
3. Migrate internal callers, examples, tests, and downstream integration
   fixtures to canonical APIs.
4. Measure warnings and compatibility failures across G5, PolyScope X, URSim,
   and physical-controller tests.
5. Remove deprecated aliases and broad adapters only in a version that permits
   the associated breaking changes.

The principal risks are rejecting valid third-party duck types, changing
attribute lookup or exception timing, and accidentally implying structured
support for Primary packages that intentionally remain raw.
