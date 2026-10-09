---
name: sync-upstream-urcl
description: Compares an updated C++ Universal Robots Client Library with the Python port, triages behavior changes, and plans or performs an aligned baseline update. Use when upstream URCL is pulled, released, updated, or suspected to have diverged from this Python implementation.
---

# Synchronize with Upstream URCL

## Alignment policy

The recorded URCL 2.15.2 baseline is a checkpoint, not a permanent fork.
Whenever the C++ Client Library changes, assess the delta and keep the Python
port aligned with active supported functionality. Do not change only version
text, and do not claim a new parity baseline before code, resources, tests, and
ledgers agree.

## Discover the actual delta

1. Work from the parent workspace containing:
   - `Universal_Robots_Client_Library/`;
   - `universal-robots-client-library-python/`.
2. Read the Python baseline from `README.md`, `docs/parity.md`, and
   `CHANGELOG.md`.
3. Determine the C++ repository's actual version and revision from
   `package.xml`, git metadata, and `CHANGELOG.rst`. Do not infer a version from
   user wording.
4. Compare the recorded baseline with upstream `src/`,
   `include/ur_client_library/`, `resources/`, `examples/`, package metadata,
   and changelog. Include inline/header-only behavior.
5. Triage independent areas in parallel where useful:
   - communication and RTDE;
   - Primary parsing and script feedback;
   - reverse, trajectory, script-command, and tool protocols;
   - dashboard backends;
   - high-level driver, executor, wrapper, resources, and examples.

## Classify changes

For each upstream change, record:

- Python destination and current parity-ledger entry;
- active behavior, compatibility-only deprecation, or unsupported upstream
  surface;
- wire-breaking versus internal/API-only impact;
- version gates, retry/timeout semantics, and lifecycle impact;
- serializer or bundled URScript coupling;
- required offline tests, URSim coverage, and documentation updates.

Follow executed C++ behavior when documentation disagrees. Retain deliberate
Pythonic API shaping unless upstream behavior requires a public change.

## Align the Python port

When implementation is requested:

1. Port coherent behavior groups rather than isolated symbols.
2. Update protocol resources and serializers together.
3. Add or update focused tests and affected examples.
4. Update `docs/parity.md` while work proceeds.
5. After parity and validation complete, update the baseline in `README.md`,
   `CHANGELOG.md`, and other status documents.
6. Run `validate-urcl-python`; run applicable URSim entries when integration
   behavior changed.

If the request is analysis-only, return the classified delta and an
implementation sequence without editing files.
