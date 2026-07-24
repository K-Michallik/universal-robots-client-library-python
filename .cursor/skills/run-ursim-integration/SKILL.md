---
name: run-ursim-integration
description: Runs bounded Python URCL examples against Docker URSim with safe startup, PolyScope X initialization, log capture, and cleanup. Use when asked for URSim tests, simulator integration, the example matrix, or local reproduction of the Docker CI job.
---

# Run URSim Integration

## Safety boundary

This workflow is only for a declared simulator. Always pass `--simulator` to
the bulk runner. Never redirect it to a physical controller or bypass physical
confirmation.

## Workflow

1. Work from the repository root and read the current `ursim` matrix from
   `.github/workflows/examples.yml`. Do not rely on a copied version list.
2. Check existing terminals, Docker containers, and the `ursim_net` network
   before starting anything. Do not duplicate a running simulator.
3. If the user asked for the complete validation sequence, run the
   `validate-urcl-python` workflow first. Ensure the package itself is
   installed with:

   ```bash
   .venv/bin/python -m pip install -e .
   ```

4. Run one matrix entry at a time:

   ```bash
   bash scripts/start_ursim.sh -m MODEL -v VERSION
   ```

   The launcher creates the network, starts the fixed-IP container, waits for
   the Primary port, and allows controller stabilization. Use `-r` only when
   the user explicitly intends to replace an existing `ursim` container.

5. For PolyScope X (`10.*`), initialize the web application before running
   examples. Follow the current CI browser command when Chrome is available,
   or ask the user to open `http://192.168.56.101`.
6. Run the bounded examples:

   ```bash
   .venv/bin/python run_examples.py \
     --simulator \
     --robot-ip 192.168.56.101
   ```

   Use `--only`, `--duration`, or `--process-timeout` only for a requested
   focused diagnostic. Exit code 77 is an explicit prerequisite/version skip,
   not an integration failure.

7. On failure, capture `docker logs ursim` before cleanup and report the
   failing example separately from expected skips.
8. Remove only a container started by this workflow:

   ```bash
   docker rm -f ursim
   ```

   Cleanup must run after both success and failure. For a full matrix, repeat
   startup, execution, log capture, and cleanup for each entry.

## Fixture hygiene

URSim writes root-owned links, installations, databases, locks, and blobs into
bind-mounted fixture directories. They are runtime artifacts: do not stage or
commit them, and do not delete intentional seed variables or directory
markers.
