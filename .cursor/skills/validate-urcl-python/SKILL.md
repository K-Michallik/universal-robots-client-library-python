---
name: validate-urcl-python
description: Validates the Python URCL repository with offline tests, Ruff, strict mypy, and conditional wheel resource checks. Use when asked to test, validate, reproduce CI, prepare a change, or confirm that implementation work is complete.
---

# Validate Python URCL

## Scope

This workflow is offline. It does not start URSim or contact a physical robot.
Report Docker integration and manual hardware validation separately.

## Workflow

1. Work from the repository containing `pyproject.toml`,
   `src/ur_client_library/`, and `run_examples.py`.
2. Use the repository `.venv`. If it is missing or stale, create it and run:

   ```bash
   .venv/bin/python -m pip install -e ".[dev]"
   ```

3. Run the offline gates:

   ```bash
   PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
   .venv/bin/ruff check src tests examples run_examples.py
   .venv/bin/mypy
   ```

   The pytest environment variable prevents unrelated ROS
   `launch_testing` plugins from contaminating local runs. Run independent
   gates concurrently when useful.

4. If packaging metadata, dependencies, `py.typed`, or package resources
   changed, also:
   - build sdist and wheel with `python -m build`;
   - install the wheel into a fresh temporary virtual environment, not
     editable mode;
   - import `ur_client_library`;
   - assert that `importlib.resources.files("ur_client_library")` contains
     `py.typed` and `resources/external_control.urscript`.

5. If a gate fails, diagnose and fix only within the requested scope. Re-run
   the failed gate, then all applicable gates before declaring success.

## Report

State separately:

- offline pytest result and test count;
- Ruff result;
- strict mypy result and checked-file count;
- package build and clean-install smoke result, when applicable;
- any validation layer not run.
