# URSim example fixtures

The controller-specific `programs/` directories provide clean persistent
storage for the Docker URSim example matrix. The launcher mounts the matching
directory into the controller image.

The dashboard example expects `wait_program.urp` on CB3/PolyScope 5 or
`wait_program` on PolyScope X, as does the upstream `C++` example. Controllers
without that optional program report the dashboard example as skipped; the
remaining examples continue to run.

The launcher retains CB3 path and image handling for historical protocol
compatibility, but the validated Python example matrix targets
PolyScope 5/e-Series and PolyScope X.

URSim writes installations, `programs.UR*` links, and PolyScope X database,
blob, and lock state into these mounted directories. Those files are runtime
artifacts and are intentionally ignored; only the seed variable files and
empty-directory markers belong in source control.

## Keeping the working tree clean after local runs

Most runtime artifacts (installations, `programs.UR*`, `*.variables.problem`,
PolyScope X `*.xd`/`*.lck`/`blobs/`/`.id-counters.json`) are untracked and
covered by [`.gitignore`](../../../.gitignore).

A few *tracked* seed files are still mutated in place when a container mounts
them (the e-Series `default.variables` gains a timestamp, and file modes may
flip to executable). `.gitignore` cannot suppress changes to tracked files, so
if the churn is distracting, mark those seeds `skip-worktree` once per clone:

```bash
git update-index --skip-worktree \
  tests/resources/dockerursim/programs/cb3/default.variables \
  tests/resources/dockerursim/programs/e-series/default.variables \
  tests/resources/dockerursim/programs/polyscopex/.allow-empty
```

Reverse it with `git update-index --no-skip-worktree <file>` if you ever need
to update the seeds themselves.

## Running the checks locally

Run every command below from the repository root
(`universal-robots-client-library-python/`) inside the project virtualenv:

```bash
# A globally sourced ROS environment (e.g. /opt/ros/*) puts its site-packages
# on PYTHONPATH, which makes pytest autoload unrelated plugins such as
# launch_testing and fail with "ModuleNotFoundError: No module named 'yaml'".
# Clear it for the current shell if that applies to you.
unset PYTHONPATH

python -m pip install -e ".[dev]"
```

### Offline suite

These are the same checks the `offline` CI job runs. No robot, simulator, or
Docker is required:

```bash
pytest
ruff check src tests examples run_examples.py
mypy
```

### Docker URSim integration

These exercise the runnable examples against a live simulator and require
Docker. Invoke the launcher with `bash` so its tracked file mode is not
changed. The `-r` flag replaces any existing `ursim` container.

PolyScope 5 / e-Series (no browser needed):

```bash
bash scripts/start_ursim.sh -m ur5e -v 5.9.4 -r
python run_examples.py --simulator --robot-ip 192.168.56.101
docker rm -f ursim
```

Swap in a newer controller (for example `-v 5.25`) to reach the version-gated
examples such as `direct_torque_control`.

PolyScope X: the browser-backed services must be initialized before the
examples run, and the backend needs time to finish the initial safety setup:

```bash
bash scripts/start_ursim.sh -m ur5e -v 10.13.0 -r
google-chrome --no-sandbox --disable-setuid-sandbox --headless=new 192.168.56.101 &
sleep 60
python run_examples.py --simulator --robot-ip 192.168.56.101
docker rm -f ursim
```

Version- and prerequisite-gated examples report as skipped rather than failing
(for example the dashboard example when `wait_program`/`wait_program.urp` is not
installed). PolyScope X timing is sensitive to host load, so reproduce any CI
failure locally before treating it as a library regression.
