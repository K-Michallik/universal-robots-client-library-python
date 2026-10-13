# URSim example fixtures

The controller-specific `programs/` directories provide clean persistent
storage for the Docker URSim example matrix. The launcher mounts the matching
directory into the controller image, or the folder given with `-p`.

PolyScope X entries from 10.12.0 on use per-version, per-model program
folders copied byte-for-byte from the upstream URCL 2.15.2 fixtures:

- `programs/polyscopex/10.12.0/ur5e`
- `programs/polyscopex/10.13.0/ur7e`
- `programs/polyscopex/10.14.0/ur7e`

Each holds the PolyScope X application database (`00000000000.xd`, `blobs/`,
and from 10.14.0 the file-system application store under `0/` with its
migration marker) that provides `wait_program`. The `programs/polyscopex`
root remains the empty default for 10.11.0 and ad-hoc runs.

`citadelDB/10.13.0` and `citadelDB/10.14.0` are preconditioned PolyScope X
settings databases, also copied from upstream. Mounted with the launcher's
`-c` option at `/citadelDB`, they make the controller boot in remote control,
which the dashboard example needs for its state-changing commands. Each
controller version needs its own database; the launcher refuses a missing
folder rather than mounting an empty one.

The dashboard example expects `wait_program.urp` on CB3/PolyScope 5 or
`wait_program` on PolyScope X, as does the upstream `C++` example. Controllers
without that optional program report the dashboard example as skipped; the
remaining examples continue to run.

The launcher retains CB3 path and image handling for historical protocol
compatibility, but the validated Python example matrix targets
PolyScope 5/e-Series and PolyScope X.

URSim writes installations, `programs.UR*` links, and PolyScope X database,
blob, and lock state into these mounted directories. In the `programs/polyscopex`
root those files are runtime artifacts and are intentionally ignored. The
per-version PolyScope X folders and the citadel databases are different:
their database files are tracked seeds, so do not regenerate or commit them
after a local run unless you are deliberately refreshing a fixture from
upstream.

## Keeping the working tree clean after local runs

Most runtime artifacts (installations, `programs.UR*`, `*.variables.problem`,
PolyScope X `*.xd`/`*.lck`/`blobs/`/`.id-counters.json` in the
`programs/polyscopex` root, and `*.lck` anywhere below it) are untracked and
covered by [`.gitignore`](../../../.gitignore).

A container writes into the tracked per-version PolyScope X folders and
citadel databases too. Discard those changes after a local run with
`git restore tests/resources/dockerursim/programs/polyscopex tests/resources/dockerursim/citadelDB`
and remove new untracked files there with `git clean -n` followed by
`git clean -f` on the same paths. URSim may create them as root, in which
case removal needs `sudo`.

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

PolyScope X: the backend needs time to finish the initial safety setup
before the examples run. `scripts/wait_for_ursim_ready.py` drives that setup
the same way CI does. Pass the per-version program folder and, for 10.13.0
and newer, the citadel database to boot in remote control:

```bash
bash scripts/start_ursim.sh -m ur7e -v 10.14.0 -r \
  -p tests/resources/dockerursim/programs/polyscopex/10.14.0/ur7e \
  -c tests/resources/dockerursim/citadelDB/10.14.0
python scripts/wait_for_ursim_ready.py --robot-ip 192.168.56.101 --no-steady-rtde
google-chrome --headless=new --user-data-dir=/tmp/polyscopex-chrome \
  http://192.168.56.101 &
python scripts/wait_for_ursim_ready.py --robot-ip 192.168.56.101 --frontend-loaded
python run_examples.py --simulator --robot-ip 192.168.56.101
docker rm -f ursim
```

Without `-c` the controller boots in local control and the dashboard example
sends no state-changing commands.

In remote control the dashboard example loads `wait_program` through the
Robot API. PolyScope X serves the program endpoints through its web frontend,
so they hang until the client times out (`PUT /program/v1/loaded timed out`)
unless a UI session is open: a headless browser, as above, or a browser tab
on `http://192.168.56.101`. Open it only after the first readiness check has
confirmed safety. `--frontend-loaded` then waits until the starting UI has
loaded its last program: until then it can power the robot off and abort an
example's external-control program.
Stop the browser before removing the container.

The readiness script ends by waiting until the RTDE stream has gone 10 s
without a gap over 100 ms, and continues with a warning after 120 s. For
several seconds after the robot becomes controllable, PolyScope X is still
starting URCap containers, and on a loaded host the realtime path stalls long
enough to trip a servo example's 200 ms receive timeout. That wait runs after
the `--frontend-loaded` wait, since the starting UI can stall the stream
again. The first call above therefore skips it with `--no-steady-rtde`. In
local control, one call without flags is enough.

Version- and prerequisite-gated examples report as skipped rather than failing
(for example the dashboard example when `wait_program`/`wait_program.urp` is not
installed). PolyScope X timing is sensitive to host load, so reproduce any CI
failure locally before treating it as a library regression.
