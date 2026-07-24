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
