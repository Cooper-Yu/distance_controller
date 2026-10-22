# Completed stopped prefix recovery

`route_prefix_return_runner` is a separate recovery entry. The ordinary
`load_source` and full-route return gates are unchanged. A failed source remains
failed; this entry does not create Task6 acceptance or obstacle-clearance evidence.

Admission is deliberately narrow: the original formal audit must contain a
contiguous nonempty **path-only** completed prefix and a scan source-freshness
failure at its final stopped boundary. Each accepted action retains its original
ready/completed/record evidence. An optional next `route_action_started` scheduler
marker is allowed, but no actual next `started`, partial turn, or later nonzero
command is allowed. The closed bag must cover cancellation, retain monotonic dual
odom timestamps, and end with at least 0.5 seconds of stopped observations. The
watcher must cover the source through the failure snapshot without epoch/gap faults.
The failure snapshot and final filtered pose must match the last completed record
within 5 mm / 0.01 rad. Raw odometry is compared to its own original raw anchor,
not forced to equal the filtered coordinate estimate.

The loader hashes the original book, audit, source configuration/status/watch,
recorder result, metadata and SQLite bag. The runtime checks their filesystem
identity throughout the session and rechecks bytes immediately before its first
nonzero command. It subscribes to raw odometry for the runner lifetime and destroys
that subscription during cleanup. Both original GIDs, `odom` / `base_link` frames,
source and receipt age <= 0.5 s, finite feedback, timestamp order and jump bounds
remain required. An exact duplicate raw sample cannot refresh its receipt; a changed
same-stamp sample faults. Before the first nonzero command, both streams must remain
stopped at their respective original anchors within 5 mm / 0.01 rad. No epoch
calibration, rebase, initialization or implicit resume is supported here.

Each explicit `next` reverses exactly one original path using `ReturnSequence`:
actual stopped point, all original path vertices in reverse order, and the original
previous actual stop. Original P01 remains fixed. Defaults are 0.03 m/s and 0.10
rad/s. The shared original `points.return_claim.json` is created before any motion;
failure/crash/partial quit cannot restart this source through either return entry.
No auto-next option exists. Exit 0 means both reversed paths reached P01; partial
quit/cancellation/failure exits nonzero. The existing unique-controller lock,
feedback stop, final stop checks and cleanup remain in effect.

`--laser-log-only` is explicitly observation only. It cannot certify unknown rear
space or satisfy Task6 laser avoidance. Physical use still requires current path,
laser/camera and endpoint checks by the operator. `--laser-enforced` retains the
existing strict guard unchanged, including its known scan-coverage limitations.

Example (UUID/GIDs must be read from this exact source and verified live):

```bash
ros2 run distance_controller route_prefix_return_runner \
  --source-run-dir /absolute/closed/failed/run \
  --confirm-same-odom SOURCE_UUID \
  --odom-publisher-gid FILTERED_GID --raw-odom-publisher-gid RAW_GID \
  --output /absolute/new/recovery.json --laser-log-only
```

Local validation uses `test/test_route_prefix_return.py` and the bounded localhost
`test/verify_route_prefix_runtime.py` fixture. The latter assembles clearly synthetic
source evidence and runs the actual installed backend with real ROS messages; it is
not a robot or collision validation. The sensor helper lifecycle and source assembly
are explicitly exempted from the statement-count rule because they are ordered
fixture setup/cleanup rather than production responsibilities.
