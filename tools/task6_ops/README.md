# Task6 read-only recording and odometry monitoring

These repository tools replace pasted `/tmp` preparation scripts. They do not publish velocity,
reset odometry, change P01, or authorize motion. Run from the Git checkout after reviewing the
revision. ROS Humble (`rclpy`, `rosbag2_py` and standard messages) is required; no build is needed.

## Start and stop

```bash
bash tools/task6_ops/run.bash record --seconds 1800 --image-hz 2 --max-mib 180
bash tools/task6_ops/run.bash watch --output ~/odom_watch_NEW.jsonl \
  --seconds 1800 --max-bytes 4000000 --base-topic /rosbot_xl_base_controller/odom
```

The wrapper sources `/opt/ros/humble/setup.bash`. Set `TASK6_OVERLAY` to an existing overlay setup
file only if needed. The watcher output parent must exist and both output files must be new.
For background operation use `nohup bash ... </dev/null >unique.log 2>&1 &` and retain that PID.
Stop only that PID with `kill -INT "$pid"`; do not kill by broad process-name pattern. Processes
also stop at their duration limits. SIGINT/SIGTERM write final evidence and release their lock.
A separate per-user/tool/ROS-domain `flock` rejects concurrent instances even with another
output directory. A stale lock file is harmless: ownership is held by the live file descriptor.

Recorder defaults remain 900 seconds, all image frames (`--image-hz 0`), and no bag-size cap
(`--max-mib 0`). Duration is bounded to 1800 seconds. Both tools require 300 MiB filesystem free
space. The 180 MiB bag limit is sampled by filesystem stat once per second and may overshoot
by one second of data plus close overhead; cloud quotas may differ from filesystem free space.
Watcher JSONL has an explicit byte cap; its small final summary is additional output.

## Evidence contract

Recorder root defaults to `~/task6_writer_evidence`. `latest_session.txt` points to a new
`timestamp_PID/` directory containing `manifest.json`, `supervisor.pid`, `health.json`,
`bag/`, and eventually `result.json`. `health.json` is atomically replaced every five seconds.
Before movement an orchestrator must check the process is alive, health is fresh, and scan and
odom counts increase. Opening a bag or printing RECORDING is not readiness. Missing image/TF
counts must be assessed explicitly. STOP file in the session, SIGINT/TERM, duration, disk floor,
and bag limit all close the writer. `result.json.closed_metadata == true` and no exception are
closure evidence, not route completion. Capacity stopping recording never permits further
unrecorded motion; check recorder health again before subsequent actions.

The ten subscriptions are `/scan_filtered`, `/scan`, `/tf`, `/tf_static`, `/odometry/filtered`,
`/rosbot_xl_base_controller/odom`, `/cmd_vel`, `/rosout`, `/image_raw/compressed`, and
`/imu_broadcaster/imu`. Only compressed images are optionally rate-limited. Source serialized
bytes and message headers are retained; bag timestamps are local callback receipt times.
`/tf_static` uses reliable/transient-local subscription. Other subscriptions use best-effort.
Writing and counters run on one callback thread. **Never query SQLite, extract images, or open
an extra bag reader while recording.** Read closed bags only after checking the process exited
and metadata exists. Prior live COUNT queries created a lock-contention risk.

The watcher writes JSONL and a final `.summary.json`: odom jumps, stamps, frames, publisher GIDs,
command activity and selected rosout messages. Idle time is a lower bound if no nonzero command
was observed. Temporal association is not proof of why an odom reset occurred. The watcher
observes feedback; it neither stops a controller nor replaces the controller watchdog.

## Verification and boundaries

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest -q test/test_task6_ops.py test/test_task6_ops_watch.py
ruff check tools/task6_ops test/test_task6_ops.py test/test_task6_ops_watch.py
bash -n tools/task6_ops/run.bash
```

Tests use isolated ROS domain 223 and loopback-only communication, never the robot. They cover
single-instance ownership, watcher limits/events, non-TTY recorder closure, image throttling,
latched TF and closed metadata. Runtime fixture code lives under `test/task6_ops_fixture.py`.
No camera viewpoint or no-contact claim follows from image counts. Scope is operational support,
not learner implementation acceptance or complete Task6 route validation. The writer main
lifecycle retains a scoped branch/statement-count exemption to keep signal/cleanup ownership
visible; no repository-wide style limits are disabled.
## Conditional odometry-epoch transfer (local verification only)

A publisher restart invalidates an old odom-frame return. `scan_compare.py` is a read-only,
bounded diagnostic: it compares the final stopped scan window of a **closed** formal session
with ten fresh scans, unchanged laser mounting, fresh odometry and a continuously running
schema-2 watcher. It requires two nonparallel wall directions and rejects ambiguous local fits.
It publishes no commands. Run the current repository watcher before capture:

```bash
source /opt/ros/humble/setup.bash
python3 tools/task6_ops/scan_compare.py \
  --session /path/to/closed/formal/session \
  --watch-log /path/to/live/schema2_watch.jsonl
```

The session must contain `points.json`, its completed `points.jsonl`, the closed recorder
`result.json`, and `bag/`. The diagnostic writes a new report and point-cloud artifact; it does
not modify the original session. A successful local fit is conditional evidence of the same
nearby physical pose, not proof of global localization or collision-free return. Earlier
schema-1 diagnostic reports cannot authorize this transfer.

The optional return runner input is `--calibration /path/to/report.json` together with
`--confirm-source-session ORIGINAL_UUID` and `--odom-publisher-gid NEW_GID`. Existing same-epoch usage
continues to require `--confirm-same-odom ORIGINAL_UUID`. Do not combine the two confirmations.
The runner remains supervised: attachment performs no route movement; `next` starts one
existing reversed action. This feature has **not passed a real-robot epoch-return trial**.

Acceptance binds the original book/audit hash, UUID and completed-event time, the reference
scan's old odometry pose, the captured new GID/pose/stamp, and the live watcher's process identity
and immutable log prefix. Until the first action it rejects expired capture (>120 seconds),
missing watcher coverage, changed identity, invalid feedback, nonzero commands or more than
2 mm / 0.2 degree observed movement. All original feedback-loss and single-controller guards
remain active during movement. It writes a separate `.calibration.json` evidence artifact;
the original source and its single-use return claim remain authoritative.

The transform is `T_new_old = T_new_current * inverse(T_old_reference * T_reference_current)`.
It maps all old absolute poses together, preserves continuous yaw and leaves P01-relative
route points unchanged. It does not create a new physical origin. The reference scan pose must
match the source's recorded last stop within 5 mm / 0.01 rad. Registration residual/overlap and
local-motion thresholds are rejection gates, not a claimed absolute positioning accuracy.
Humble publisher identity is sampled from the ROS graph; this is not a per-message transport
identity guarantee. Sparse or repeating geometry and incomplete laser coverage remain limits.

Reference selection is tied to the original `route_completed` wall time: bag messages
must lie between 0.5 s before and 3 s after that boundary, and scan source timestamps
must remain within -0.1/+3.5 s of the original final odom stamp. A later closed-bag tail
with the same numeric pose cannot substitute for this window. The report binds the
source hash and completion boundary, session directory, closed result, metadata and
SQLite file hashes; attachment rechecks those bytes once. Missing evidence rejects
calibration rather than choosing a different window.

The selected reference medoid scan uses its nearest valid odom sample within 50 ms,
instead of the final odom callback. The current medoid likewise uses its paired pose.
This reduces a time-association error; it does not eliminate sensor/odometry error or
establish absolute accuracy. Schema-2 observation now explicitly rejects bad odom
quaternions, duplicate/backward timestamps and wall-clock source ages outside
-0.1/+0.5 s. The calibration path is intended for the real robot, not simulated time.
At the first actual nonzero command the runner rechecks expiry, watcher and pose after
any preparation/guard delay; a zero or rejected command cannot end that validation.
Watcher snapshots are spaced by 2 s and may be up to 3 s old; callback events and
interval extrema supplement them, while the runner checks fresh current pose before
release. These remain sampled observations, not proof of physically continuous
immobility or identity between every transport message.

```bash
python3 -m pytest -q test/test_route_return_calibration.py \
  test/test_route_return_reference.py test/test_task6_calibration_watch.py \
  test/test_route_return_calibration_first_command.py \
  test/test_route_return_calibration_runtime.py test/test_route_return.py \
  test/test_task6_ops.py test/test_task6_ops_watch.py
```

The mathematical tests cover transform direction, immutable route points and rejection gates.
A separate isolated ROS fixture verified actual schema-2 observation and production runner
attach/quit without action execution. Neither test certifies real-robot recovery.
