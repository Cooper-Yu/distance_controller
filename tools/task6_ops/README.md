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