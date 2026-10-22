# Supervised current-frame backward step

This is an explicit manual remote-control tool, not automatic route recovery.
It does not know or restore historical P01. The operator must watch the robot and
approve each invocation independently. There is no stdin command queue, route file,
automatic next step or resume. Each new output records `LOCAL_START` and `LOCAL_END`.

The required mode is either:

- `--supervised-manual`: execute one backward target, with fresh laser observations
  and shadow strict-guard diagnostics. Unknown/blocked geometric predictions are
  logged, **not** interpreted as free space and **not** automatically enforced.
  This separate entry never disables guards on a production route. It is not
  Task6 obstacle-avoidance acceptance. Camera supervision and stopping distance
  remain the operator's responsibility; manual operation can still hit an obstacle.
- `--strict-dry-run`: capture fresh stopped input and report the strict decision;
  never create a motion publisher or execute a step. An `unknown` report is still
  a successful diagnostic process, not movement permission.

`--distance` is positive backward meters, maximum 0.05 m. `--speed` is 0.01–0.03 m/s;
all actual XY commands are norm-limited to this speed. Heading/lateral corrections
use the existing Console feedback (angular cap 0.10 rad/s). A small bounded forward
settling correction is possible after overshoot; this is not an available forward
operator command. Current-frame displacement may not exceed requested distance
plus 1 cm, accumulated travel may not exceed distance plus 2 cm, and heading drift
may not exceed 0.01 rad. These are additional conservative local bounds, not an
absolute positioning accuracy claim. The motion budget is 10 monotonic seconds;
existing final stopped-pose checks and cancellation remain active.

Both current odometry GIDs are explicit inputs. Both streams must use `odom` /
`base_link`, finite poses/velocities, valid quaternions, monotonic stamps and fresh
source/receipt ages <=0.5 s. Duplicate identical samples cannot renew the guard's
receipt age; changed duplicates fault. A fresh 0.5-second stopped hold establishes
LOCAL_START. Until the first actual nonzero command, both streams must remain
stopped within 5 mm / 0.01 rad of their own captured pose. A new epoch requires a
new invocation and inspection; there is no automatic epoch adoption while running.

Even manual mode requires each command's scan source/receipt age <=0.5 s, valid
mounting TF, and bounded scan/odom pairing. `RouteLaserGuard.snapshot()` reprojects
all scan-time points into the current base frame on every assessment; cached scan
geometry does not freeze the current pose. No NaN/Inf is filled or close point
deleted here. A missing or stale scan stops motion; a fresh partially missing scan
can be observed in manual mode only. Existing planar body geometry is not a
complete physical robot/attachment envelope, and scans cannot see all heights.

The common process lock and publisher checks prevent concurrent owned control.
Cancellation, timeout, stale feedback, identity change and exceptions use existing
Console zero publication and cleanup; subscription destruction cannot skip runner
cleanup. New process/output is required for another step. Success requires exit 0
and `local_step_completed`; `strict_dry_run` is a separate nonmotion result.

```bash
ros2 run distance_controller current_frame_step --distance .05 --speed .03 \
  --odom-publisher-gid CURRENT_FILTERED_GID --raw-odom-publisher-gid CURRENT_RAW_GID \
  --output /absolute/new/step.json --supervised-manual
```

Use the separate pid operations wrapper for record/watch and closed evidence.
Backend JSONL logs alone are not a bag or an observation of camera supervision.

Local checks: `test/test_current_frame_step.py` and the bounded localhost fixture
`test/verify_current_frame_step.py`. Synthetic sensor tests establish command,
feedback-fault and cleanup behavior only; they are not real collision tests.

The am02 closed-bag diagnostic `/tmp/am02_strict_step_audit.json` found 21/21 strict
`unknown` results in the old P03 stopped tail, including with measured velocity set
to zero. Relevant rear missing rays were 68–83 of 720. This explains why strict
coverage is not a usable prerequisite for this explicitly supervised manual mode;
it does not show those missing regions are free or certify the current new epoch.

2026-10-22 local Humble verification: `python3 -m pytest -q
 test/test_current_frame_step.py test/test_route_laser_guard.py` passed 33 tests.
`bash /tmp/current_step_ros.bash` ran the installed entry on localhost domain 198.
All 13 cases passed under `/tmp/current_frame_step_ros_20261022/`: normal, manual
unknown coverage, nonmoving strict dry-run, cancellation, raw/filtered/scan stale,
raw/filtered publisher replacement, frozen-pose timeout, pre-first-command pose
shift, heading bound and travel bound. Each case has `verification.json`, audit,
command trace and sensor state. The pre-first-command shift produced only 20 zero
commands; dry-run produced none. Motion/fault cases ended with zero, no publisher,
released lock, and never more than one command publisher. These are synthetic
runtime checks, not real robot return or collision acceptance. No thresholds were
relaxed to obtain these results.
