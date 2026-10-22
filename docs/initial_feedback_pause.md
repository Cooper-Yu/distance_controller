# Precise initialization feedback pause

With `precise_start:=true`, initialization owns one bounded stopped recovery for
missing/invalid scan geometry or missing/stale filtered odometry. A filtered odom
receipt or header age over 0.35 s triggers zero before the outer 0.5 s moving
watchdog. Initial absence also enters this pause. Other route modes retain their
existing odom recovery behavior.

The controller publishes zero before logging `FEEDBACK_RECOVERING:` and on every
50 ms timer tick throughout the pause. The first detected loss starts one fixed
5 s deadline; changing the missing stream or receiving unusable data cannot
restart it. It discards an interrupted micro-correction target, PID history,
stopped measurement batch, and wall-heading evidence.

Recovery requires both streams to be newer than pause entry, header/receipt age
at most 0.15 s for odom and 0.2 s for scans, a valid wall fit, stopped measured
linear/angular speed below 0.003 m/s and 0.01 rad/s, and at least three distinct
frames from each stream sustained over at least 0.3 s. The transition still
publishes zero, then logs `FEEDBACK_RECOVERED:`. Initialization rebuilds wall
heading and position evidence and performs its normal final fresh stopped
verification. It never republishes a cached movement command.

The original total/stage deadlines continue during a pause; a deadline may end
recovery before its full 5 s. The preparation travel bound, final 5 mm/0.5 degree
criteria, small-step bounds, and reduced-speed settings are unchanged. Scan frame
changes, odom frame/pose discontinuities, nonfinite or invalid odometry, and a
backwards ROS clock remain terminal. Queued scan and duplicate odom timestamps are rejected
and do not count as new recovery evidence. Backwards, zero, or future odom timestamps fail immediately. The node observes filtered odometry;
raw-odom identity/freshness remains the outer owner's responsibility.

`scan_recovery.cpp` deliberately keeps the zero/deadline/evidence decision in one
function (over the default 60-line review threshold): all early returns preserve
zero, the transition is auditable, and splitting it would add shared mutable
state without a new responsibility.

## Local synthetic checks

Build/install the updated C++ node in Ubuntu-22.04/Humble first. With ROS and the
workspace overlay sourced, run each mode in a separate process:

```bash
python3 test/verify_initial_feedback_pause.py startup /tmp/init_feedback_startup
python3 test/verify_initial_feedback_pause.py scan /tmp/init_feedback_scan
python3 test/verify_initial_feedback_pause.py odom /tmp/init_feedback_odom
python3 test/verify_initial_feedback_pause.py both /tmp/init_feedback_both
python3 test/verify_initial_feedback_pause.py timeout /tmp/init_feedback_timeout
python3 test/verify_initial_feedback_pause.py odom_reset /tmp/init_feedback_reset
python3 test/verify_initial_feedback_pause.py odom_invalid /tmp/init_feedback_invalid
python3 test/verify_initial_feedback_pause.py odom_frame /tmp/init_feedback_frame
```

The fixture uses a generated room, localhost-only domain 194, and a direct node
child. It checks zero commands throughout the pause, stabilization after return,
success after short gaps, and a latched bounded failure when feedback never
returns. It records commands and logs under the supplied output directory. These
checks do not certify cloud deployment, real sensor behavior, or real hardware.
Verified locally on 2026-10-23 after the coordinator built the updated node:
all eight modes above passed against the installed ROS executable. Evidence is
`/tmp/init_feedback_<mode>_20261023/runtime.log` and `commands.json`. Run script:
`/tmp/run_init_feedback_pause.bash`. `clang-format --dry-run --Werror` on the five
changed C++ files, `ruff check test/verify_initial_feedback_pause.py`, Python
compilation, and `git diff --check` passed. The coordinator owns full build and
package CTest evidence; real hardware was not connected.