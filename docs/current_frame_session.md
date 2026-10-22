# Fixed-reference local backward session

`current_frame_session` is a separate, explicitly supervised local tool. It does
not prepare P01, load historical coordinates, resume another process, execute a
route, or certify obstacle avoidance. The existing `current_frame_step` and
formal route behavior are unchanged.

## Start and explicit one-step protocol

After independently checking the present robot view, stopped state, feedback and
current publisher identities, an operator may start a NEW session:

```bash
ros2 run distance_controller current_frame_session \
  --step-distance .03 --speed .01 --max-seconds 600 \
  --odom-publisher-gid CURRENT_FILTERED_GID \
  --raw-odom-publisher-gid CURRENT_RAW_GID \
  --output NEW_SESSION.json --supervised-manual
```

The placeholders must be replaced with the current complete GIDs and a new output
path. This command attaches and waits; it does not automatically move. No old
source book, P01 coordinate, or epoch override is accepted. The global existing
motion lock remains held throughout the session. `--max-seconds` is finite,
1–1800 seconds, including attachment and waiting; it never resets between steps.

A `session_ready` JSONL event supplies a new unpredictable `ready_token` and
`next_index`. Send exactly `step TOKEN` for ONE configured backward step, or
`quit` to close. The wrapper can forward the current token after an explicit
operator request. There is no auto-next command. Tokens are single-use, replaced
only after completion and release; stale, duplicate and unknown tokens are
rejected. Motion-time inputs other than `stop`/`quit` are discarded. `stop`,
`quit`, EOF or SIGINT during motion cancels the session; no fault can resume.

READY has no `/cmd_vel` publisher. It continuously checks stopped dual feedback,
identities, frames, scan/TF pairing and pose continuity. It does not publish zero
commands to keep the robot or epoch alive. Closing/fault cleanup reuses the
existing bounded zero/release lifecycle.

## Two distinct references

The first fresh dual-odometry stopped hold freezes `LOCAL_SESSION_START`, the
session UUID, filtered/raw anchors and publisher GIDs. This is a new local frame,
not a claim that the robot is at P01. The JSON book uses `relative_to_session` and
explicit `historical_p01=false`, `resumable=false` metadata.

Each step separately captures its actual stopped start and raw counterpart for
pre-command checks and travel limits. If the measured position in session axes
is `(sx, sy)`, a request of `d` uses a fixed-frame path from `(sx, 0)` to
`(sx-d, 0)`, with heading zero relative to the session anchor. The next target
uses measured longitudinal progress; it does not add a prior endpoint shortfall
to the new request. All steps retain the same lateral line and yaw.

The actual start-to-target chord `hypot(d, sy)` must be <=5 cm; a 5 cm request
with lateral error is rejected rather than silently shortened. The chord is
checked again before the first nonzero command, even if the actual start has
changed within the original 5 mm pre-command allowance. No reconnect or
pre-motion turn is called. Existing `Console.move` path logic is reused, with
`use_path_length=True` so distance from the initial anchor does not inflate the
step budget.

Limits are deliberately not enlarged for a longer return:

- Speed .01–.03 m/s, angular command cap .10 rad/s, ten monotonic seconds per
  requested step including stopped verification/recording.
- Entry lateral error <=1 cm and heading error <=1 degree relative to the fixed
  session axes; final endpoint uses the existing **2-D distance <5 mm**, and
  wrapped yaw error <.01 rad.
- The existing manual active .035 rad heading bound is enforced against both
  the fixed session yaw and each step's start; raw heading is checked against
  its own fixed raw anchor. Wrapped equivalent angles are accepted; a genuine
  yaw discontinuity is not reinterpreted as a new reference.
- Each step retains actual displacement <=chord+1 cm and total travel
  <=chord+2 cm, plus the existing pre-first stopped/5 mm/.01 rad checks.
- Source/receipt age thresholds remain .5 s. Source stamp regressions,
  same-stamp content changes, frame/GID changes and stale geometry invalidate
  the session. The new session additionally rejects source gaps >.5 s and
  observable per-sample pose changes exceeding speed*dt+5 mm or .10*dt+.01 rad.
  Continuity is checked on actual adjacent receive callbacks, including inside
  exclusive/stop loops; gaps between guard calls are not source gaps. Same-stamp
  content changes latch a fault and unchanged duplicates retain their first receipt.
  These are conservative rejection bounds, not proof that every small odom
  reset can be detected.

The original 4 cm turn-center guard is unchanged; this tool has no turn action.
Small heading corrections during translation/settling use the existing path
controller. Its bounds do not prove collision clearance. The explicit manual
mode retains laser **observation only**: unknown/blocked decisions are logged,
not converted into free space, and fresh scan/TF pairing is still mandatory.
Every command obtains current geometry/scan pose through the existing guard.

## Audit and result contract

- `session_started`: `session_id`, immutable `anchor` (x/y/yaw), dual `gids`,
  `step_distance_m`, `speed_cap_m_s`, `laser_policy=observation_only`,
  `supervised_manual=true`, `historical_p01=false`, `task6_acceptance=false`.
- `session_ready`: same anchor/id, `next_index` starting at 1, `ready_token`,
  actual stopped pose/stamp, `cmd_publishers=0`.
- `step_started`: same anchor/id, `index`, `actual_start`, planned fixed-frame
  motion. `step_completed` records verified final pose with the same index.
- Only after publisher release, dual stopped hold, endpoint/record checks and
  completion is another READY advertised. Completion alone is not permission
  for another step.
- Normal idle quit returns 0 with `session_closed` and `completed_steps`, after
  cleanup. Failure/cancellation returns 2 with `session_failed`. Cleanup errors
  are separately recorded without replacing an earlier fault.

## Verification boundary

`test/test_current_frame_session.py` exercises the actual pure path model,
reference/feedback checks, permits, deadline and lifecycle orchestration with
mock ROS I/O. `test/test_backward_reference.py` separately demonstrates the old
per-move residual inheritance and actual fixed-path feedback signs. Run under
WSL Ubuntu-22.04/Humble with the workspace overlay sourced:

```bash
python3 -m pytest -q test/test_current_frame_session.py \
  test/test_backward_reference.py test/test_current_frame_step.py \
  test/test_motion_console.py
```

These are local software checks. The independently coordinated isolated ROS
fixture and real-robot supervision/clearance tests have separate evidence gates.
This candidate does not establish a verified physical return to P01 or Task6
avoidance acceptance.