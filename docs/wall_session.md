# Task6 wall-guided commissioning

This is a complete **single-step experimental route**, not a verified autonomous maze run.
Use `next` to validate every segment on site. `run` is disabled for wall policies.
The existing distance-only action session and Task1–5 tags remain unchanged.

## Start at physical P01

```bash
ros2 run distance_controller action_session --wall-guided
ros2 run distance_controller action_session --wall-guided --start prepare
```

Preview starts no ROS controllers. Preparation defaults to the **right wall** for heading, centers
between both walls, and uses the existing rear wall-distance target of 0.280 m.
That rear initialization setting is measured from the base_link origin, not the body edge.
After preparation finishes and its controller exits, five distinct stable stopped
scans establish the carried **left body clearance** `d`. No route motion starts yet.
The odom epoch must remain unchanged. Do not use this start command at P03 or another
intermediate point. `--start current` explicitly adopts a new origin at the current
stopped pose; it does not resume old progress and still requires a left reference wall.

## Route policies

All clearances below are fitted wall-to-body distances. Most default actions inherit the carried reference; P03 uses a calibrated absolute front target. Targets resolve independently: explicit absolute clearances override inherited `d` plus offsets.
The `value` of wall-stop actions is a nominal preview distance, not their stopping criterion.
Every action has an independent accumulated maximum-travel limit. All translations
hold planned odom heading. Straight wall-following adds at most 0.012 m/s lateral
correction; main translation speed defaults to 0.060 m/s. Set `--wall-speed 0.03` to use the original
slow trial speed; accepted limits are 0.01-0.08 m/s. Speed reduces proportionally near
the distance/clearance target. Initialization, return and turn speeds are unchanged. Turns are separate and use
turn_controller at 0.20 rad/s. The odom heading remains continuous across +/-pi.

| Action | Reference / arrival | Maximum path |
|---|---|---|
| P01-P02 forward | left `d`; travel 0.90 m | 1.20 m |
| P02-P03 forward | left `d`; front 0.075 m, tolerance 0.010 m | 1.05 m |
| P03 right 90 degrees | capture new left `d` after stopped turn | no translation |
| P03-P04 forward | left `d`; front `d` | 0.75 m |
| P04-P05 right | stop at right `d` | 0.60 m |
| P05-P06 forward | right `d`; front `d` | 0.80 m |
| P06-P07 left | stop at left `d` | 0.70 m |
| P07-P08 forward | left `d`; travel 0.50 m | 0.75 m |
| P08-P09 right | stop at right `d` | 0.80 m |
| P09-P10 forward | right `d`; front `d-0.01` | 0.70 m |
| P10 right 90 degrees | capture new left `d` after stopped turn | no translation |
| P10-P11 forward | left `d`; travel 0.40 m | 0.65 m |
| P11 right 90 degrees | heading only | no translation |
| P11-P12 forward | no side reference; front `d-0.01` | 1.10 m |
| P12 left 90 degrees | heading only | no translation |
| P12-P13 forward | travel 0.55 m | 0.80 m |
| P13 right 90 degrees | heading only | no translation |
| P13-P14 forward | travel 0.48 m | 0.73 m |
| P14 left 90 degrees | heading only | no translation |
| P14-P15 forward | travel 0.40 m | 0.65 m |
| P15 left 180 degrees | heading only | no translation |

These are initial trial choices, not measured optimal clearances. The 1 cm front offset
comes from the local model's front extent 0.17 m versus lateral extent 0.16 m. It
predicts equal post-turn side clearance only for perpendicular walls and negligible
rotation-center drift. Turn clearance is checked separately; endpoint geometry alone
cannot certify a rotation. P04/P06 use `d` because a lateral move follows.

## Geometry, feedback and stops

The body model is the union of chassis rectangles (+/-0.170 m, +/-0.135 m) and wheel
rectangles (+/-0.135 m, +/-0.160 m). Physical protrusions must be checked against this
local model. Laser points are transformed with full stamped TF into base_link;
translation offsets and the 180-degree lidar mounting are not hardcoded.

Each required wall uses a +/-20 degree window and an orthogonal line fit. At least
8 valid points, 60 percent valid coverage, 0.08 m tangent span, <=0.012 m RMS, and a
normal within 12 degrees of its expected direction are required. Body gap subtracts
the entire footprint's projection onto the fitted wall normal, including corners.
Five distinct scans spanning <0.015 m establish a reference in [0.05,0.30] m.

Missing/stale scan or TF, stale odom (>0.5 s), discontinuity, rejected fit, a side gap
more than 0.08 m away from its reference, travel limit, or timeout stops and marks
the action incomplete. There is no fallback from a missing wall to nominal distance.
A reference may still select the wrong nearby parallel surface; inspect site geometry.
No unseen obstacle or transparent surface is certified safe by the laser model.

Approach slows proportionally before the target. Arrival requires distance/gap and
side residuals <=0.008 m (front arrival uses its per-action stop_tolerance), heading error <=0.01 rad, measured linear speed <0.01 m/s
and angular speed <0.02 rad/s, continuously for 0.5 wall seconds. A stopping wall
already more than 0.015 m closer than its target rejects motion for inspection.

Observed obstacle points are checked against current and short predicted footprint
poses with a 0.02 m minimum gap. Turns require surrounding scan coverage and check
the requested sweep before launch, plus a short sweep while turning. A failed scan
can block a turn even when the scene looks open. Do not bypass this by assuming no return
means free space. Translation exceptions send repeated zero commands before releasing
the publisher. Turn exceptions stop the owned C++ process. The base watchdog is still
required if the supervising process or host disappears. This entry targets real-time
Humble feedback; it is not a simulation-clock controller.

## Editing, history and return

Type one command per line, then Enter:

```text
plan
next
measure
status
history
```

`WALL_ACTION` prints the concrete carried reference, stop target, nominal distance,
and maximum travel before motion. `WALL_PROGRESS` shows remaining error and fitted
gaps. `BODY_CLEARANCE` supplements raw sensor ranges; unavailable fits are explicit.

```text
back
BACK
policy 2 offset 0.00
next
save task6_wall_trial.json
```

`back` first requires the operator to confirm a clear return path. It goes to the
recorded actual action start, retaining scan protection but disabling forward wall
arrival policies. A successful return restores the prior reference distance and makes
that action editable. Incomplete steps must be returned before retrying. A collision,
blocked path, stale feedback or odom reset is not automatically recoverable.

`set INDEX METERS` changes fixed-distance travel (or the nominal preview for a wall-stop
step). `set INDEX DEGREES` changes a turn. `policy INDEX offset METERS` changes wall
arrival clearance relative to `d`; `policy INDEX max_travel METERS` changes its bounded
search distance. Executed actions cannot be edited without returning first.

`save` retains every wall policy; reload with `--route task6_wall_trial.json`. A saved
route is not resumable robot state. `--reverse` is rejected for wall routes because
nominal distances cannot invert feedback-determined endpoints; use same-session back.

After each completed action, future previews use the actual stopped x/y and planned
heading. Fixed-distance-only legacy sessions still keep their original planned anchor.
JSONL retains attempted targets, resolved endpoints, actual start/end, residuals,
longitudinal/lateral displacement, accumulated path, reference changes and snapshots.
The resolved XY of a wall stop is measured rather than independently commanded; use
wall residuals to assess arrival, not its necessarily-zero resolved-position error.

## Local verification

```bash
python3 test/test_wall_guidance.py
python3 test/test_action_plan.py
python3 test/test_action_preparation.py
# Source ROS/overlay; fixture forces localhost and ROS_DOMAIN_ID=181.
python3 test/verify_wall_session.py
```

Pure tests cover geometry, sign conventions, range/coverage rejection, footprint sweep,
front and lateral arrival, reference rollback and actual-anchor propagation. The ROS
fixture raycasts a virtual rectangular room; it is not a complete CyberWorld run.
Physical P01-P15 acceptance and the Task6 tag remain pending.

## Independent clearance per action

Each wall policy supports `follow_clearance` and `stop_clearance` in meters (0.04-0.35).
`null` inherits the current carried reference: follow uses `d + follow_offset`, while
stop uses `d + offset`. An absolute value takes precedence over that offset. It does
not change the shared reference or the next action. Only explicit post-turn capture
updates `d`. Fields for disabled follow/stop walls are rejected.

At WAITING, for example before action 2 (P02-P03):

```text
policy 2 follow_clearance 0.12
policy 2 stop_clearance 0.15
plan
save task6_trial.json
next
```

These example values are body clearances, not sensor ranges or site-validated recommendations.
Use `policy 2 follow_clearance auto` to restore inheritance; `follow_offset` can then
adjust the inherited follow gap. `offset` adjusts an inherited stop gap. Edits only
apply to unexecuted actions; except for the stopped front-endpoint calibration below, `back` must complete before editing a finished/partial
action. Returning restores the recorded reference but retains the action's independent
configuration. Saved JSON includes all values; `--route task6_trial.json` reloads them.
Existing `--start` semantics still apply: saved routes do not restore progress or odom.

Execution prints resolved follow/stop targets and records them in `wall_result`.
`plan` shows configured fields and the current reference; targets after future captures
remain provisional. Distance-only actions have no stop-wall target. Turns do not
translate or automatically search for space: insufficient clearance still stops the
action. Inspect and return before changing the preceding translation target.

## Dominant wall fitting

Wall sessions enable `robust_wall_heading` for preparation (legacy default remains false).
Heading and front-arrival fits select a dominant line using bounded candidate
pairs and a 12 mm perpendicular inlier threshold. At least 80% of valid window points
must agree; original minimum return coverage, fitted span, angle and RMS checks still
apply. Preparation retains the continuous stability gate. No dominant line means stop.
The recorded single scan `test/left_wall_returns.json` reproduces sparse distant returns;
it does not certify continuous hardware operation or identify the physical reflector.
All original valid scan points remain available to clearance/swept-obstacle guards.
This estimator never declares discarded wall-fit points to be free space.

Stopped reference capture tolerates individual fresh-scan fit rejections within its
original four-second deadline. Each rejected fit clears the five-frame sequence;
only five distinct stable accepted scans commit a new reference. Stale/missing
feedback, TF failures and cancellation still abort immediately. Movement-time wall
loss immediately commands zero, with the bounded stationary recovery described below. Permanent bad fits time out with the last fit reason.

Wall-guided heading preparation collects at least eight accepted stopped scans spanning
0.5 seconds in a recent 1.5-second window. Directions are expressed in odom and
unwrapped before taking the median; 80% must lie within 0.02 rad of it. The resulting
yaw target is frozen during odom-controlled rotation. After settling, a fresh window
verifies alignment against the unchanged heading tolerance. At most two refinements
are allowed; existing preparation/scan/odom guards remain active. Legacy non-robust
alignment is unchanged. Logs distinguish frozen odom target and wall verification.


## Side-distance tracking and bounded recovery

During translation, side distances use a +/-30 degree window (previously +/-20).
The held odom heading supplies the wall normal in the current body frame; the median
projected distance estimates offset without treating noisy fitted angles as new headings.
At least 80% of valid returns must lie within 20 mm of the median, with 60% ray coverage,
12 mm RMS and 8 cm span. Front arrival uses an independent +/-12 degree TLS window; heading initialization retains its existing fit.
Stopped side-reference capture uses this same constrained side-distance estimator.
Raw obstacle points are never discarded by this estimator. The three user scan replays
in `test/side_distance_scans.json` cover initial, centered and interrupted placements;
they are base-link points under the user-provided mounting TF, not ground truth.

A fit failure or a distance jump over 3 cm commands zero immediately. The action keeps
its original target and waits at most two seconds for three distinct stopped scans
with less than 15 mm gap spread. A recovered wall must stay within 3 cm of the last
accepted distance. Total recovery time is limited to four seconds per action. Odom,
scan/TF and obstacle failures still abort immediately. Timeout retains a partial action;
`back` returns to its recorded start before retrying. Logs identify `WALL_RECOVERING`,
`WALL_RECOVERED` and the constrained side estimator. This is local validation, pending
cloud single-step acceptance; it does not certify gaps or the entire hardware route.


## Independent preparation heading wall

`--alignment-wall right` (default) selects the preparation heading reference only.
`--alignment-wall left` explicitly selects the other wall. This option requires a
wall-guided route and `--start prepare`; it does not rewrite action follow/stop policies.
A failed heading measurement stops preparation; there is no silent wall switching.
The C++ controller still centers between both sides and adjusts rear distance to 0.28 m.

After preparation, the stopped origin yaw is frozen for five-frame **left** side-distance
capture with the +/-30 degree constrained estimator. Post-turn reference capture uses
the planned turn heading instead. Neither capture requires the old free-angle left-wall
fit, but fresh scans/odom/TF, stopped feedback, spread and the four-second deadline remain
required. Normal manual `BODY_CLEARANCE` output still uses independent TLS diagnostics.

```bash
ros2 run distance_controller action_session --wall-guided --start prepare \
  --alignment-wall right --wall-speed 0.06
```

First confirm `right-wall alignment`, `preparation complete`,
`REFERENCE left ... held_heading_30deg`, then `WAITING`. Execute one `next`.
Right-wall heading and subsequent left-wall following assume the selected walls are
parallel at the start; local tests do not establish that at every real placement.


## Front stopping window

Front arrival and manual front BODY_CLEARANCE now use +/-12 degrees. Expected-ray
coverage is counted in that same window, including invalid returns. The original
80% consensus, 60% inlier coverage, 12 mm RMS, 8 cm span and angle limits remain.
The real P02 replay has 38/43 inliers (88.4%), about 0.9805 m body clearance.
Narrowing applies only to wall estimation; full scan points and wider travel coverage
still protect motion. A missing or short front surface is never treated as clear space.

An already-running Python session does not load this code update. To keep route meaning,
return to P01 with the existing session's back command while the return path is clear,
then quit, update/build and prepare again. Do not start the full route with --start current
at P02: that would adopt P02 as a new origin and repeat P01-P02.

### Clearance failure diagnostics

`CLEARANCE_DIAGNOSTIC` reports the triggering scan stamp, pose, measured linear
speed and absolute angular speed, static body gap, swept gap, separate speed
allowances, protected residual and 0.02 m threshold. Point coordinates and bearing
are in `base_link` (meters and radians; an explicit degree bearing is also printed).
`RETURN_CLEARANCE` identifies recorded-pose return protection; `TURN_CLEARANCE`
identifies delegated rotation. Return now evaluates directional sweeps of signed measured body velocity and
the fresh delegated command; the 2 cm threshold remains unchanged.
The session JSONL failure event includes all transformed valid points under
`laser[].guard_failure.points_base_xy_bearing`. Preserve this audit when reporting
a failure: a later stationary scan cannot reconstruct the triggering geometry or
velocity feedback. This is diagnostic evidence, not proof that a return path is safe.

Translation `OBSTACLE` failures now emit the same diagnostic prefix with
`mode=translation`, `movement`, `zero_command`, `command_vx_vy_wz`, and the
0.5-second prediction horizon. Static gap and predicted gap are separate.
A zero-command check occurs during recovery/settling and does not subtract
measured speed allowances. Measured speeds are recorded only as context for this
branch. Full transformed returns are retained in the failure audit. No points
are discarded and the 2 cm threshold is unchanged.

### Resume at a stopped waypoint after updating code

The CLI prints an `Auto-checkpoint: ...checkpoint.json` path and atomically updates
it after each operator command. `checkpoint FILE` explicitly saves a stopped
snapshot (distinct from `save FILE`, which still exports route definitions only).
Before a command the automatic file is marked unusable; an abrupt crash during
motion therefore cannot silently restore an earlier completed state.

Exit the old session before starting another controller:

```bash
ros2 run distance_controller action_session --resume /absolute/path/session.checkpoint.json
```

Confirm `SAME_ODOM` only if the odometry system was not reset and the robot was not
manually relocated. Fresh stopped feedback, unchanged frame, nondecreasing stamp,
position within 3 cm and heading within 0.05 rad are required. These checks cannot
prove continuity after every possible odom reset. A reset requires establishing a
new reference; there is no force/automatic coordinate rebase. Recovery skips P01
preparation and wall-reference recapture and enters WAITING without motion.
Continuous yaw turns are restored across the +/-pi boundary.

- Completed action: `next` executes the saved next action.
- Interrupted outward action: `resume`, then `RESUME`, preserves the original
  target, translation origin and cumulative path limit. Wall-stop actions retain
  their wall policies; they still require valid live scans.
- `back`, then `BACK`, returns to the recorded actual start. A failed return must
  finish returning before outward work resumes. Inspect the path before motion.
- Completed history, route edits, clearance policies and carried reference survive
  restart. Invalid/unknown-stop snapshots refuse recovery.

For existing pre-checkpoint sessions, `--resume` also accepts a stopped `.jsonl`
audit. It replays the recorded route/history/reference and checks the latest stop
against current pose. Legacy audits lack frame/stamp epoch evidence, so operator
continuity confirmation is essential. Legacy interrupted actions allow `back`
only; their missing cumulative travel budget is never guessed. Truncated/in-flight
or unknown-stop audits are rejected. Do not restart the entire route at the current
position using `--start current` as a substitute for recovery.

Local verification: `test/test_session_checkpoint.py` and
`WALL_CASE=checkpoint python3 test/verify_wall_session.py` cover checkpoint
round-trips, mismatched pose/frame/time, busy files, yaw wrapping, legacy migration,
and real ROS process restart for completed and interrupted translations in the
synthetic scan/odometry fixture. This does not certify the real maze or hardware.

### Directional protection during history return

Return prediction uses signed `vx`, `vy`, `wz` from fresh odometry (twist frame
must be `base_link`) and the latest `/cmd_vel` command received within 0.5 s.
Each scenario is sampled over the existing 0.5 s horizon; the minimum clearance
across both and all original scan points controls stopping. Static overlap is
still checked at time zero. A point on the right does not automatically consume
backward clearance merely because speed magnitude increased. Yaw sign is retained.
The observer never publishes a competing command. Unknown twist frames refuse
return; stale odom/scan checks and the 2 cm threshold are unchanged.

Diagnostics label `prediction=directional_sweep`, list the velocity scenarios and
identify the limiting one. Old scalar allowance fields are zero; compare
`static_gap_m` to `sweep_gap_m`. This remains a sampled, constant-velocity model,
not a proof about unseen obstacles or future acceleration beyond the horizon.
The captured nearest point from the cloud failure is covered by backward,
approaching, rotational and static-overlap tests. Full triggering scan replay
still requires the cloud JSONL, not just its nearest-point console summary.


### Bounded stopped recheck during return

A `RETURN_CLEARANCE` event still stops motion immediately. The owned C++ child
is interrupted and reaped using the existing zero-command shutdown. Only then
`RETURN_RECOVERING` holds zero commands for a maximum of 2 seconds. It requires
at least three distinct fresh scan/odom pairs spanning 0.20 seconds, linear speed
below 0.01 m/s and absolute yaw rate below 0.02 rad/s. All original scan points
must clear the unchanged 2 cm boundary at rest and over the 0.5-second prediction.
Recheck includes the triggering velocities, current measured motion, and a
0.03 m/s target-directed restart with conservative heading correction up to the
child's default 0.25 rad/s. Missing travel-direction returns cannot count as clear.

`RETURN_RECOVERED` restarts the controller toward the **same absolute history
pose**; it does not repeat the original full distance or reinitialize P01. There
are at most two recovery attempts per return command. Persistent blockage reports
`RETURN_RECOVERY_TIMEOUT`; repeated interruptions report `RETURN_RECOVERY_BUDGET`.
Both preserve incomplete history/checkpoint state. Stale feedback, invalid TF,
odom discontinuity, cancellation and turn-clearance errors are not automatically
retried. The 2-second window starts after child shutdown, so total visible pause
also includes stopping and subsequent controller startup.

Verification: `test/test_return_recovery.py` covers confirmation, duplicate scans,
non-stopped feedback, missing coverage, predicted collisions, cancellation and
retry limits. `WALL_CASE=return_recovery python3 test/verify_wall_session.py`
executes transient and persistent rear-obstacle scenarios with real local ROS
processes in isolated domain 181. These synthetic checks are not hardware acceptance.

2026-10-10 local evidence: 74 relevant pure tests passed (including 9 new return
recovery tests), Ruff format/check, `git diff --check` and Humble `colcon build`
passed. Final ROS fixture: `/tmp/wall_session_test/1791614903979554593`.
Transient rear returns recovered to x=0.009557 m near the original x=0 target;
persistent returns timed out at x=0.070595 m, remained incomplete and stopped.
No real-robot or full-route acceptance is claimed by these results.


### Capture the actual wall-estimation failure

`WALL_FIT_DIAGNOSTIC` identifies the first failing `side` and `reason`, estimator,
scan/odom stamps, pose, held heading and measured motion. Motion sends zero before
logging. The JSONL observation `wall_fit_failure` also retains all transformed
valid points (`points_base_xy_bearing`) and scheduled ray counts for offline
replay; the terminal omits the large point array. Duplicate stage/side/scan failures
are suppressed. Distance jumps report the previous/current gaps separately from
line-fit failures. Successful stationary scans cannot explain earlier moving scans.

`WALL_RECOVERY_CHECK` reports fresh fitted gaps, previous gaps, stopped status,
distance continuity, previous stable sample count and already-used recovery time.
These are diagnostics only: the +/-12-degree front window, side estimator,
consensus thresholds, 2 cm guard and 2/4-second recovery limits are unchanged.
After updating a partial session, restore its latest checkpoint with unchanged
odom; use `resume` / `RESUME` for outward continuation, not `next` or P01 preparation.
Send the newly printed audit JSONL if another failure occurs.


### Opposite-side clearance limits lateral correction

For forward actions with a follow wall, `SIDE_CORRECTION_LIMITED` reports a reduced
lateral correction when the requested correction consumes nearby clearance.
The controller tests 75%, 50%, 25%, then zero of the **planned lateral component**;
planned forward velocity and yaw correction are preserved before rotating to
body axes. It seeks 2.5 cm predictive margin where feasible, otherwise the margin
of the command without lateral correction, never below the existing 2 cm boundary.
If removing correction is unsafe, the original full guard still decides to stop.
Original requested-direction coverage is checked before limiting: missing side
returns cannot enable this fallback. All scan points remain in the calculation.
Measured body motion also remains guarded (`velocity_source=measured` in a failure),
so reducing the requested velocity does not excuse momentum toward an obstacle.

This does not change the carried wall reference or endpoint tolerance. If the
side target remains infeasible at the endpoint, completion is withheld and the
existing action timeout remains. Strafe and turn actions do not use this limiter.
Independent `follow_clearance` / `follow_offset` policies remain available; this
change deliberately leaves P02-P03's target unchanged for isolated cloud testing.
A partial action normally requires return before edits. The stopped front-endpoint
calibration exception below permits only stop_clearance and stop_tolerance. Do not edit checkpoint JSON
by hand to bypass this boundary.

Local evidence (2026-10-10): 86 relevant pure tests, Ruff and colcon passed. Replay
of the user's 641-point triggering scan with the actual heading rotation reduced
the planned lateral term to zero: predicted clearance .01851275 -> .02192001 m.
The ROS_DOMAIN181 narrow-patch fixture completed at x=.112187 m and stopped:
`/tmp/wall_session_test/1791616140146098746`, `WALL_CASE=side_limit`.
This is sampled-model and synthetic-process evidence; the real corridor is still
awaiting single-action cloud verification.

## Front fit and braking-distance continuity

The front +/-12 degree fit uses an 18 mm inlier band, independently of the side
wall estimators. The 80% consensus, 60% scheduled-ray coverage, 12 mm final TLS
RMS, 8 cm span and 12 degree normal limit still apply. This was replayed against
16 failed front scans from session 1791616736137976195; their final RMS is below
12 mm. Raw points remain available to the unchanged 2 cm obstacle guard.

Continuity compares measured gaps against the last trusted wall transported by
odom translation and rotation, including the change in body support along its
normal. The allowed residual remains 3 cm. The snapshot is fixed through a
recovery and duplicate scans cannot re-anchor it. A scan without a historical odom pair within 0.15 s,
translation over 8 cm or rotation over 0.15 rad from that snapshot rejects the
compensation. Odom history retains up to 200 validated samples. Bracketed samples interpolate position and continuous yaw at the scan timestamp; outside the history a nearest sample within 0.15 s is used without extrapolation. The pairing is frozen for that scan, so newer odom does not invalidate a retained scan. Missing pairs enter the bounded zero-command wall recovery; stale feedback still aborts.
The three stopped scans, 2 s retry and 4 s cumulative limits remain unchanged.
`WALL_RECOVERY_CHECK` includes `expected_gaps`, `scan_pose` and `pair_offset_s` to distinguish braking travel from
a change of surface. These are bounded local checks, not proof of physical safety.
Before wall processing, the executor refreshes ready callbacks with zero wait,
up to 32 calls or a 5 ms budget (checked between callbacks). This prevents a single
scan/command callback from indefinitely delaying queued odometry; it does not
wait for missing feedback. Cancellation and latched odom errors are checked during
and after refresh. A slow callback can itself exceed the time budget.

Motion fits, including failed fits, are cached only for the exact scan timestamp,
heading and requested wall set. Geometry and pose freshness checks still run on
every use; new scans re-fit. The cache holds one result, not a history of surfaces.
The 0.15 s pairing bound, 2/4 s recovery budgets and all-point obstacle guard remain.

## P03 stopped front-endpoint calibration

P03 now requests 0.075 m fitted body clearance with `stop_tolerance=0.010` m.
This is provisional calibration from three stationary readings (0.07738, 0.07101,
0.06948 m), not a verified turn clearance. The carried left reference stays unchanged.
Other wall stops default to 0.008 m tolerance; supported tolerances are 0.005-0.010 m.

For front-wall stops, entering the front band first commands zero on all axes.
Three distinct fresh scans with stopped odom are required before lateral/heading
correction resumes. A missing fit or out-of-band front reading resets this gate.
The final all-axis arrival/0.5 s hold remains required. The 15 mm too-close rejection,
raw-point 20 mm collision guard, scan freshness and accumulated travel limits remain.
`front_verified` in progress logs distinguishes verification from normal approach.

Existing checkpoints preserve their own policies; new defaults do not overwrite them.
Quit the old process, update/build, then restore the **latest** stopped checkpoint
using `--resume` (unchanged odom and no manual relocation). At the restored prompt:

```text
status
policy 2 stop_clearance 0.075
policy 2 stop_tolerance 0.010
plan
resume
```

Use these commands only when status identifies action 2 as the current partial P03
approach. Confirm RESUME after inspecting the stopped pose/path. Do not run `next`
to turn until this action completes and its resulting clearance is reviewed.

Partial edits are restricted to the current outward front-stop action, require fresh
stopped feedback and unchanged pose, and reject edits during an interrupted return
or legacy back-only recovery. They preserve start, target pose preview, reference,
path budget and completed history; the route and partial record change together,
with previous/new definitions in the append-only audit and automatic checkpoint.
Distance, following policy and maximum travel cannot be changed this way.

Local verification: 116 unit tests, Ruff and colcon passed in Ubuntu-22.04/Humble.
ROS synthetic feedback validated the 0.075 m stop with 0.010 m tolerance and completed/
partial checkpoint recovery. Cloud P03 completion and subsequent turn remain pending.
