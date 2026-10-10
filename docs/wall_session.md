# Task6 wall-guided commissioning

This is a complete **single-step experimental route**, not a verified autonomous maze run.
Use `next` to validate every segment on site. `run` is disabled for wall policies.
The existing distance-only action session and Task1–5 tags remain unchanged.

## Start at physical P01

```bash
ros2 run distance_controller action_session --wall-guided
ros2 run distance_controller action_session --wall-guided --start prepare
```

Preview starts no ROS controllers. Preparation aligns with the **left wall**, centers
between both walls, and uses the existing rear wall-distance target of 0.280 m.
That rear initialization setting is measured from the base_link origin, not the body edge.
After preparation finishes and its controller exits, five distinct stable stopped
scans establish the carried **left body clearance** `d`. No route motion starts yet.
The odom epoch must remain unchanged. Do not use this start command at P03 or another
intermediate point. `--start current` explicitly adopts a new origin at the current
stopped pose; it does not resume old progress and still requires a left reference wall.

## Route policies

All clearances below are fitted wall-to-body distances. The default route keeps inherited values. Targets resolve independently: explicit absolute clearances override inherited `d` plus offsets.
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
| P02-P03 forward | left `d`; front `d-0.01` | 1.05 m |
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
side residuals <=0.008 m, heading error <=0.01 rad, measured linear speed <0.01 m/s
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
apply to unexecuted actions; `back` must complete before editing a finished/partial
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
Both heading and body-clearance fits select a dominant line using bounded candidate
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
loss remains an immediate stop. Permanent bad fits time out with the last fit reason.

Wall-guided heading preparation collects at least eight accepted stopped scans spanning
0.5 seconds in a recent 1.5-second window. Directions are expressed in odom and
unwrapped before taking the median; 80% must lie within 0.02 rad of it. The resulting
yaw target is frozen during odom-controlled rotation. After settling, a fresh window
verifies alignment against the unchanged heading tolerance. At most two refinements
are allowed; existing preparation/scan/odom guards remain active. Legacy non-robust
alignment is unchanged. Logs distinguish frozen odom target and wall verification.
