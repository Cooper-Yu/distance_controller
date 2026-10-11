# Relative motion and teaching console (ROS 2 Humble)

This supervised tool executes relative translations/turns and records named odom
poses. It does not initialize against walls, navigate to recorded points, resume
old sessions, or provide obstacle avoidance. Laser data is observation-only.
Physical distance accuracy is not certified by synthetic-plant tests.

## Cloud update and launch

Stop teleop and other motion controllers first. Keep the existing robot drivers,
EKF, laser and TF running. The current stopped pose becomes P01 automatically.
Place/initialize the robot as desired before launching this console.

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src/distance_controller
git pull --ff-only
cd ~/ros2_ws
colcon build --packages-select distance_controller
source install/setup.bash
ros2 run distance_controller motion_console --laser-log-only
```

Default maximum translation speed is 0.04 m/s, angular speed 0.10 rad/s. Override
with `--speed` (.01-.08) and `--turn-speed` (.02-.20). Optional `--output FILE.json`
must be a new file. The default is `motion_points_<timestamp>.json` in the current
directory, with a matching `.jsonl` audit. Existing output is never overwritten.

## Commands

```text
move forward 0.10
move backward 0.05
move left 0.03
move right 0.03
move xy 0.10 0.03 --heading on --lateral on
move forward 0.10; move left 0.03
turn right 90
turn left 90
record P02
delta P01 P02
points
status
measure
stop
quit
```

Meters for translation; positive degrees for left/right turns. `xy` accepts signed
forward/left coordinates. Translation length is bounded to 2 m per command; turns
are bounded to 360 degrees. Commands are parsed as data, never evaluated by a shell.
A whole semicolon batch is syntax-validated before its first command executes.
Each motion captures its own actual stopped start pose. Sequential moves therefore
include the preceding motion's actual endpoint error; they are not absolute route
waypoints. An error, stop or Ctrl+C aborts the remaining batch.

`stop` + Enter works during motion. Ctrl+C also stops; the prompt remains available.
`quit` + Enter and EOF stop and exit. SIGTERM stops and exits. While moving, other
newly typed commands are discarded, not scheduled. Use semicolons to submit a batch.
This is an interactive tool, not a pipe-based automatic route executor.

## Correction meaning

Both flags default to `on`, independently on every `move`:

- `--heading on`: hold this command's initial heading using angular feedback.
- `--lateral on`: correct displacement perpendicular to the requested line.
- `--heading off`: angular command is zero; desired translation is still converted
  from the frozen start axes into the current body axes.
- `--lateral off`: no cross-track correction; arrival only checks along-track
  distance (plus heading when enabled). Full endpoint x/y accuracy is not claimed.

For `move xy .10 .03`, the reference is a straight diagonal, not forward then left.
Cross-track correction is perpendicular to that diagonal, not simply body `vy`.
Turns command angular motion only, preserve the requested sign, and reject more
than 4 cm estimated center drift. Completion needs the enabled errors within
5 mm along/cross, 0.01 rad heading, plus stopped measured speed for 0.5 seconds.
These are **odom acceptance tolerances**, not real-world accuracy guarantees.

## Point data

`record NAME` captures a stopped pose and laser observation, rechecks the pose,
and atomically saves the JSON. Duplicate names are rejected; use a new label for
revisions or pre/post-turn poses. P01 is created automatically and cannot be replaced.

Every point contains raw odom x/y/yaw, timestamp, session UUID (at file level),
P01-relative forward/left/yaw and a laser observation. `delta A B` returns:

- `start_body`: forward/left displacement relative to A's heading, straight-line
  distance and shortest signed yaw difference in degrees;
- `p01_axes`: displacement in the session origin's axes;
- `odom_axes`: raw odom coordinate subtraction.

A delta is net displacement, not traveled path length or proof of free passage.
No cross-session import/motion is supported: a new odom epoch needs a new file.

`measure` prints sensor-origin ranges and separately labelled fitted body gaps.
Body gaps reuse the existing **provisional** chassis/wheel model; they are not a
certified hardware outline. Missing scans/TF or failed wall fits are labelled
unavailable, never interpreted as open space. Laser observations run while stopped,
including after each completed motion and during point recording; they do not stop
motion even for a nearby obstacle. No continuous laser avoidance is implemented.

## Data/control flow

```mermaid
flowchart LR
  Input[Interactive command] --> Parser[Validate entire batch]
  Parser --> Goal[Freeze stopped start and target]
  Odom[Filtered odometry] --> Feedback[Along/cross/heading feedback]
  Goal --> Feedback
  Feedback --> Limit[Velocity and acceleration limits]
  Limit --> Cmd[cmd_vel]
  Cancel[Stop / stale odom / jump / timeout] --> Zero[Zero output and cancel batch]
  Scan[scan_filtered + TF] --> Observation[Stopped laser observations]
  Observation --> Files[Point JSON and audit JSONL]
  Odom --> Files
```

The runtime reuses the existing odom continuity/freshness checks, detects competing
velocity publishers and uses the same local lock as other supervised tools. This
is not a hardware emergency stop and cannot prevent an independent publisher from
commanding the base. The base watchdog must remain enabled. Plausible accumulated
odom drift and small undetected resets cannot be excluded by jump checks.

## Verification

```bash
python3 -m unittest discover -s test -p 'test_*.py'
# Dedicated localhost domain only; synthetic odometry, no real robot:
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=197 python3 test/verify_motion_console.py
```

See `motion_model.py` for frame conversion and correction logic and
`motion_console.py` for ownership, cancellation, observation and persistence.
The integration fixture verifies the installed ROS executable; it is not a maze
simulation and does not validate wheel slip, braking distance or obstacle clearance.
## Optional right-wall start preparation

Before starting the console, prepare the stopped origin with:

```bash
ros2 run distance_controller odom_trial \
  --start prepare --alignment-wall right \
  --positioning-wall right --right-target-distance 0.26 \
  --laser-log-only
```

At the `odom>` prompt, enter `quit`, then start `motion_console`. Do not enter
`next` or `run` when only preparing the origin.

`alignment-wall` selects heading alignment. `positioning-wall right` additionally
uses the right wall for lateral positioning instead of left/right centering.
The 0.26 m target is measured from the base_link origin, not the body edge or
laser origin. Rear positioning retains its 0.28 m target. Missing left returns
are observed but do not invalidate this preparation mode. Missing right/rear
returns still stop preparation, as do stale data and existing travel limits.
The default `positioning-wall center` preserves the two-side centering behavior.

Local synthetic verification (no hardware):
```bash
WALL_CASE=odom_trial_right_only python3 test/verify_wall_session.py
WALL_CASE=odom_trial_right_only PREP_FAIL=right python3 test/verify_wall_session.py
WALL_CASE=odom_trial_right_only PREP_FAIL=rear python3 test/verify_wall_session.py
WALL_CASE=odom_trial_prepare python3 test/verify_wall_session.py
```
These checks verify required-wall selection, correction direction, stopped
handoff and fault stops; hardware placement accuracy remains unverified.

## Precise P01 preparation (experimental measurement acceptance)

Add `--precise-start` to the right-wall preparation command:

```bash
ros2 run distance_controller odom_trial \
  --start prepare --alignment-wall right \
  --positioning-wall right --right-target-distance 0.26 \
  --precise-start --laser-log-only
```

This opts into 5 mm distance and 0.5 degree heading tolerances (or tighter C++
parameter overrides), caps positioning at 0.02 m/s and at 0.008 m/s within 2 cm
of both distance targets. It does not alter the default route or legacy mode.

Precise preparation defaults to a rear sampling half-angle of 10 degrees; legacy
preparation retains 5 degrees. An explicit C++ ROS parameter
`rear_window_half_angle` (radians) overrides either default. The startup log prints
the effective window. This changes the estimator, not the acceptance tolerances.
The stationary P01 bag showed a rear median range of 14.95 mm at 5 degrees and
5.08 mm at 10 degrees, with a 3.59 mm median shift. The sensor transform was
confirmed as translation (0.020, 0, 0.173) m and yaw pi. Reduced variation is
offline evidence only; physical accuracy and repeated initialization remain
to be validated on the robot.

On entering tolerance, command zero; require measured translation below 0.003 m/s,
rotation below 0.01 rad/s, and a 0.5 s hold. Then collect 15 distinct post-hold
scans. More than 3 mm odom displacement or 0.005 rad heading drift during this
window discards the evidence and starts a new hold. Existing stale-data, required
right/rear validity, preparation travel and overall time limits remain active.

Each frame contains base-origin right/rear distances and a fresh right-wall angle.
Joint samples more than 2 cm or 2 degrees from the batch medians are discarded;
invalid heading fits count against consensus. Require at least 12/15 retained,
right/rear retained ranges each at most 1 cm, and angle range at most 2 degrees.
Only then compare the retained medians with 0.26/0.28 m and zero wall angle.
Stability and target accuracy are separate checks. Sensor bias or a consistently
wrong wall cannot be detected by these statistics alone.

An unstable batch keeps zero velocity while another batch is collected. Each
verification attempt has a 5 s budget; at most three stable-but-off-target
refinements are allowed. Heading refinement reacquires the wall direction.
`START_VERIFY_TIMEOUT` or `START_REFINEMENT_LIMIT` fails without recording P01;
no tolerance is widened automatically. Valid completion prints `START_VERIFIED`
and the usual `preparation complete` handoff. `START_VERIFY` reports medians,
spreads and retained counts; the legacy `Centered A` distances are the latest
single scan and can differ from the accepted median.

This is initialization-only. `motion_console record` has not yet been converted
to this batch policy. The tool still captures its own current stopped P01 after
handoff. Save the preparation terminal output as the multi-frame evidence.
These are measurement acceptance targets, not a claim of 5 mm physical accuracy.

Synthetic checks additionally support `PRECISE_START=1`, `PRECISE_NOISE=spikes`
or `PRECISE_NOISE=unstable` with `WALL_CASE=odom_trial_right_only` in the existing
fixture. No real hardware is driven by these localhost-domain tests.

## Continuous P01 to P02 repeatability trial

After successful precise initialization, quit `odom_trial` and start a **new**
`motion_console --laser-log-only` session without moving or resetting odom in between.
Then enter these commands separately:

```text
path_plan
path P02
```

`path_plan` is read-only. `path P02` loads the installed
`config/task6_p01_p02_path.json`, transforms its relative coordinates using this
session's recorded P01, and follows one continuous path. It does not reuse old
absolute odom coordinates. Input evidence is the user's measured P01, P01_F10
(0.100728402, 0.000929167 m), and P02 (0.840238840, -0.002600029 m).
Each measured edge is subdivided into <=5 cm edges, preserving P01_F10 exactly:
19 points total, including start and end. The heading stays at this session's
P01 heading; the taught P02 yaw error of -1.467 degrees is deliberately excluded.

The tracker projects onto the polyline, advances monotonic arc progress, and aims
5 cm ahead. It never stops at intermediate points or returns to a passed one.
Tracking uses a fixed P01 reference, body-frame velocity conversion and existing
acceleration limits; endpoint approach slows proportionally. At P02 require
Euclidean position error <5 mm, heading error <0.01 rad, and stopped feedback for
1 s; a further stopped check precedes reporting success. This is an odom criterion.
After success, record actual P02 with the existing stopped laser snapshot and print
`P02 comparison`. The JSON/JSONL contain actual points, full interpolated path,
current origin and transformed target, progress, completion/failure and observations.

Start requires the current pose within 1 cm / 1 degree of this session's P01.
A second `path P02` after P02 is recorded is rejected. After partial movement or
failure there is no automatic path resume: return near physical P01 under operator
control, reinitialize, then start a new console session. Off-path distance >5 cm,
odom continuity/freshness faults, conflicting velocity publishers and timeouts stop
motion. `stop`, Ctrl+C, quit/EOF and SIGTERM retain existing cancellation behavior.
Laser remains observation-only; interpolation does not certify free space.

This first entry supports only a reviewed forward near-straight P01-P02 path
(each edge within 10 degrees of P01 forward, total <=2 m). Do not edit it into
corners, reversals or a general maze route. Endpoint-only teaching cannot recover
an unrecorded avoidance maneuver. No automatic return to P01 is implemented.

For three independent trials, repeat initialization and create a new point file
each time; retain all console output and JSON/JSONL. Compare P02-relative endpoint,
heading, available wall observations and visual location. A small odom residual
alone does not establish physical repeatability or correct the earlier laser/odom
scale discrepancy. Ordinary `record` still uses a snapshot, not 15-frame acceptance.

```bash
# Local isolated synthetic test only, no robot domain:
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=197 python3 test/verify_teaching_path.py
```

### Short scan gaps during precise preparation

With `--precise-start`, a scan gap longer than 0.5 s immediately commands zero
and prints `SCAN_RECOVERING`. Recovery has a fixed 2 s budget. Pre-stop, duplicate
and queued scans older than 0.2 s cannot establish recovery. Require at least
three distinct valid post-stop scans spanning 0.3 s, fresh odometry, and measured
speed below 0.003 m/s / 0.01 rad/s. Then `SCAN_RECOVERED` starts wall-heading
acquisition and positioning again, including the full 15-frame final verification.
No prior partial batch survives the gap. Existing stage, total-time and travel
limits remain in force. Persistent loss or old frames yield
`SCAN_RECOVERY_TIMEOUT`; required-wall invalidity outside recovery still fails.
Legacy preparation and path execution do not acquire this recovery behavior.

### P01-P02 speed selection

Start a new console at the initialized P01 using `--speed 0.06` for 6 cm/s.
The default remains 0.04 m/s. Path lookahead is max(0.05 m, speed / 0.8), so
the requested cruise speed is reachable; interpolation spacing stays at most
5 cm. The endpoint remains fixed, with proportional deceleration, position
error below 5 mm, heading error below 0.01 rad and a 1 s stopped hold.
These are odometry acceptance criteria, not guaranteed physical accuracy.
