# Supervised reverse survey (Task6)

`survey_return` executes exactly one reviewed action and exits. This is a survey
helper, not the Task6 automatic solver or an obstacle-avoidance controller.
It uses the existing distance and turn controllers; it does not reset odometry
or perform wall alignment. `turn_controller` must also be installed.

The starting condition is P15 **after** the observed final left 180-degree turn.
Step 1 undoes that turn clockwise. Step 2 moves backward toward P14, preserving
the original body orientation/swept path. Later turns undo the outward turns.
Do not skip step 1 and then execute step 2 unchanged.

```bash
# Preview only: no ROS node, controller, or movement.
ros2 run distance_controller survey_return

# Execute only the selected action; type RUN 1 after checking its physical start.
ros2 run distance_controller survey_return --step 1 --execute

# Inspect position/clearance before choosing the next action.
ros2 run distance_controller survey_return --step 2 --execute
```

Existing controllers must be stopped before execution. The helper checks for
publishers on `/cmd_vel` and an existing step service before starting its own
controller; no new publisher may be started concurrently. Each translation
adopts the current fresh stopped pose, moves at 0.03 m/s, checks the expected
WAITING endpoint with empty queues, then requests a clean shutdown. Turns use
0.20 rad/s and require exit 0. A failed request is not retried. A timeout or
interruption stops the owned process group and sends repeated zero commands.
The previously verified base command watchdog remains necessary for loss of
this process/host. Each execution writes a timestamped JSON audit in the current
directory. Controller diagnostics remain visible in the terminal.

Distances come from **consolidated outward commands**, not exact measured reverse
paths. Multiple small moves stop within tolerance, so their sum need not match
one long move. P06/P08/P09 adjustments were made out of order; their consolidated
segments are explicitly marked provisional. Every step requires visual review.
A completed action is not proof of obstacle clearance or exact waypoint arrival.
No automatic resume index is inferred; do not repeat an executed step blindly.

A shorter exploration move can be selected explicitly:

```bash
ros2 run distance_controller survey_return --step 2 --distance 0.10 --execute
```

That only moves 0.10 m. It does **not** mark the full P15->P14 leg complete or
compute a remaining distance. Check the actual position before another action.

## Coordinate evidence

The survey used three odom epochs: original A/P03, reset at physical P03, and
reset near P09 after its first right correction. A controller restart alone does
not reset odometry. Old absolute coordinates must not be concatenated or used
for automatic return after a reset. The helper holds the current heading per
translation; drift can accumulate across restarted controllers. This provisional
reverse route needs physical validation before use as a reusable route.

## Local checks

- Seven unit tests (including distance observation): reverse geometry from an arbitrary starting pose, final-turn
  reversal, strict endpoint status, preservation of provisional corrections.
- Synthetic ROS feedback: backward motion, a right turn, missing odom, conflicting
  command publisher. These are orchestration checks, not a real maze test.

```bash
python3 test/test_survey_return.py
# Source ROS and the workspace first; this fixture uses localhost domain 177.
python3 test/verify_survey_return.py
```
## Four-direction distance records

Read distances without starting a controller or publishing motion:

```bash
ros2 run distance_controller survey_return --measure
```

Each executed action also captures a new scan before motion and after successful
completion. The terminal shows FRONT / LEFT / RIGHT / REAR valid/total counts and
minimum, median and maximum range in meters. The same data is saved under
`laser_observations` in the action JSON. Read-only snapshots are saved as
`laser_observation_<timestamp>.json` in the working directory.

Directions are relative to `base_link`, using TF at the `/scan_filtered` message
stamp. This handles the observed 180-degree laser mount without hard-coded
left/right swapping. Each window is +/-5 degrees. At least three finite returns
and 50% coverage are required per window. No TF, missing scans, stale stamps,
invalid geometry, or sparse returns are explicitly unavailable, never infinity
or zero clearance. A snapshot waits at most two seconds for a new stamped scan;
stamp and receipt ages must be at most 0.5 seconds. This entry targets the real
robot's system time, as do the scene-2 controllers.

Distances are **ray lengths from the laser sensor**, not vehicle-edge clearance
and not fitted perpendicular wall distances. TF translation is recorded for
interpretation, but body dimensions are not subtracted. Sensor frame, stamp,
age, TF rotation/translation and observation phase accompany each snapshot.
Observations are read-only: missing laser data is reported but does not veto a
motion authorized by the operator. They do not add autonomous obstacle avoidance.

Seven unit checks and six synthetic ROS cases cover reverse control, 180-degree
mounting, angular wraparound, invalid returns/tilt, read-only measurement, stale
scans and before/after JSON persistence. Physical distances and the return route
still require cloud verification.
