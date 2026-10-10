# Supervised P01-P05 odom trial

Separate commissioning entry point; existing wall-guided action_session and
checkpoints are unchanged. **No laser obstacle stop in this trial.** Supervise
each action; Ctrl+C stops. Keep teleop and other cmd_vel publishers stopped.

| Action | Relative geometry |
|---|---|
| P01-P02 | forward .90 m |
| P02-P03 | forward .59 m, then forward .20 m + right .04 m |
| P03 turn | clockwise 90 degrees |
| P03-P04 | forward .44 m |
| P04-P05 | right .287 m |

The .44 m candidate combines observed .382 m approach, .028 m adjustment,
and the user-requested 3 cm forward correction at P04. P04-P05 is purely
rightward in the planned heading; no extra forward correction at P05.
P05 inherits the corrected P04 origin. Physical validation is pending.
Lengths use previous nominal/observed evidence, rounded deliberately. Accumulated
lateral odom drift is not interpreted as intentional geometry. No continuous
teleop trace was supplied. Late 4 cm right offset is a proposed shape, not a
measured collision-free path. Replacing left-wall correction with odom feedback
does not establish equivalent physical paths.

Configuration: config/task6_odom_p01_p05.json. Offsets are forward/left in each
segment's planned heading. All targets derive from the preceding planned target
and the new P01 origin; editing an earlier displacement moves downstream targets.
Actual stopping errors never silently rebase the route.

Interpolation spacing <=5 cm; intermediate targets have no dwell and advance
within 1.5 cm. Key points stop for .5 s. Body vx/vy close position error and wz
closes heading error; large yaw error pauses requested translation. Default
speed .06 m/s, with bounded acceleration. Turns are separate; small turn-center
drift is accepted within 3 cm, over 4 cm aborts. Real odom drift remains a limit.

## Start and commands

Physically place the robot at P01's start area and stop existing controllers:

    ros2 run distance_controller odom_trial --start prepare --laser-log-only

Preparation reuses existing laser left-wall alignment/centering once, then
captures P01. It cannot return an arbitrary P05 position to P01.
--start current adopts the current stopped pose as P01 without preparation.
Omitting --start only previews the route.

At odom> enter one next for each table row. Inspect before each next.
Available: next, resume, status, measure, quit.
resume retains the same target/index only within this process after interruption.
No continuous run, automatic back, or cross-process resume in this first slice.
Old action_session checkpoints cannot be loaded.

Odom stale/nonfinite/jump/frame/time faults, competing publishers, endpoint
errors and timeouts still stop. Laser loss and near returns do not stop motion.
Preparation still requires laser. Afterward laser snapshots are recorded at
stopped key-point boundaries, missing scans marked unavailable, without warning
spam. Audit odom_trial_<timestamp>.jsonl contains actual pose, intermediate target,
commands and observations; it is not a restart checkpoint.

## Validation boundary

Pure tests cover rotated origins, downstream edits, late offset, turn direction,
body-frame feedback, invalid inputs and acceleration bounds.
The isolated ROS synthetic fixture executes five actions with laser absent after
the first 2.5 cm and tests odom-stale stopping. This validates wiring, not physical
clearance or odom accuracy. Initialization reuses existing code.
Real P01-P05 acceptance is pending; no Task6 tag or independent learner PASS.

After physical validation, teleop recording of remaining key points in one
uninterrupted odom epoch is the next slice; a recorder is not implemented here.
