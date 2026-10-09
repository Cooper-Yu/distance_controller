# distance_controller Control Flow

This page describes the current implementation, updated 2026-10-09. Interfaces are in the class documentation, control code in `src/distance_controller.cpp`, and the entry point in `src/main.cpp`.

## Input, state, and output

`/odometry/filtered` -> `on_odom()` -> stored feedback and steady receipt time -> `on_timer()` -> `/cmd_vel`.

The single-threaded executor serializes subscription and 50 ms wall-timer callbacks. Feedback callbacks store data; timer callbacks validate, calculate, and publish. Pose uses odom coordinates; feedback twist and published commands use body coordinates.

## One control tick

\image html control_flow.svg "Current on_timer() control flow" width=900px

The diagram uses actual function names. A stored Graphviz source produces the SVG; browser Mermaid support is not required.

| Stage | Behavior |
| --- | --- |
| Fault, feedback, and time | Stop on a latched fault; wait for first feedback; latch a fault after a 0.5 s receipt timeout or backwards node time; reset timing state and stop this tick after a forward interval above 0.2 s |
| Initial scan guard | Stop before first valid scan/TF; latch faults on subsequent invalid/stale input or preparation time/travel limits |
| Initial alignment | Scene 2 rotates toward the measured right-wall direction, then verifies continuous standstill before laser side/rear positioning; scene 1 bypasses |
| Initial centering | Compare median side body-y and rearward body-x distances; adjust x/y with yaw hold; verify standstill and capture A |
| Target and error | Capture current A after centering in scene 2; initialize route targets from that fixed origin, heading, and cumulative displacement; stop if unavailable, otherwise read pose and compute ex/ey |
| Heading recovery | Scene 2 pauses translation for large yaw errors or corrects yaw at an arrived position; drift during dwell revokes completion |
| Completed segment | Stop through dwell; executor returns Completed; on_timer() calls advance_route() once |
| Position acceptance | Inside 0.01 m, stop, reset PID, and check continuous feedback standstill |
| PID timing | Seed missing history and stop; stop for zero interval; reset and stop for negative or greater-than-0.2 s interval; calculate only with valid dt |
| Command | PID -> save raw values -> speed limit -> acceleration limit -> odom-to-body rotation -> scene-2 yaw command -> publish |
| Log | Print when due; logging does not throttle command computation or publication |

## Settling, dwell, and advancement

`settling_` means continuous standstill is being timed. Position error must remain below 0.01 m, planar feedback speed below 0.01 m/s, and absolute yaw rate below 0.02 rad/s, and scene-2 yaw within heading_tolerance_ for at least 0.5 node-clock seconds.

Initial scene-2 preparation requires fresh side/rear-wall scans and a fixed laser-to-body TF. Rotate first, then adjust both planar axes with yaw hold. Center offset, rear-distance error, yaw and measured standstill must remain accepted for alignment_settle_duration; capture current odom as A. Rear distance targets 0.28 m from base_link; no old absolute A is used.

For route arrivals, `segment_completed_` then disables tracking and starts extra dwell, default 1 node-clock second. When dwell ends, advance the index, reset PID/segment flags, and end this tick; initialize the next target on the next tick. After the final dwell, keep zero velocity and call `rclcpp::shutdown()`.

Feedback receipt timeout uses steady time. PID, settling, and dwell use node time. Stops bypass the acceleration ramp and clear its history. Scene 2 controls heading toward the measured right-wall direction; initial centering uses laser, but route obstacle avoidance is not implemented; rear positioning is initialization only.

## Diagnostic data flow

`execute_current_segment()` creates `ControlDiagnostics data` and fills six input fields: position, yaw, errors, and pid_dt. `compute_and_publish_command(data)` reads errors/yaw/dt and writes six planar velocity fields and data.wz_robot into the same object. `log_control_state(data)` reads those results.

## Maintenance and evidence

```bash
dot -Tsvg docs/diagrams/control_flow.dot -o docs/diagrams/control_flow.svg
doxygen docs/Doxyfile
```

The current code completed 10/10 segments and exited normally in the local empty-world run on 2026-10-09. That normal-route evidence does not replace fault injection, cloud, or hardware acceptance.


## Route and preparation boundaries

Motion factories in `route.hpp` describe distance, speed and dwell. `compose_route()`
validates connected names starting at A. `route_execution.cpp` owns target freezing
and the nonblocking segment tick; `on_timer()` owns advancement. AB, AB/BA, AB/BC and
the default four edges share one executor. Each target stays fixed through tracking,
standstill and dwell.

`preparation_guard.cpp` owns steady-time limits: measurement 20 s, alignment 30 s,
positioning 30 s, plus total 60 s. Stages advance once; poor fits do not reset deadlines.
Faults distinguish stage/total deadlines, travel, invalid scans and stale scans.
PID, settling and dwell still use node time.


## Manual and feedback dispatch

After initialization, manual WAITING bypasses segment execution: publish zero linear
velocity and hold the persistent heading, while odom/time/fault guards remain active.
A validated service request or resume transitions to exactly one active step.
Busy/faulted requests are rejected. On completion, record the endpoint and either
wait for another request, advance automatically, or exit according to mode.

Before PID, apply_step_feedback checks active-step duration/path length and required
laser windows. Position goals use frozen odom targets. Optional side centering replaces
lateral odom error. Front goals use front-body clearance instead of longitudinal odom
error. Both share heading recovery, measured standstill and dwell. Required laser loss
latches stop without fallback. The full interface contract is in manual_steps.md.

## History and continuation

Scene 2 records A once, captures each actual step start at target initialization and commits its stopped endpoint only after dwell. Completed return legs pop active ancestry but remain in the audit. A return batch uses the ordinary executor and goes to each recorded start in sequence. WAITING checks displacement before motion acceptance; excessive drift locks zero output. See history_return.md.

## Scene 2 feedback interruption

The odom guard runs before preparation or route execution. A gap over 0.5 s
enters `RECOVERING`, publishes zero, clears settling/dwell qualification, resets
PID, and preserves the target and route index. Within a fixed 2 s steady-clock
budget, feedback must remain fresh and stopped for 0.3 s with unchanged frames
and bounded pose change. Success resumes the original target on a later tick;
failure latches zero commands. With an independently verified base watchdog and
`base_command_watchdog_verified:=true`, terminal recovery failure exits with code 2.
Preparation and segment deadlines are not restarted. History and new motion
requests cannot advance an interrupted step. See the README for exact thresholds.