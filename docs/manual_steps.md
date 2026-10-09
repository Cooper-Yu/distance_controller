# Planar steps and manual exploration

## Scope and ownership

Scene 2 supports named routes and optional manual continuation. Every translation,
including backward and lateral motion, holds the same initial right-wall-derived
heading reference. A new command never silently accepts accumulated yaw drift as a
new reference. Large yaw errors suspend translation; small errors are corrected
while moving. Idle manual mode commands zero translation and corrects yaw drift.

Only one action executes at a time. The service accepts descriptions; the timer
owns normal velocity publication. Cancel is an explicit fault/stop override.
Task3 calls for a separate `turn_controller.cpp` / `TurnController`; Task4 reuses
that program with real-robot waypoints. Those controllers are not implemented here.
A future sequencer must deactivate translation heading hold while a turn owns yaw,
and update the held reference after a verified turn. Two independent controllers
must not publish competing commands. This version rejects `turn` requests.

## Start with AB and pause at B

Build and source the overlay, then run:

```bash
ros2 run distance_controller distance_controller 2 --ros-args \
  -p forward_distance:=0.93 -p lateral_distance:=0.516780 \
  -p heading_tolerance:=0.01 -p 'route:=[AB]' -p manual_mode:=true
```

This uses the latest user trial overrides; package defaults are still 0.90 m,
0.516780 m and 0.02 rad. Initialization remains mandatory. AB stops and verifies
standstill/dwell before `Manual WAITING`; the node then stays alive. Add
`-p start_paused:=true` to pause immediately after A initialization instead.
Manual continuation is only available in scene 2. Without manual mode, the node
still exits after the selected route.

From a second terminal with the same ROS environment/domain and overlay:

```bash
ros2 run distance_controller step status
ros2 run distance_controller step forward 0.20 --label D
# Wait for WAITING before choosing the next command:
ros2 run distance_controller step backward 0.10
ros2 run distance_controller step left 0.05
ros2 run distance_controller step right 0.05
ros2 run distance_controller step relative --x 0.10 --y -0.05
ros2 run distance_controller step target --x 0.40 --y 0.20 --label E
ros2 run distance_controller step finish
```

These are alternatives to execute one at a time, not a script to queue blindly.
`ACCEPTED` means accepted for execution, not arrived. `status` reports
PREPARING/RUNNING/WAITING/FAULT, the endpoint label, pending segments, current pose
and held heading. A new step is rejected while busy, faulted, uninitialized, moving,
misaligned, or without fresh odometry. Requests do not accumulate in a hidden queue.
Each new ad-hoc relative step anchors to the current accepted pose; named route
segments retain their planned geometry. `Endpoint D recorded=(x,y), yaw=...` logs
the actual stopped pose after dwell. Save the controller log as trial evidence.

`resume` executes the next preconfigured segment; it is required when that segment
is still pending. New ad-hoc steps are accepted only after the configured route has
finished. `cancel` immediately latches a stopped fault and requires node restart.
`finish` shuts down from an aligned, stopped WAITING state. Ordinary Ctrl+C also
ends the process; the ROS runtime and robot's command watchdog remain external.

## Frames, bounds and per-step choices

- Default `--frame route`: fixed axes captured at A; forward/backward are +/-route x,
  left/right are +/-route y.
- `--frame heading`: axes defined by the persistent held heading, not changing actual
  yaw. They currently coincide with route axes; a future accepted turn can separate them.
- `target --x ... --y ...`: absolute odom position in the current odom reference;
  it does not request a turn. Do not reuse coordinates from a different odom reference.
- `--speed`: positive planar cap, additionally bounded by global max_speed; omitted/0
  selects global cap. `--dwell` is literal nonnegative node-clock seconds (CLI default 1).
- `--timeout`: positive steady seconds for execution including recovery/settling/dwell;
  omitted/0 selects 60 s.
- `--max-travel`: positive accumulated odom path-length bound. Omitted/0 selects
  max(0.30, nominal distance + 0.25) m; front goal uses a 1 m nominal search plus 0.25 m.
- `--side-centering`: explicit optional left/right wall correction, only for forward
  or backward steps (also allowed with a front goal). Both side windows must remain
  fresh and valid; loss faults the step. It replaces lateral odom tracking and also
  participates in arrival. Lateral moves cannot enable this opposing correction.

Default steps ignore route laser availability. No automatic side centering or laser
obstacle stop is added to ordinary translation. In manual waiting, only heading is
held; position is not automatically recovered if the robot is physically displaced.

## Use front clearance as the arrival condition

First measure and configure `front_body_extent`: the frontmost body coordinate in
meters along base_frame x. It defaults to -1 (unconfigured); front goals are rejected
until a positive measured value is provided. The existing laser filter box is not
proof of the real body outline. For example, **only if the measured extent is 0.20 m**,
add `-p front_body_extent:=0.20` to the controller startup command. Then at WAITING:

```bash
ros2 run distance_controller step front_wall 0.20 \
  --speed 0.03 --max-travel 0.50 --timeout 30 --label D
```

The distance is **front-body clearance**, not raw laser range. Scan points are
transformed into base_frame using TF. The front +/-5-degree sector needs at least
six valid points, 50 percent coverage and median absolute deviation <=side_max_mad.
Its median body-x coordinate minus front_body_extent gives measured clearance.

The step only approaches forward, holds heading, and retains its starting lateral
line unless side centering is selected. It slows as clearance error decreases;
body-forward speed is also capped by centering_max_speed (default 0.03 m/s).
Arrival requires the combined position/clearance error below 0.01 m, heading within
tolerance, measured standstill for 0.5 s, then dwell. Already-too-small clearance
(target minus more than 0.01 m), invalid/stale required scans, timeout or travel
limit latch a stopped fault. The controller never backs away automatically.

This narrow sector is an arrival measurement, not whole-footprint collision avoidance.
It cannot prove wall identity or protect against obstacles outside the sector. The
previously observed 0.15 m minimum laser range cannot measure a 0.03 m raw ray.
A 3 cm body-clearance target is also not a validated hardware setting; body geometry,
measurement error, stopping distance and physical clearance need separate validation.

## Add or tune a named segment

AB/BC/CB/BA have independent factory methods in route.cpp. Any new two-letter edge,
such as BD, can be defined using explicit dx/dy parameters, without editing PID or
the executor. Names must join continuously from A. Unknown edges without coordinates,
zero relative steps and disconnected routes reject startup.

```bash
ros2 run distance_controller distance_controller 2 --ros-args \
  -p forward_distance:=0.93 -p lateral_distance:=0.516780 \
  -p 'route:=[AB, BD]' -p manual_mode:=true \
  -p segments.BD.dx:=0.20 -p segments.BD.dy:=0.0 \
  -p segments.BD.speed:=0.05 -p segments.BD.timeout:=30.0
# At B:
ros2 run distance_controller step resume
```

An installed equivalent example is `config/route_ab_bd.yaml`. From the repository:

```bash
ros2 run distance_controller distance_controller 2 --ros-args \
  --params-file config/route_ab_bd.yaml
```

Each `segments.ID` accepts dx, dy, speed, dwell, timeout, max_travel, side_centering,
completion (`position` or `front_wall`) and front_clearance. A front-wall segment
must describe forward motion and also needs front_body_extent. Ordinary segments
advance the planned endpoint, preventing stopping residuals from accumulating.
Sensor-defined arrival/side correction deliberately records the actual endpoint as
the next anchor because that step's final geometry is measured, not predetermined.

## Initialization decomposition

`on_scan()` validates message/TF; `measure_wall_windows()` measures independent sectors.
`handle_centering_guard()` applies preparation bounds. `handle_initial_alignment()`
rotates and settles. `handle_initial_positioning()` combines the separate
`compute_centering_velocity()`, `compute_rear_position_velocity()` and
`compute_heading_hold_velocity()` outputs with one vector speed cap.
`record_route_origin()` records A after joint acceptance. Execution remains
nonblocking and the initialization order is unchanged.

## Service and testing boundary

The CLI calls `distance_controller/srv/ExecuteStep` at `/distance_controller/step`.
Use `--service /namespace/distance_controller/step` for namespaced controllers.
No background client heartbeat is required: an accepted step is bounded by its
execution limits. Client timeout means acceptance is unknown; query status before
retrying instead of repeating a possibly accepted request.

Local synthetic tests and Task1 regression do not constitute Task2 cloud/hardware
or independent learner acceptance. See the test directory for actual test entry points.
