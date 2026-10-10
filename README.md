# distance_controller

For Task6 wall-referenced single-step trials, see [Wall-guided commissioning](docs/wall_session.md).
Use `action_session --wall-guided --start prepare` only at physical P01.

## Editable action sessions

For Task6 route exploration, use [one-origin action sessions](docs/action_session.md): separate translation/turn actions, regenerate future targets, execute one step, return to actual stops, edit and retry. Start with `ros2 run distance_controller action_session` for a motion-free preview. The supplied route remains provisional; Task1/2 tags are unchanged.

A ROS2 Humble planar distance controller for ROSBot XL. It uses odometry feedback, planar PID, speed and acceleration limits, and odom-to-body velocity conversion. Each segment ends with verified standstill and an extra dwell; the node stops and exits after the final segment unless manual continuation is enabled.

The Task1 acceptance snapshot is Git tag `task1` (`e1a26a6`), which remains unchanged. Scene 2 supports independently configured segments, manual continuation, completed-step history and sequential return. Current history/return features have local test coverage; their cloud/hardware acceptance remains pending.

## Task2 accepted course entry

```bash
ros2 run distance_controller distance_controller 2
```

The previous default four-segment route completed on the real robot on 2026-10-09;
the learner confirmed no wall contact and final stopping. The `task2` snapshot
uses 0.90 m forward/backward, 0.516780 m lateral travel, and 0.01 rad heading
tolerance after the requested configuration unification. This exact combination
still needs real-robot revalidation. It runs automatically and exits after the final dwell. Official
grading remains pending. See the [Task2 acceptance record](docs/task2_acceptance.md)
for measured results, limitations and reproduction instructions.

The independent YAML profile below uses the same distances and heading tolerance,
but enables manual mode;
it is not required for the accepted course command.

## Scene 2 quick start: independent segments

Update and rebuild in the cloud workspace:

```bash
cd ~/ros2_ws/src/distance_controller
git pull --ff-only origin master
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select distance_controller
source install/setup.bash
```

Start with AB using the installed profile:

```bash
ros2 run distance_controller distance_controller 2 --ros-args \
  --params-file "$(ros2 pkg prefix distance_controller)/share/distance_controller/config/segments.yaml" \
  -p 'route:=[AB]'
```

The profile selects independent configuration and manual mode. AB is 0.90 m;
BC/CB are 0.516780 m laterally, with heading_tolerance=0.01 rad. Each edge owns
its displacement, speed, dwell and feedback policy. `route` selects order only.
Do not combine this profile with forward_distance/lateral_distance overrides.
Change an individual edge with, for example, `-p segments.AB.dx:=0.90`.

Initialization aligns to the right wall, centers left/right, adjusts rear distance
and records A. AB then executes and stops at B. Keep this controller terminal running.
From a second terminal, source ROS and the same overlay, then inspect the state:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run distance_controller step status
ros2 run distance_controller step history
```

Once WAITING, choose one action and wait for completion before submitting another:

| Command | Result |
| --- | --- |
| `ros2 run distance_controller step forward 0.10 --label D` | Explore a point 10 cm forward while holding heading |
| `ros2 run distance_controller step backward 0.05` | Move backward 5 cm |
| `ros2 run distance_controller step left 0.05` | Move left 5 cm |
| `ros2 run distance_controller step right 0.05` | Move right 5 cm |
| `ros2 run distance_controller step backtrack 1` | Return along one completed edge; 1 means a segment count, not meters |
| `ros2 run distance_controller step return_to A` | Follow the active history chain back to A |
| `ros2 run distance_controller step return_to --visit-id 1` | Select an exact historical visit when labels repeat |
| `ros2 run distance_controller step resume` | Execute the next pending configured segment |
| `ros2 run distance_controller step finish` | Exit from an accepted stopped WAITING state |
| `ros2 run distance_controller step cancel` | Stop and latch a fault; restart required |

An accepted return replaces pending configured segments. Return legs proceed in order
with arrival/standstill/dwell checks, then enter WAITING at the selected destination.
History exists only in the current process and odom reference; restart recovery and
obstacle avoidance are not implemented. Waiting displacement beyond 0.02 m locks a
stop instead of re-running A initialization at an intermediate point. Turn control
remains reserved for Task3/4; default planar gains remain P=1.5, I=0, D=0.

See [Independent segments and history return](docs/history_return.md) for visit IDs,
return-policy conversion and bounds, and [Planar steps and manual exploration](docs/manual_steps.md)
for frames and optional laser arrival. Commands are examples to select, not a script
of consecutive unobserved robot motions.

## Layout and reading order

```text
distance_controller/
├── README.md
├── CMakeLists.txt
├── package.xml
├── config/segments.yaml
├── config/route_ab_bd.yaml
├── srv/ExecuteStep.srv
├── include/distance_controller/
│   ├── distance_controller.hpp
│   ├── route.hpp
│   └── route_history.hpp
├── src/
│   ├── main.cpp
│   ├── distance_controller.cpp
│   ├── initial_centering.cpp
│   ├── right_wall_heading.cpp
│   ├── preparation_guard.cpp
│   ├── route.cpp
│   ├── route_execution.cpp
│   ├── step_configuration.cpp
│   ├── step_feedback.cpp
│   ├── manual_steps.cpp
│   ├── route_history.cpp
│   └── history_control.cpp
├── docs/
├── test/
└── tools/step.py
```

| File | Responsibility |
| --- | --- |
| `main.cpp` | Initialize ROS, parse the optional scene, construct the node, and spin serially |
| `distance_controller.hpp` | Class declarations, interface documentation, state descriptions, and member defaults |
| `distance_controller.cpp` | Timer dispatch, odometry/time guards, PID, speed limits and heading control |
| `right_wall_heading.cpp` | Startup right-wall line fitting and temporal consistency |
| `initial_centering.cpp` | Scan windows, initial side/rear positioning and A capture |
| `preparation_guard.cpp` | Initialization stage deadlines, scan validity and travel limits |
| `route.hpp` / `route.cpp` | Motion descriptions, named-edge factories and validation |
| `route_execution.cpp` | Frozen targets, per-tick execution, standstill/dwell and route advancement |
| `step_configuration.cpp` | Independent/legacy startup configuration and per-edge parameters |
| `step_feedback.cpp` | Selected odom/laser feedback, step time/travel bounds and front speed cap |
| `manual_steps.cpp` | Service requests, WAITING state and endpoint logging |
| `route_history.hpp` / `route_history.cpp` | Completed traversal audit, visit IDs and pure reverse-plan construction |
| `history_control.cpp` | History/controller integration and continuation-position checks |
| `config/segments.yaml` | Independent segment geometry and policies; route selects order |
| `srv/ExecuteStep.srv` / `tools/step.py` | Typed request interface and installed command-line client |
| <code>\ref build_cmake "CMakeLists.txt"</code> / <code>\ref build_package_xml "package.xml"</code> | Build targets and ROS dependencies |

Read `main()`, then the class interface and state, then follow `on_timer()` into its helpers. This README is the package entry; no duplicate README is needed under `src/`.

## Task1 build and simulation

Use ROS2 Humble with the ROSBot XL simulation dependencies already available:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select distance_controller
source install/setup.bash
```

Terminal 1, in the configured course environment:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch rosbot_xl_gazebo empty_simulation.launch.py
```

The course environment's launch default has already been changed to `mecanum=True`. The controller does not configure the simulator's drive mode. A fresh environment must provide the same drive configuration.

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run distance_controller distance_controller
```

No scene argument selects scene 1, equivalent to an explicit `1`. Scene 1 defaults to simulation time and the ten-segment route. Both terminals need matching ROS communication settings; odometry and a simulation clock must be available.

## Scene 2 compatibility mode: shared distance parameters

Without an independent profile, `segment_configuration=legacy` preserves the older shared-length interface. Its defaults are forward 0.90 m, right 0.516780 m, left 0.516780 m and backward 0.90 m. For new experiments, use the independent quick start above.

Scene 2 reads `forward_distance` (default 0.90 m) and `lateral_distance` (default 0.516780 m) once at startup. Both must be finite and positive. These parameters define route lengths; changing them during execution does not rebuild the route.

- Units are meters; each row is a segment displacement in the fixed route frame (captured right-wall heading for scene 2).
- Positive x is route forward; positive y is route left. Enter `0.0` for an unused axis.
- The four segments go from start to corner, corner to goal, back to corner, and back to start.
- Defaults use the distances selected during the user tests. L and W remain adjustable; all four targets use the A captured after initial alignment and laser centering; segment arrival residuals do not redefine it.
- Scene 2 first aligns parallel to the right wall, then adjusts side centering and rear distance using laser scans while maintaining that heading. Rear distance targets 0.28 m from base_link; route obstacle avoidance is not implemented. The initial right wall must be straight and parallel to the intended route; its direction is captured in odom for this run.

Once configuration and hardware operating conditions have been verified, the entry is:

```bash
ros2 run distance_controller distance_controller 2
```

Nonfinite displacements reject startup before motion interfaces are created. Scene 2 defaults to system-backed node time, a 0.10 m/s speed limit, and a 0.20 m/s² acceleration limit. These defaults have not been validated on the automatic hardware route.

## Parameters and topics

| Parameter | Scene 1 default | Scene 2 default | Meaning |
| --- | --- | --- | --- |
| `use_sim_time` | `true` | `false` | Use simulation time; explicit overrides take precedence |
| `kp` / `ki` / `kd` | `1.5 / 0.0 / 0.0` | Same | Gains shared by both planar axes |
| `max_speed` | `0.40` | `0.10` | Planar command speed limit, m/s |
| `max_acceleration` | `0.60` | `0.20` | Normal command-vector change rate, m/s² |
| `dwell_duration` | `1.0` | `1.0` | Extra dwell after standstill verification, s |
| `odom_topic` | `/odometry/filtered` | Same | `nav_msgs/msg/Odometry` input |
| `cmd_vel_topic` | `/cmd_vel` | Same | `geometry_msgs/msg/Twist` output |

```bash
ros2 run distance_controller distance_controller 1 --ros-args -p max_speed:=0.2
```

ROS parameter overrides and topic remapping are supported. Pose is assumed to use odom coordinates and feedback twist to use body coordinates. Frame names are assumed. Scene 2 checks consumed numeric values and quaternion norm before accepting feedback.

## Control and completion

1. `on_odom()` stores feedback and its steady-clock receipt time.
2. A 50 ms wall timer calls `on_timer()` to check faults, feedback, and node time.
3. Targets use the fixed route origin, measured initial heading in scene 1 or captured right-wall heading in scene 2, and cumulative nominal displacements; actual stopping errors are not accumulated into later targets.
4. PID velocities in odom are speed-limited, acceleration-limited, rotated to the current body frame, and published.
5. Inside 0.01 m position tolerance, publish zero. Require feedback planar speed below 0.01 m/s and absolute yaw rate below 0.02 rad/s continuously for at least 0.5 node-clock seconds.
6. Dwell for `dwell_duration`, then advance. After the final dwell, keep the zero command and shut down the ROS context.

Feedback timeout uses local steady-clock receipt time with a 0.5 s threshold. Before the first message, keep waiting at zero velocity. PID, settling, and dwell use node time. Scene 1 feedback timeout and backwards node time latch a fault; scene 2 has bounded recovery as described below; an observation interval above 0.2 s resets timing-related state and stops the current tick.

Stop commands bypass acceleration limiting and clear ramp history. There is no obstacle avoidance; scene 2 holds the captured right-wall heading. Publishing zero and verifying standstill are separate steps.

## Documentation and initialization conventions

- Project documentation, configuration descriptions, and source comments use English. External Obsidian and course learning records remain Chinese.
- Source comments use Doxygen forms: `/** ... */`, `///`, and `///<`.
- Function interface documentation is in the header; `main()` documentation stays in `main.cpp`.
- Each parameter states `[in]`, `[out]`, or `[in,out]`, the actual calling function and source field, read purpose, writeback destination, and validity conditions. Member-state changes are documented as side effects.
- Member defaults live in the header and use `{}` initialization. The constructor initializer list initializes the base class.
- Scene/ROS parameter reads and runtime state updates remain assignments. Zero initialization does not replace validity flags.

## Documentation and checks

From the package root:

```bash
cd ~/ros2_ws/src/distance_controller
git diff --check
doxygen docs/Doxyfile.check
doxygen docs/Doxyfile
```

The check includes private members. HTML starts at `docs/generated/html/index.html`. Install/check tools in the actual target environment; configuration alone is not verification.

- [Control flow](docs/flow.md)
- [Tooling](docs/tooling.md)
- [Comment checks](docs/comment_checks.md)
- [Style profile](docs/style_profile.md)
- [Test planning](test/README.md)
- Style check script `tools/check_style.py`: accepts `--compile-db ~/ros2_ws/build/distance_controller`; missing dependencies report `NOT_READY`. It does not test controller behavior.

## Verification scope

2026-10-09: the documented refactor and initialization changes build locally. Doxygen checks/HTML generation and whitespace checks passed. The historical function-size warning for `on_timer()` was reviewed as a coordination responsibility; its old line count included comments removed later.

The earlier refactor completed all ten segments in the local WSL empty-world adapter: about 61.84 s, completion errors 7.652–8.114 mm, peak commanded speed 0.40 m/s, no controller warnings, and normal exit code 0 after stopping. Cloud and hardware were not revalidated in that run. `test/` contains route-model GoogleTests and isolated ROS fixtures; these are not a complete behavior suite.

Official Task1 acceptance uses tag `task1`. Preserve working changes before switching versions, rebuild, and source the overlay: checking out source alone does not replace the installed executable.

## Scene 2 heading behavior

Only scene 2 enables heading control. Initial alignment publishes zero linear velocity and a
bounded yaw rate toward the measured right-wall direction. The robot must then remain within heading tolerance,
with measured planar speed below 0.01 m/s and yaw rate below 0.02 rad/s, for the configured
settling duration. Initial side/rear positioning follows. After center offset, rear distance, heading and standstill acceptance for alignment_settle_duration, record current odom position as A and begin the four segments. Route axes use the captured aligned odom yaw, so heading hold and all four position targets share the same reference.

| Startup parameter | Default | Meaning |
| --- | --- | --- |
| `heading_gain` | 1.0 | Proportional yaw gain, 1/s |
| `max_yaw_rate` | 0.25 | Symmetric angular speed cap, rad/s |
| `heading_tolerance` | 0.01 | Absolute heading acceptance and command deadband, rad |
| `translation_pause_angle` | 0.15 | Pause translation above this yaw error, rad |
| `alignment_settle_duration` | 0.5 | Initial continuous standstill, node-clock seconds |

All values must be finite and positive; tolerance must be below the pause angle, which must
be below pi. During travel, small heading errors are corrected while translating. Larger
errors suspend translation and clear planar PID/ramp history. At a position target, heading
is corrected before settling. If position or heading leaves acceptance during dwell, the
same target is reacquired and its settling/dwell starts again. Odom timeout, backwards time,
or invalid scene-2 feedback latch a stopped fault; restart is required. A forward time jump
resets settling, and frozen node time stops scene-2 commands. Laser sets the initial wall reference and position. Route laser feedback is opt-in per segment; see manual_steps.md. No general obstacle avoidance or physical corridor-alignment guarantee is included.

Example after validating the hardware operating conditions:

```bash
ros2 run distance_controller distance_controller 2 --ros-args \
  -p forward_distance:=0.90 -p lateral_distance:=0.516780 \
  -p heading_gain:=1.0 -p max_yaw_rate:=0.25
```

For a scene-2 local simulation, add `-p use_sim_time:=true`. Scene 1 and the existing `task1`
tag retain their original acceptance boundary. Local tests do not certify the real course route.

### Historical heading-version verification (2026-10-09, before fixed A)

Scene 2 completed 4/4 segments in the local Gazebo empty world using default distances and speed: 53.14 s, completion errors 7.708–8.778 mm, peak commanded speed 0.10 m/s, final yaw 0.001212 rad, final zero command and normal exit. The isolated odometry fixture also verified initial rotation without translation, injected heading recovery, interrupted dwell recovery, feedback timeout and invalid-data latching. Parameter and clock probes checked six invalid heading configurations, frozen time and backwards time. Scene 1 was rerun to verify the existing ten-segment route. These are local observations, not hardware or official acceptance. The 82-line on_timer coordination warning is retained after responsibility review; helper functions contain the new control policies.

### Historical fixed-start local verification (2026-10-09)

The historical fixed-start version completed initial A approach and 4/4 route segments in the local empty-world adapter: 50.29 s, segment acceptance errors 7.047–7.514 mm, final odom distance to configured A 7.521 mm, final yaw 0.001383 rad, maximum commanded planar speed 0.10 m/s, final zero command, and exit code 0. Log assertions checked all four absolute targets and that A acceptance preceded segment 1. Isolated fixtures verified simultaneous x/y approach, initial rotation gating, heading/dwell recovery and feedback faults; 16 parameter cases passed. This does not validate the real corridor or a reset odom reference.
## Initial laser positioning (current scene 2)

Manually place near the intended start, within the preparation travel bound. Startup now checks fresh side scans, rotates parallel to a stable right-wall fit, then adjusts both planar axes while maintaining that heading. Initial rotation has zero translation; the positioning phase may move forward or backward. Center error is `(left_wall - right_wall) / 2`; positive means move left. Rear error is `rear_target_distance - rear_wall`: positive commands forward, negative backward. Once both distance errors and yaw are accepted and measured velocities remain low for `alignment_settle_duration`, the current odom pose becomes A. The route remains L=0.90 m forward, W=0.516780 m right, W left, L back, along the captured wall-aligned route axes. The former `start_x`/`start_y` overrides reject startup to prevent accidental reuse of obsolete absolute coordinates.

Filtered scans use SensorDataQoS. A fixed TF from the scan header frame to `base_frame` transforms directions and points, including the observed 180-degree laser mounting. All three wall windows must contain at least six valid rays and at least 50% valid coverage. Median body-y distance estimates reject excessive median absolute deviation. This assumes nearby walls in both windows; no wall identity or parallelism is inferred. A separate wider right-side window estimates initial heading; distance windows alone do not establish wall direction.

| Startup parameter | Default | Meaning |
| --- | --- | --- |
| `scan_topic` | `/scan_filtered` | Filtered `sensor_msgs/msg/LaserScan` input |
| `base_frame` | `base_link` | Body frame for left/right windows |
| `centering_gain` | 0.5 | Lateral offset proportional gain, 1/s |
| `centering_max_speed` | 0.03 | Initial combined planar speed cap, m/s; also capped by max_speed |
| `centering_tolerance` | 0.01 | Accepted center offset and rear-distance error, m (side difference <=0.02 m) |
| `side_window_half_angle` | 0.174533 | Half-width around body +/-90 degrees, rad |
| `side_min_distance` / `side_max_distance` | 0.18 / 1.5 | Accepted wall distance from body origin, m |
| `side_max_mad` | 0.03 | Maximum median absolute deviation in one window, m |
| `scan_timeout` | 0.5 | Scan stamp and steady receipt age limit, s |
| `preparation_timeout` | 60.0 | Initial-adjustment duration limit, steady seconds |
| `preparation_max_travel` | 0.20 | Maximum odom displacement from initial placement, m |

Scan timestamps must advance and use the node's time basis. Before the first accepted scan/TF, remain stopped; after accepted data becomes invalid or stale, latch a stopped fault during preparation. Time/travel limit violations also latch stop. Restart is required. The initial planar command is proportional and vector-speed capped; the route's acceleration ramp is not applied during centering. Stop commands are immediate.

After A is captured, scans do not gate ordinary displacement steps. Explicit side-centering or front-wall goals require fresh valid windows and fault on loss. Right/left movement cannot enable side centering. Rear positioning is initialization only; no general route obstacle avoidance is provided.

The fixed-A version's local empty-world success did not establish a repeatable real-world odom reference. The user subsequently reported reverse motion into a wall when starting at odom (0,0). This version removes that absolute-A approach; it still requires target-environment scan/TF verification and supervised testing before hardware acceptance.

### Historical side-only centering verification (2026-10-09)

A synthetic LaserScan/odom closed-loop fixture verified both lateral signs, the 180-degree mounting TF, zero body-x preparation, centered A capture and four route segments after scans stop. Eight fault cases verified scan loss, invalid ranges, old timestamps, missing frames/TF, absent scans and preparation travel/time bounds. Twenty-four invalid parameter/obsolete-coordinate cases rejected startup. Scene 1 completed 10/10 in local Gazebo (62.40 s, exit 0). Synthetic parallel-wall tests do not establish real scan-window reliability; scene-2 corridor/hardware validation remains pending.

### Rear-distance initialization

The rear +/-5-degree window uses the median positive rearward body-x distance after TF transformation, including mounting translation. `rear_target_distance=0.28` m refers to the `base_frame` origin, not bumper clearance. `rear_window_half_angle=0.0872664626` rad and `rear_min_distance=0.22` m are startup parameters. Rear samples use the same minimum six points, 50% coverage, maximum distance and MAD checks as side windows. The target must be above the minimum plus position tolerance and below the maximum minus tolerance. A closer-than-minimum return prevents preparation motion; it is not an automatic escape maneuver.

First rotate parallel to the right wall and verify standstill; then adjust center offset and rear error together, keeping yaw control active. The combined planar speed remains <=0.03 m/s (also capped by max_speed), with the existing preparation time/travel bounds. Both distance errors must be <=0.01 m, yaw accepted, and measured speed low continuously before A is captured. Route distances remain 0.90/0.516780 m. Loss of rear measurements blocks or faults preparation just like loss of side measurements. These checks do not guarantee obstacle-free movement or identify which surface generated the return.

Local synthetic tests passed both forward/backward correction combined with opposite lateral offsets, followed by full four-segment routes. Rear-data loss and below-minimum distance both latched stop; the existing eight preparation failure cases also passed. Hardware wall identity and repeatability remain unverified.
## A-to-B wall observations

Scene 2 continues processing scans after initialization solely for observation. During every selected A-to-B segment, `A->B observation` logs approximately once per second, including rotation/settling/dwell ticks, and once at B standstill acceptance. Fields are odom x/y, `dy_from_A=y-route_y_`, actual yaw in radians/degrees, left/right body-origin distances in meters, and scan receipt age. Each side is checked independently; a missing rear return does not hide valid side observations. Invalid or stale sides print `unavailable`, never a cached numeric distance presented as current. Scan and odom are latest callback values, not a synchronized sensor pair.

These observations do not change route targets, steering, completion or stop conditions; no ongoing centering or laser obstacle stopping is introduced. Side-window identity can change near openings and corners, so distance trends need physical context. Compare route-frame `cross_track` with left/right trends (`dy_from_A` remains an odom-axis diagnostic) to distinguish odom tracking from corridor alignment.
## Right-wall initial heading (current scene 2)

A nearby straight right wall defines forward heading. The fit uses transformed
body-frame endpoints within +/-30 degrees of the right direction; this is separate
from the narrower distance windows. At least 12 points, 50% coverage, 0.18 m tangent
span and perpendicular RMS <=0.012 m are required. A fitted direction outside +/-30
degrees of body forward is rejected. Place the robot roughly facing along the wall.
The estimated odom-frame wall direction must remain within 0.02 rad of an anchor
for 0.5 steady seconds before rotation is allowed. An inconsistent fit restarts
that interval. Poor geometry keeps the robot stopped until the preparation timeout;
there is no fallback to odom yaw zero. A straight unrelated surface can still pass
these checks: real wall identity and measurement repeatability require cloud testing.

The initial command uses the right-wall angle, then requires heading tolerance and
measured standstill. Capture the actual aligned odom yaw as `heading_reference_`.
Side/rear positioning holds this reference; capture A only after that preparation.
All four relative targets are rotated by this same reference. During the route,
heading uses odometry against the frozen reference; changing right-wall observations
near B do not change it. This cannot correct later odometry drift relative to walls.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `wall_heading_half_angle` | 0.523599 rad | Fitting window half-width; positive and less than pi/4 |
| `wall_heading_min_span` | 0.18 m | Minimum observed wall length |
| `wall_heading_max_rms` | 0.012 m | Maximum perpendicular fit residual |

`Right-wall alignment` prints validity, stability, relative angle, RMS and span.
`Centered A` includes actual yaw and the captured reference. `A->B observation`
adds `cross_track` in the rotated route frame and wrapped `heading_error`.
A nonzero odom yaw is expected when the wall is not parallel to odom x.

For the user's recent trial distances and heading tolerance (explicit overrides):

```bash
ros2 run distance_controller distance_controller 2 --ros-args \
  -p forward_distance:=0.90 -p lateral_distance:=0.516780 \
  -p heading_tolerance:=0.01
```

Package distance defaults remain 0.90 / 0.516780 m. This change does not add route
collision avoidance. Synthetic fixtures are support evidence, not hardware acceptance.

Local verification (2026-10-09): all 17 synthetic right-wall fixture cases and 40
invalid-parameter cases passed. Scene 1 completed 10/10 in local Gazebo in 62.20 s,
with final zero command and exit 0. Build, Doxygen generation/check, Ruff and Git
whitespace checks passed. Clang-tidy retains reviewed size warnings for the timer
coordinator, scan decoder and initial positioning state machine (92/79/61 lines);
no claim of warning-free analysis is made. Cloud right-wall measurements remain pending.


## Composable planar routes

Scene 2 accepts the startup string-array parameter `route`, default `[AB, BC, CB, BA]`.
Use connected segments starting at initialized A. Unknown names, an empty route, or a
disconnected sequence reject startup before motion interfaces exist. Every route uses
initial right-wall alignment and side/rear positioning.

```bash
# A -> B, then stop and exit after dwell; recent user trial overrides.
ros2 run distance_controller distance_controller 2 --ros-args \
  -p forward_distance:=0.90 -p lateral_distance:=0.516780 \
  -p heading_tolerance:=0.01 -p 'route:=[AB]'
# Substitute the route argument to choose another composition:
# -p 'route:=[AB, BA]'
# -p 'route:=[AB, BC]'
# -p 'route:=[AB, BC, CB, BA]'
```

Package defaults remain L=0.90 m, W=0.516780 m and heading_tolerance=0.01 rad.
The example deliberately uses the latest user trial overrides. Shorter routes finish
at their last named endpoint; they do not automatically return to A.

`PlanarMotion::move_forward(distance, speed, dwell)`, `move_backward`, `move_left`,
`move_right`, and `move_relative(dx, dy, speed, dwell)` construct validated motion
**descriptions** in `route.hpp` / `route.cpp`; they do not start motion or block.
Forward/backward use fixed route x; left/right use fixed route y. Units are meters,
meters per second and node-clock seconds. The controller also caps segment speed by
`max_speed`. Named routes default to startup speed/dwell; `segments.ID` overrides each segment independently.

`configure_route_steps()` assembles independently configured edges and checks continuity;
`compose_route()` remains the pure standard A/B/C composition helper.
`execute_current_segment()` in `route_execution.cpp` advances one tick of tracking,
standstill or dwell through shared PID/heading helpers. It returns `Running`,
`Completed`, or `Failed`. Only Completed lets `on_timer()` call `advance_route()`.
Faults stop without selecting another segment. Targets are frozen once per segment
from initialized A plus cumulative planned displacement rotated by the captured heading.
Ordinary named steps use planned endpoints without accumulating stopping residuals.
Sensor-defined steps deliberately use the measured final anchor; ad-hoc relative steps
anchor to the accepted current pose. Scene 1 retains ten displacements.
Turning is not included in this change.

## Initialization timeout diagnosis

| Startup parameter | Default (steady seconds) | Budget |
| --- | --- | --- |
| `wall_measurement_timeout` | 20 | Initial wait for accepted stable wall measurements |
| `alignment_timeout` | 30 | Heading rotation and standstill after wall acquisition |
| `positioning_timeout` | 30 | Side/rear positioning and standstill |
| `preparation_timeout` | 60 | Total preparation, unchanged |

The total timer starts on the first preparation tick after odometry is available.
Logs progress through `wall_measurement`, `heading_alignment`, and `positioning`.
Each stage timer starts once. Fit loss during alignment stops movement without
restarting its deadline; repeated instability cannot extend the total budget.
Node-time settling remains separate from these steady-clock deadlines.

| Fault reason | Meaning / first check |
| --- | --- |
| `WALL_MEASUREMENT_TIMEOUT` | No stable fit; check scan/TF, valid points, RMS, span and wall identity |
| `HEADING_ALIGNMENT_TIMEOUT` | Heading/standstill did not finish; inspect angle, fit stability and yaw response |
| `POSITIONING_TIMEOUT` | Side/rear positioning did not finish; inspect distance errors and measured motion |
| `SCAN_INVALID` / `SCAN_STALE` | Accepted scan became invalid or stopped arriving; inspect scan_reason, stamps, TF and receipt age |
| `PREPARATION_TOTAL_TIMEOUT` | Total preparation budget exhausted |
| `PREPARATION_TRAVEL_LIMIT` | Displacement from initial placement exceeded the bound |

Errors include stage/total elapsed time, scan rejection detail and wall-fit metrics.
Sensor/travel failures take precedence over deadlines. All faults latch zero velocity
and reset PID; restart after correcting the cause. Increasing a deadline does not
repair bad measurements or failed movement.


### Local verification (2026-10-09)

The history update passed 10 GoogleTests, 65 startup rejection cases and 21
manual/automatic continuation scenarios, plus CLI/configuration conflict probes.
The earlier initialization refactor passed 25 regression scenarios. Task1 completed 10/10 in local Gazebo in
61.88 s, errors 6.954-8.007 mm in odom, peak command 0.40 m/s, final zero and exit 0.
An independent build/install also verified Python service typesupport and the CLI.
Doxygen checks/generation, Ruff, XML and whitespace checks passed. The reviewed 63-line on_step_request() keeps one request transaction together.
The existing 68-line measure_wall_windows() warning is retained: it collects independent sectors
from one scan and publishes their validity. Generated ROSIDL serialization functions
also exceed the size threshold; generated files are not edited to silence it.
These tests do not certify cloud/hardware clearance or independent course acceptance.

## Manual continuation and per-segment feedback

See [Planar steps and manual exploration](docs/manual_steps.md) for ready-to-use
commands, custom BD configuration, frame/heading contracts, front-body clearance,
manual pause/resume and fault behavior. Scene 2 can now keep the node alive after AB
with manual_mode:=true, accept one next planar step, and record the new endpoint.
All planar directions hold the persistent heading; turn remains outside this version.
The latest interface is described there. Task3 explicitly requires turn_controller.cpp and
TurnController; Task4 reuses that program with real waypoints. Neither is implemented
by this planar refactor.

## Waypoint logs

After Scene 2 initializes A, `Waypoint` lines list the fixed route goals in the
odometry message frame (meters and radians), including repeated B/A visits. Yaw
is the held heading captured during alignment, not necessarily odom zero. Each
segment logs its target x/y/yaw; `Pose` is live feedback, while `Reached` reports
actual pose only after stopping and completing dwell. Feedback-dependent goals
(front-wall arrival, side centering, or runtime-relative steps) stop the startup
preview; their execution targets and actual endpoints are reported at runtime.
The preview is informational and does not change route execution.

Initialization also logs stage changes and throttled `Initialization progress`:
current odom pose, left/right/rear distances, rear target, centering/rear/heading
errors and tolerances, readiness flags and settling hold requirement. The heading
basis changes from `right_wall` to `held_odom_heading` after alignment. Settling
flags reflect the preceding tick. Invalid/missing scans and stage/total timeouts
retain their explicit wait/fault messages; `Centered A recorded` marks completion.

## Scene 2 odometry recovery

A gap above 0.5 steady seconds in accepted odometry enters `ODOM_RECOVERING`.
The controller continuously publishes zero, resets PID/ramp history and restarts
settling/dwell. The current target, route index and completed history stay intact.
A gap observed by the odom callback itself also enters recovery even if the timer
has not yet run. Scene 1 retains its original immediate timeout fault.

Recovery has a fixed 2 s budget measured from the detected stop, never extended
by intermittent packets. Accepted scene-2 stamps must increase and be no older
than 0.5 node-clock seconds (at most 0.1 s in the future). During recovery,
the source-age limit tightens to 0.15 s. Recovery additionally
requires 0.3 steady seconds of continuous fresh feedback, accepted intervals no
larger than 0.15 s, planar speed below 0.01 m/s and yaw rate below 0.02 rad/s.
Stale, duplicate or reordered samples do not refresh accepted feedback time.
Compared with the last good pre-gap pose, a position change over 0.25 m, wrapped
yaw change over 0.35 rad, or a changed parent/child frame terminates recovery.
These are conservative continuity guards, not proof that localization is correct.

`ODOM_RECOVERED` permits the next tick to re-evaluate the original target with
new PID timing. It does not skip a waypoint or reuse old dwell time. Preparation
and segment time budgets keep running. Manual motion requests are rejected while
recovering; status/history remain readable and cancel remains available.

`ODOM_RECOVERY_TIMEOUT` or `ODOM_RECOVERY_POSE_OR_FRAME_JUMP` ends automatic
recovery. By default the node publishes a final zero command, shuts down, and
returns process status 2. The plain course command requires no additional flag.
`base_command_watchdog_verified` now defaults to true for this Task2 deployment;
set it explicitly to false to retain latched zero commands instead of exiting.
The user read `cmd_vel_timeout=0.5` from the cloud base controller and requested
this default. It is a deployment assumption, not automatic watchdog detection.
The supplied full-route log confirms normal completion and user-observed stopping;
it does not exercise command-loss braking or recovery timeout. Those cloud tests
remain pending. A final ROS message alone does not guarantee delivery.

The right-wall fit RMS default is now 0.012 m, supported by the user's successful
four-segment run with that override. This measurement tolerance is distinct from
the 0.01 rad heading tolerance. The new recovery behavior still needs cloud tests.


## Continue from an intermediate stopped pose

After the previous velocity publisher has exited, start a new manual session:

```bash
ros2 run distance_controller distance_controller 2 --ros-args \
  -p manual_mode:=true -p adopt_current_pose:=true -p max_speed:=0.03
```

This requires fresh stopped odometry continuously for alignment_settle_duration.
No wall preparation or automatic AB motion occurs. Current yaw becomes the held
heading and the route axes; pending configured segments are discarded. WAITING
accepts manual steps. Scene 1 and non-manual adoption are rejected.

```bash
ros2 run distance_controller step status
ros2 run distance_controller step forward 0.30 --speed 0.03 --frame heading --label P04_trial
```

The new history calls its origin A, but physically this can be P03. It does not
restore the original A/B history or reset odom. `return_to A` now means this new
session origin. Fixed-distance motion does not use laser stopping; inspect clearance
and remeasure after completion. No physical robot was driven by the local tests.

Verification: isolated no-scan fixture rejects adoption while measured linear speed
is nonzero, remains stopped, captures a rotated pose, moves forward along its heading,
then stops/exits. Existing manual sequence regression also passed. Build and GTest passed.
The general colcon suite is not fully green: flake8/uncrustify style failures and
xmllint timeout remain; the repository uses Ruff/clang-format conventions.

## Supervised reverse survey

See [Supervised reverse survey](docs/survey_return.md) for the provisional Task6 return route. Preview with `ros2 run distance_controller survey_return`; execution requires one explicit step and physical verification. No automatic full-route replay or old-odom restoration is provided.

Read and save four-direction laser ranges without moving: `ros2 run distance_controller survey_return --measure`. Executed return actions save before/after observations in their JSON audits; unavailable scans are explicit. See the supervised reverse survey guide for range definitions.


### Task6 side-wall tracking update

Wall-guided translations now use a held-heading side-distance estimate over +/-30 degrees.
Brief fit loss stops motion and allows bounded stationary recovery; persistent loss leaves
an incomplete action. Heading preparation keeps its existing estimator; front-wall arrival uses an independent +/-12 degree TLS window.
See [wall session details](docs/wall_session.md#side-distance-tracking-and-bounded-recovery).
Manual BODY_CLEARANCE output still reports the original independent line fits; motion logs
identify the constrained side estimator separately. Cloud step-by-step validation is pending.


Task6 preparation now defaults to the right wall for heading. Use `--alignment-wall right`
or `--alignment-wall left` with `--wall-guided --start prepare`. Centering/rear positioning
remain unchanged; the initial left-side reference is measured with the constrained
side-distance estimator after alignment. This does not change per-action following walls.

Stopped action sessions can now restart with `action_session --resume FILE` without P01 preparation. See [checkpoint recovery](docs/wall_session.md#resume-at-a-stopped-waypoint-after-updating-code) for odom continuity checks, partial-action resume, and legacy audit migration.


Task6 P03 uses a provisional **9.5 cm body-clearance target / 1 cm tolerance**.
P02-P03 follows 2 cm closer to the left reference wall, with bounded side alignment before forward travel.
Front-wall arrival pauses for three distinct stopped scans before final side correction.
Existing partial checkpoints require an explicit policy edit; see
[P03 calibration and continuation](docs/wall_session.md#p03-stopped-front-endpoint-calibration).


Task6 blocked-turn adjustment: `adjust_turn 0.02` performs a guarded, checkpointed
2 cm right correction without starting the turn. See [wall-session limits and retries](docs/wall_session.md#explicit-clearance-adjustment-before-a-blocked-turn).
