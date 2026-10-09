# distance_controller

A ROS2 Humble planar distance controller for ROSBot XL. It uses odometry feedback, planar PID, speed and acceleration limits, and odom-to-body velocity conversion. Each segment ends with verified standstill and an extra dwell; the node stops and exits after the final segment.

The Task1 acceptance snapshot is Git tag `task1` (`e1a26a6`). The current working tree also contains the Task2 scene entry and the documented header/source refactor. Scene 2 contains four nominal corrected displacements; automatic hardware execution has not been validated.

## Layout and reading order

```text
distance_controller/
├── README.md
├── CMakeLists.txt
├── package.xml
├── include/distance_controller/distance_controller.hpp
├── src/main.cpp
├── src/distance_controller.cpp
├── docs/
├── test/
└── tools/
```

| File | Responsibility |
| --- | --- |
| `main.cpp` | Initialize ROS, parse the optional scene, construct the node, and spin serially |
| `distance_controller.hpp` | Class declarations, interface documentation, state descriptions, and member defaults |
| `distance_controller.cpp` | Feedback, timing, targets, PID, limiting, stopping, and route transitions |
| <code>\ref build_cmake "CMakeLists.txt"</code> / <code>\ref build_package_xml "package.xml"</code> | Build targets and ROS dependencies |

Read `main()`, then the class interface and state, then follow `on_timer()` into its helpers. This README is the package entry; no duplicate README is needed under `src/`.

## Build and simulation

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

## Scene 2: nominal corrected displacements

`DistanceController::select_waypoints()` configures forward 1.360566 m, right 0.766780 m, left 0.766780 m, and backward 1.360566 m.

Scene 2 reads `forward_distance` (default 1.360566 m) and `lateral_distance` (default 0.766780 m) once at startup. Both must be finite and positive. These parameters define route lengths; changing them during execution does not rebuild the route.

- Units are meters; each row is a segment displacement in the fixed initial body frame.
- Positive x is initial forward; positive y is initial left. Enter `0.0` for an unused axis.
- The four segments go from start to corner, corner to goal, back to corner, and back to start.
- The estimate preserves measured A, assumes odom yaw zero, preserves the measured A-to-B chord length, and projects measured C onto the lateral line through corrected B. If alignment changes A or the heading, recompute the geometry.
- The controller currently captures the initial body heading; it does not perform automatic heading alignment, corridor centering, or laser clearance checks.

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

ROS parameter overrides and topic remapping are supported. Pose is assumed to use odom coordinates and feedback twist to use body coordinates. Frame names and quaternion validity are not checked.

## Control and completion

1. `on_odom()` stores feedback and its steady-clock receipt time.
2. A 50 ms wall timer calls `on_timer()` to check faults, feedback, and node time.
3. Targets use the fixed route origin, initial heading, and cumulative nominal displacements; actual stopping errors are not accumulated into later targets.
4. PID velocities in odom are speed-limited, acceleration-limited, rotated to the current body frame, and published.
5. Inside 0.01 m position tolerance, publish zero. Require feedback planar speed below 0.01 m/s and absolute yaw rate below 0.02 rad/s continuously for at least 0.5 node-clock seconds.
6. Dwell for `dwell_duration`, then advance. After the final dwell, keep the zero command and shut down the ROS context.

Feedback timeout uses local steady-clock receipt time with a 0.5 s threshold. Before the first message, keep waiting at zero velocity. PID, settling, and dwell use node time. Feedback timeout or backwards node time latches a fault; an observation interval above 0.2 s resets timing-related state and stops the current tick.

Stop commands bypass acceleration limiting and clear ramp history. There is no obstacle avoidance or target-heading controller. Publishing zero and verifying standstill are separate steps.

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

The current code completed all ten segments in the local WSL empty-world adapter: about 61.84 s, completion errors 7.652–8.114 mm, peak commanded speed 0.40 m/s, no controller warnings, and normal exit code 0 after stopping. Cloud and hardware were not revalidated in that run. `test/` still contains planning, not a complete automated behavior suite.

Official Task1 acceptance uses tag `task1`. Preserve working changes before switching versions, rebuild, and source the overlay: checking out source alone does not replace the installed executable.
