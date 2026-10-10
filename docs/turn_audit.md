# Local-only turn clearance audit

This diagnostic compares laser clearance warnings with independent Gazebo wall
contacts. It does not alter `WallRunner`, install an executable, add a hardware
bypass option, or change Task6 acceptance. Do not use it to certify the current
real P03 pose.

## Reproduce in the verified environment

WSL Ubuntu-22.04, ROS2 Humble, existing ROSbot XL Gazebo model and overlays:
`~/checkpoint16_sim_ws`, `~/checkpoint18_assets/local_support`, `~/ros2_ws`.
The fixture reuses `local_support/route_test/test_world.sdf` for physics plugins,
removes the maze, and adds one known flat wall. These local assets are prerequisites;
they are not bundled or installed by this package.

```bash
cd ~/ros2_ws/src/distance_controller
bash test/run_turn_audit.bash
```

The runner isolates ROS domain 184 and Gazebo partition `cp18_turn_audit`, uses
headless Gazebo, bounds each case, and cleans up only its own process group.
Do not run two copies concurrently. Output is under `/tmp/turn_physics_audit_*`;
`/tmp/turn_physics_latest.txt` identifies the latest directory.

The existing C++ turn controller performs a right 90-degree turn at up to
0.20 rad/s. Laser sweep checks are logged, not enforced, **only inside this test**.
Stale feedback aborts. A wall contact interrupts the controller and sends zero
velocity. This contact stop happens after simulated contact; it is not preventive
collision protection. Ground contacts provide a sensor heartbeat; wall/robot
collision names identify wall contact independently of the laser model. The narrow
case must produce wall contact, so silence alone cannot pass the detector check.

## Observed 2026-10-10 result

See `../test/turn_audit_results.json` for exact counts and the raw log path.
Wall distance is from initial base origin to the wall plane, not body clearance.

| Wall distance | Result | Wall contact messages | Final yaw |
|---|---|---:|---:|
| 0.300 m | Turn completed | 0 | -1.566596 rad |
| 0.230 m | Turn completed | 0 | -1.564709 rad |
| 0.185 m | Contact detected; test stopped | 10 | -0.192876 rad |

All three emitted raw-scan clearance warnings. In the spacious case a nearest
point was approximately base (-0.1634, -0.0576) m, inside the body envelope and
away from the known wall. Raw `/scan` includes returns requiring separate
investigation; it differs from hardware `/scan_filtered`. These tests do not
identify the real robot's rear returns as noise or justify filtering them out.

The experiment shows that completing a turn after ignoring warnings is not proof
of safety: the narrow case makes physical contact. The real P03 remaining turn,
its cables, real sensor filtering, and actual footprint were not reproduced.
Keep hardware protection; identify persistent near returns before changing policy.

## Verification scope

Three case-result/contact/heading assertions passed; wrong-domain entry was
rejected before ROS setup; Ruff format/check and Bash syntax checks passed.
Initial fixture preparation failed on missing contact heartbeat and then missing
TF readiness; both startup conditions now gate motion. No production code changed,
so no new colcon build or production regression claim is made. Fixture construction
and orchestration are explicit test setup; their size is reviewed as one bounded
experiment rather than split into artificial functions. GDB, GUI visualization,
rosbag and coverage remain deferred under the existing Task6 engineering plan.
