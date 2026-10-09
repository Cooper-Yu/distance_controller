# Task2 acceptance record

## Accepted invocation

On 2026-10-09, the learner ran the course command in the CyberWorld robot
workspace and supplied the complete four-segment log. The learner explicitly
confirmed no wall contact throughout the run and a final stationary robot.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run distance_controller distance_controller 2
```

Task2 functional execution is complete with Coach implementation support.
Official grading and independent learner reconstruction are separate, pending
checks. Manual history/return extensions have local tests; their cloud validation
is not claimed by this four-segment acceptance.

## Current tag configuration (updated after the accepted run)

| Setting | Accepted default |
| --- | --- |
| Route | A -> B -> C -> B -> A |
| Forward / backward distance | 0.90 m |
| Right / left distance | 0.516780 m |
| Speed cap | 0.10 m/s |
| Dwell per segment | 1.0 s |
| Heading tolerance | 0.01 rad |
| Rear distance from base_link origin | 0.28 m |
| Manual mode / simulation time | false / false |

Initialization aligns to the right wall, holds the captured odom heading while
centering and adjusting rear distance, then records A. Route goals are relative
to that initialized origin and heading, not hard-coded absolute odom locations.
The route uses position feedback and held heading; it is not general obstacle avoidance.

`config/segments.yaml` now uses the same 0.90 m forward distance and 0.01 rad
heading tolerance. It still explicitly enables manual mode, unlike the course
command. The AB/BD example also uses 0.90 m for AB.

The user requested replacing the published task2 tag after this unification.
The prior snapshot remains commit `735d329`. Its successful default run used
0.90 m / 0.02 rad; the earlier YAML run used 0.93 m / 0.01 rad. The exact new
0.90 m / 0.01 rad combination has not yet been revalidated on the real robot.
The measurements below are historical evidence, not results of the new defaults.

## Observed default-run results

The supplied log spans ROS timestamps 1791543835.147256132 to
1791543879.348440317 (about 44.20 s). It contains initialization completion,
all five planned waypoint entries, four `Reached` messages, and `Route completed.`.
No WARN or ERROR appears in the supplied log.

| Segment | Odom position error | Heading error relative to held reference |
| --- | ---: | ---: |
| A -> B | 5.235 mm | +0.700 deg |
| B -> C | 6.303 mm | +0.834 deg |
| C -> B | 4.574 mm | +0.166 deg |
| B -> A | 7.591 mm | -0.648 deg |

Errors are calculated from the rounded target/actual poses in `Reached` lines.
They are odometry-frame errors, not independent external measurements of physical
position. The held yaw was -0.011218 rad. Initial A was (0.018118, -0.010108) m;
final actual A was (0.025656, -0.009210) m. User observation confirms no wall
contact and final stopping; a shell exit code and independent final cmd_vel
capture were not supplied.

Evidence references in the local training record:
- Default-run attachment: f3edfa45-8fda-4c2e-aeb3-77eef6aa593f.
- Earlier explicit-YAML run: 2d014619-2485-4ffc-9dca-626453eb7b18.
- Detailed analysis and confirmations: Checkpoint18 code_lab/verification.md.

## Evaluation snapshot

The annotated `task2` tag freezes this acceptance documentation and the existing
controller implementation. This revision changes the heading default and YAML
distances only; no control algorithm is rewritten. The historical `task1` tag is unchanged.

Use a clean checkout/worktree of `task2` for evaluation, rebuild the package,
and source that workspace before running the accepted command. Do not assume
switching Git revisions changes an already built executable.

```bash
git fetch origin --tags
git checkout task2
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select distance_controller
source install/setup.bash
ros2 run distance_controller distance_controller 2
```
