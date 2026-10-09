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
0.90 m / 0.02 rad; the earlier YAML run used 0.93 m / 0.01 rad. At that revision, the exact
0.90 m / 0.01 rad combination still required real-robot revalidation.
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
controller implementation. The current revision also adds bounded odom recovery
and changes the right-wall RMS default to 0.012 m. Recovery requires new cloud
validation; prior route results do not certify fault-recovery behavior. The historical `task1` tag is unchanged.

Use a clean checkout/worktree of `task2` for evaluation, rebuild the package,
and source that workspace before running the accepted command. Do not assume
switching Git revisions changes an already built executable.

```bash
git fetch origin +refs/tags/task2:refs/tags/task2
git checkout task2
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select distance_controller
source install/setup.bash
ros2 run distance_controller distance_controller 2
```

## Subsequent diagnostic evidence and recovery revision

With 0.90 m travel and 0.01 rad heading tolerance, one run stopped on odom timeout;
a later initialization failed because RMS repeatedly crossed the former 0.010 m
limit. Attachment 3475f57e-4b69-4d45-971f-09f4eb722762 records successful full-route
execution using `wall_heading_max_rms:=0.012`: initialization 7.35 s, endpoint
position errors 2.854/7.650/6.783/5.438 mm, and `Route completed.`. This run supports
the new RMS default but does not prove network recovery or independent accuracy.

The user authorized moving task2 again after adding bounded feedback recovery.
See README's odometry-recovery section for qualification thresholds and the
process-exit policy. Following the user's readback of `cmd_vel_timeout=0.5` and
explicit request, the course command now exits with code 2 on terminal recovery
failure, after a final zero command. Explicit false preserves the old latch mode.
Attachment 69cef29d-d051-46eb-9ef3-8b7fe0221cea shows normal four-leg completion
and the user confirmed stopping; it does not show a recovery timeout. Cloud
fault-path verification remains pending.

## Local recovery verification (2026-10-09)

Ubuntu-22.04 / ROS 2 Humble verification passed:

- Ten fault-injection cases: short loss and stable recovery, terminal process exit
  with an asserted base watchdog, stale stamps, future stamps, pose jumps, frame
  changes, a single fresh sample, moving feedback, interrupted dwell, and retained
  segment timeout budgets.
- Four regressions: idle feedback loss with latched zero commands, waypoint logs,
  manual motion sequence, and chained history return.
- Ten GoogleTests and a ten-segment Task1 Gazebo run; the latter exited with code
  zero, ended with a zero command, and had odom endpoint errors of 7.096–8.390 mm.
- Build, Ruff, Doxygen, and whitespace checks passed. Clang-tidy reported existing
  generated-message/function-size findings; no new recovery-method finding.

These local tests exercise synthetic feedback and simulation. Cloud verification
of the new recovery behavior and the actual base command watchdog remains open.
The watchdog parameter is an operator assertion, not automatic discovery.
Default-exit follow-up: the no-watchdog-override timeout case, explicit-false latch
case, and short-outage recovery case passed locally after changing the default.
Build, Ruff, Doxygen and whitespace checks passed. Other prior regression results
above remain historical; the controller gains and trajectory were not changed.
