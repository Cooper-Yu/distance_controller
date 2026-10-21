# Precise start: rolling evidence and bounded median correction

This changes only optional `precise_start` preparation. It does not change route
control, waypoints, the 4 cm turn drift limit, or enable laser obstacle avoidance.
The original final distance tolerance (at most 5 mm), wall-angle tolerance (at most
0.5 degrees), 12/15 consensus, 1 cm distance spread and 2 degree angle spread remain.

The motivating stopped recording had complete batches with approximately 9 mm rear
offset. Disjoint batches rejected their spread, while overlapping windows using the
same criteria could be stable. A stable median outside the target previously returned
to single-frame positioning, which could immediately reenter verification on a noisy
in-tolerance frame without performing the required adjustment.

## State boundaries

1. **Stopped verification:** keep the existing speed/anchor conditions and 0.5 s
   stopped hold. Accept only new post-hold scan stamps. Retain the latest 15 frames
   and evaluate on every new frame once full. An unstable window remains stopped;
   it neither clears the window nor restarts the existing 5 s verification budget.
   Unstable summary logs are limited to one per 0.5 s; diagnostic last-summary state
   updates on every evaluation. `batches` now counts evaluated overlapping windows.
2. **Stable acceptance:** the unchanged median distance and wall-angle gates alone
   allow recording P01. Stable does not itself mean on target.
3. **Heading correction:** a stable angle outside tolerance follows the existing
   bounded alignment path, then requires a new full stopped verification. It consumes
   the existing refinement counter and does not reset stage/total deadlines.
4. **Small translation:** with accepted median heading, each median distance error
   must be at most 2 cm. A larger error faults `START_CORRECTION_RANGE`; it is not
   silently converted into a larger move. An axis already within its original
   distance tolerance stays at zero. Otherwise target minus median defines body
   displacement (rear error is x; right error is y), with combined Euclidean norm
   capped at 1 cm. Rotate once using the stopped odom yaw and freeze the odom target.
   Consume one of the existing maximum three refinements. Later individual wall
   measurements cannot rewrite or prematurely complete this target.
5. **Feedback tracking:** P gain 2/s on current odom target error; body velocity norm
   is capped by the minimum of 0.008 m/s, configured centering speed and max speed.
   Angular command is zero. Fault on more than 4 s, more than 1.2 cm actual odom
   displacement, more than 3 mm cross-track displacement, more than 0.005 rad
   yaw change from the frozen anchor, or violation of the original heading tolerance.
   Nonfinite feedback, invalid plans or missing required wall-heading evidence fault.
   Odom frame/child-frame are frozen with the anchor; changed frames or invalid source
   timestamps during a step fault. A small target overshoot may briefly reverse the
   P command toward the same frozen target, within all the same speed/travel/time
   bounds; this implementation does not promise monotonic motion without overshoot.
   These are additional bounds; the outer positioning/total timers and travel guard
   still execute first and retain their original origins and deadlines.
6. **Stop and reverify:** odom residual at most 0.5 mm commands zero and discards the
   motion target. This is local relative odom tracking, not absolute P01 accuracy.
   Start a new unchanged 5 s verification cycle, including physical stopped feedback,
   the full 0.5 s hold and 15 new samples. No old samples or single-frame positioning
   branch can grant acceptance.
7. **Sensor recovery/fault:** preparation faults and scan/odom recovery stop and cancel
   the entire active target/anchor; recovery never resumes its remaining displacement. After
   the existing recovery/alignment path, require new stopped evidence before choosing
   another correction. Refinement usage is retained. Other existing global fault
   paths (for example invalid odom quaternion) may retain inert state, but latch zero
   commands and cannot restart any motion.

No extra budget is granted to an unstable sliding window. The 4 s motion limit does
not replace the existing 5 s per-verification, 30 s positioning or 60 s total limit.
The node's configurable stage/total settings remain authoritative; this change does
not raise them. New `START_CORRECTION`, `START_CORRECTION_STOP`, and
`START_CORRECTION_CANCELLED` events identify target ownership and evidence resets.

## Local verification

Target environment: WSL Ubuntu-22.04 / ROS 2 Humble. Pure tests extend
`test_start_verification`; `test/verify_precise_correction.py` launches the real
installed preparation node through `odom_trial`, using a synthetic raycast room,
ROS domain 194 and localhost only. It never connects to the robot. The fixture
includes a rear median offset of 8.92674018 mm, intermittent individually acceptable
measurements, and a stable window spanning a rejected disjoint-batch boundary.
These are controlled reproductions, not claims to replay callback scheduling from
the original bag.

```bash
source /opt/ros/humble/setup.bash
source /home/cooper/ros2_ws/install/setup.bash
colcon build --packages-select distance_controller
ctest --test-dir build/distance_controller --output-on-failure \
  -R 'test_start_verification|test_robust_wall'
python3 src/distance_controller/test/verify_precise_correction.py single_frame
```

Other fixture modes: `reverse`, `dual`, `post_stop`, `scan_gap`, `odom_gap`,
`freeze`, `heading`, `cross_track`, `travel`, `unstable`, `stop_velocity`, `oversized`,
`odom_frame`, `odom_stamp`, `attempt_limit`, `stage_deadline`, `total_deadline`.
The deadline cases use the real WallRunner preparation adapter with shorter test-only
outer deadlines, proving that these preempt an active correction without waiting 4 s.
The frame/stamp cases observe the real C++ node through the existing base Runner's
status service and cleanup. This avoids the higher Python adapter stopping first on
the same invalid input and hiding the C++ fault branch. The normal entry also rejects
a changed odom frame; the fixture does not disable or alter that production check.
The attempt-limit case keeps the measured offset after each successful relative move;
exactly three correction events occur, followed by `START_REFINEMENT_LIMIT` without
a fourth motion target.
Success and fault cases require last zero command and no remaining cmd_vel publisher.
The fixture tests local preparation only; real-robot calibration, repeatability and
final Task6 obstacle protection remain separate acceptance gates.

### 2026-10-22 local results

Humble build and two relevant CTest targets passed (11 start-verification tests and
5 robust-wall tests). All 18 fixture modes above passed with last-zero/pub0 checks.
The existing `HEADING_REENTRY=1 WALL_CASE=odom_trial_right_only PRECISE_START=1`
fixture also passed: `/tmp/wall_session_test/1792626603247381895`.
Representative runtime evidence is under `/tmp/precise_correction_ros/`:

- `1792626561111746675_single_frame`: rolling stable median commands -8.927 mm,
  stops, and passes a new complete verification cycle.
- `1792626572880177417_scan_gap` and `1792626588913614062_odom_gap`: interrupted
  target discarded; attempts 1 then 2 use a new median target after recovery.
- `1792626522708605920_attempt_limit`: three completed corrections, fourth rejected.
- `1792626541977951438_stage_deadline`: positioning timeout at 3.006 s;
  `1792626551521962054_total_deadline`: total timeout at 4.842 s. Both cancel active
  correction before its additional 4 s budget expires.
- `1792626504971686859_odom_frame` and `1792626513879829609_odom_stamp`:
  actual C++ frame/stamp fault branches, zero output and publisher release.

Clang-format, Ruff and `git diff --check` passed. The available system clang-tidy
is LLVM 14: its function-size checker counts ROS logging macro expansions despite
the project's `IgnoreMacros` option, so the new correction file's strict check
reports three macro-nesting findings. Reviewed authored functions are below 60
lines with at most two nested control levels; this is a tool-version limitation,
not a claimed clean clang-tidy run. Existing verification remains one stopped
evidence transition (about 65 lines); its responsibility was reviewed without
splitting its stop/hold/sample ordering merely to satisfy the size heuristic.
