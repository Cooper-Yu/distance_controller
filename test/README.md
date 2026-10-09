# Test Planning and Entry Points

This directory holds the package's test plan. It does not yet contain a complete automated behavior suite, and no always-passing placeholder test is provided.

| Case field | Required content |
| --- | --- |
| Stable ID and contract | TODO |
| Inputs and initial state | TODO |
| Expected output, side effects, and tolerance | TODO |
| Normal/boundary/state-transition rationale | TODO |
| Target environment and command | TODO |

Use GoogleTest/ament_cmake_gtest for suitable C++ cases and pytest for Python. ROS integration tests may use launch_testing with readiness conditions, bounded waits, and cleanup. Start with a meaningful comparison for one implementation slice, then add related boundaries.

Preserve generated ament lint tests. Ruff and all ROS/ament checks are not identical; inspect conflicts without disabling unrelated checks globally. Test wiring must be verified, not inferred from template files.

Existing local simulation/fixture evidence remains in `/home/cooper/ros2_ws/training_notes/checkpoint18/robot_control_rosbot_xl/code_lab/verification.md`. The current normal-route run passed, but this does not make every planned package test complete.

## Heading-control verification (2026-10-09)

Coach-owned helpers under the existing code_lab record provide reproducible local checks: verify_heading_fixture.py (closed-loop plant and injected faults), verify_heading_clock.py (parameter/time guards), verify_current_task2.bash (four-segment Gazebo run), verify_current_task1.bash (ten-segment regression), and analyze_heading_run.py (log/telemetry assertions). Their commands, ROS isolation and output paths are in verification.md; these helpers are not wired into colcon test or claimed as portable hardware tests.

Cases observed: initial translation blocked until aligned/stopped; large injected yaw pauses translation; disturbed dwell reacquires the same target; all four segments return near A; odom loss and invalid values latch stop; frozen clock stops and backwards clock latches; invalid heading parameters reject startup.

## Fixed-start verification (2026-10-09)

The existing code_lab now also provides verify_fixed_start_fixture.py, verify_fixed_start_parameters.py and analyze_fixed_start.py. They check a configured A different from initial placement, rotation before translation, simultaneous x/y approach, unchanged segment numbering, targets derived from configured A, final return, invalid-coordinate rejection and updated defaults. The three fixture cases and 16 parameter cases passed. verify_current_task2.bash exercised the default A/L/W in the local Gazebo adapter; analyze_heading_run.py and analyze_fixed_start.py checked telemetry and transition order. Evidence stays in the authoritative training verification record, not in generated HTML.

## Initial laser centering (current)

The Coach-owned verify_centering_fixture.py uses isolated domain 166, a synthetic parallel corridor, best-effort scans, odometry feedback and a static 180-degree mounting TF. Two full routes check centering signs, initial body-x zero, A capture and no recentering after route start. Eight failure cases check scan loss, invalid ranges, old stamps, missing frame/TF, absent scans, travel and time bounds. verify_centering_parameters.py rejects 24 invalid limits or obsolete start_x/start_y overrides. Existing fixed-start fixtures describe the historical version and are not current acceptance tests. Real scene-2 scan windows and rear-wall sensing remain pending; no hardware acceptance is claimed.

## Side/rear initialization (current)

verify_rear_fixture.py extends the isolated fixture with a rear wall, nonzero forward/backward placement errors, and simultaneous lateral errors. It asserts rotation before translation, the combined 0.03 m/s speed cap, correct correction signs, captured A near the geometric wall-based target, four-segment completion, and independence from scans after initialization. Twelve cases passed (two routes, eight existing faults, rear loss and rear too close). Side-only fixture descriptions above are historical; current initialization requires all three windows. No real hardware movement was performed by these tests.

verify_wall_log_fixture.py exercises a full route while removing rear returns after initialization, invalidating only the left window, pausing scans, and restoring valid sides. Assertions check numeric/independent-unavailable/stale log states and route completion without scan-driven fault or recentering. These are read-only observations; no hardware behavior claim is added.

## Right-wall heading fixtures (current)

Run from the package root after building and sourcing the ROS/workspace environments:

```bash
python3 test/verify_right_wall_fixture.py
python3 test/verify_right_wall_parameters.py
```

The scripts force localhost-only ROS domains 166 and 167 and resolve the installed
executable through `ros2 pkg prefix`. Keep those domains reserved for fixtures.
Output defaults to `/tmp/distance_controller_fixture/runtime_logs`; override
`FIXTURE_OUTPUT_ROOT` to preserve evidence elsewhere. `CENTER_CASE` selects one
fixture case. These scripts are not registered with `colcon test`.

Seventeen cases cover rotated corridors on both sides of odom zero, angle wrapping,
initial rotation before translation, four rotated targets, final return, route
heading disturbance recovery, scan/TF/time/travel/rear faults, insufficient wall
span, unstable direction and excessive fit residual. Forty invalid parameter cases
include the three new wall-heading limits. The tests use synthetic measurements;
real reflective surfaces, wrong-wall selection, wheel slip and moving-scan distortion
remain cloud/hardware validation concerns. Historical fixtures above describe the
versions at their recorded dates.