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
