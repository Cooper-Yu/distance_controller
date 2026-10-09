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
