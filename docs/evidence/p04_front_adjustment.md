# P04 front endpoint correction — 2026-10-10

Environment: WSL Ubuntu-22.04, ROS2 Humble, ROS_DOMAIN_ID=181, ROS_LOCALHOST_ONLY=1.
No real robot was commanded. Synthetic scan/odom/TF fixture, not Gazebo/contact proof.

- Ruff format/check on eight affected Python files: pass.
- python3 -m unittest discover -s test -p 'test_*.py': 158 tests, pass.
- colcon build --packages-select distance_controller: pass.
- WALL_CASE=front_adjust WALL_SCAN_PERIOD=.1 timeout 70 python3 test/verify_wall_session.py:
  pass; adjusted x=.295135596 (expected .30 within .005 stop tolerance);
  restart/repeat caused no further displacement; next right action y=-.192115761.
  logs: /tmp/wall_session_test/1791630765206495288.
- WALL_CASE=front_adjust_fault WALL_SCAN_PERIOD=.1 timeout 60 python3 test/verify_wall_session.py:
  obstacle stopped correction x=.276355026; next blocked; persisted pending state;
  same-odom restart completed fixed target x=.295172005.
  logs: /tmp/wall_session_test/1791630826572215551.
- git diff --check: pass.

The cloud history reported action4 stop=.117754023 and actual front=.120914278.
The new .08 m target is provisional. Left reference remains unchanged, original
completed record is retained, checkpoint cursor stays at action5. Physical
P04-P05 passage and full Task6 acceptance remain unverified.


## Stationary window repair — 2026-10-10

Same WSL/Humble isolated ROS environment. Ruff and colcon build pass.
Stationary real scan replay: python3 /tmp/settle_replay.py, 50 frames / 47 valid
front fits; all 44 full windows accepted with provided stopped yaw and .117754 m
left reference. First window .623 s, median .075890 m, spread .010650 m.
Replay does not contain odom and cannot validate timing/motion continuity.

WALL_CASE=front_adjust: /tmp/wall_session_test/1791631261954140189:
adjusted x=.290420300, unchanged after restart/repeat, next right y=-.192193896.
WALL_CASE=front_adjust_fault: /tmp/wall_session_test/1791631314583239390:
obstacle stop x=.276617575, pending/next blocked, checkpoint retry x=.290552858.
Unit cases additionally reject biased/rough/too-close windows, too few valid
fits, duplicate stamps, moving feedback, stale feedback and raw guard failures.
Final full unittest discovery: 162 tests pass. No real hardware run.
