# P01-P05 odom trial local verification — 2026-10-10

WSL Ubuntu-22.04 / ROS2 Humble. No real hardware commands.
Ruff format/check and git diff --check pass.
python3 -m unittest discover -s test -p 'test_*.py': 166 pass.
colcon build --packages-select distance_controller: pass.

ROS_DOMAIN_ID=181 ROS_LOCALHOST_ONLY=1 WALL_CASE=odom_trial WALL_SCAN_PERIOD=.1
timeout 180 python3 test/verify_wall_session.py: pass.
Five actions completed in one process with laser unavailable after first 2.5 cm.
Final pose (1.412998598,-.449999464,-1.560807900), target (1.403,-.45,-pi/2).
First action start through P05 completion: 79.143 s (synthetic test only).
Odom-stale injection stopped at x=.049971275 and produced no completed action.
Fixture asserts zero final command. Logs: /tmp/wall_session_test/1791632501662856097.
Initial test found integer zeros rejected by ROS Twist; fixed to float zeros and
reran the complete sequence successfully. This is control-flow/kinematic evidence,
not a maze contact simulation or physical clearance proof.

Preparation uses the unchanged existing left-wall preparation routine; this
fixture used --start current. New-entry preparation on real hardware remains
pending. Candidate route geometry and P01-P05 physical replay remain pending.
Teleop point recorder is deferred until this trial is accepted.
