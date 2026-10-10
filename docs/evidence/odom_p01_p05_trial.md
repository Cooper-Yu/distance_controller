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


## Named waypoint batching — 2026-10-10
--until P04 runs four ordered actions, then returns to the prompt at rest.
run_to P05 subsequently executes only action five. Unique waypoint labels support
future taught segments without hard-coded P05 limits. P03 is arrival before turn.
Invalid/passed labels and partial actions refuse a new batch; failures abort it.
Ruff, 168 unittest tests, colcon build, and git diff --check passed in Ubuntu-22.04.
WALL_CASE=odom_trial WALL_SCAN_PERIOD=.1 ROS_DOMAIN_ID=181 ROS_LOCALHOST_ONLY=1
 timeout 180 python3 test/verify_wall_session.py passed:
P04 stop at completed_actions=4, P05 stop at 5, final zero velocity;
final pose (1.412998204,-.479999466,-1.560800378).
Odom-stale batch stops on action one with no completed action (x=.049967966).
Logs: /tmp/wall_session_test/1791633603829436020.
This is a synthetic ROS fixture, not real maze clearance validation.


## Explicit preparation wall — 2026-10-10
Added --alignment-wall right (default remains left). User stationary scan contains
37 frames over 3.898 s; matching dominant-line/TLS replay with known laser transform
passes left heading in 3 frames, right heading in all 37 at the default 30-degree
half-window. Physical cause of non-line returns remains unknown.
Ruff, 168 unit tests, colcon and diff checks pass. WALL_CASE=odom_trial_prepare
with WALL_SCAN_PERIOD=.1 in isolated domain181 passes C++ right alignment and
handoff into odom_trial, without executing a route. Logs:
/tmp/wall_session_test/1791637696289867694. Real robot verification pending.


## Taught P01-P09 prefix — 2026-10-10
Independent task6_taught_p01_p09.json: eight translations plus one P03 clockwise
turn; 104 interpolated targets at <=5 cm. Source sample simplification deviation
max 0.007893692 m; this measures polyline fit, not robot accuracy.
Ubuntu-22.04: ruff format/check on affected Python tests passed; python3 -m unittest
discover -s test -p 'test_*.py' passed 169 tests; colcon build --packages-select
distance_controller passed. WALL_CASE=odom_trial_taught WALL_SCAN_PERIOD=.1
ROS_DOMAIN_ID=181 ROS_LOCALHOST_ONLY=1 timeout 180 python3 test/verify_wall_session.py
passed all nine actions, P09 boundary and no additional action after next.
Final synthetic pose=(1.335843080,-1.547147808,-1.560825419), within 2 cm/.02 rad.
Logs: /tmp/wall_session_test/1791640176154093915; laser absent after initial motion.
This validates synthetic ROS execution, not maze clearance. P09-P10 collision and
later uncertain/reset data excluded. Physical verification and Task6 remain pending.
