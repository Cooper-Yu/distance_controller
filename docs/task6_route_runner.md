# Task6 scene-2 route backend

`task6_route_runner` is a noninteractive execution interface for the
`pid_maze_solver` scene-2 frontend. This is an intermediate implementation:
laser scans are required for precise initialization and recorded during the
route, but route motion has no laser obstacle avoidance. It does not establish
official Task6 completion, physical clearance, or independent learner competence.

```bash
ros2 run distance_controller task6_route_runner \
  --scene 2 --route-json /path/to/validated_manifest.json --laser-log-only \
  --output /path/to/new_session.json
```

The frontend owns YAML review status and destination-prefix selection. The
backend accepts only this manifest structure, validates every action before
creating ROS ownership, and executes its supplied coordinates:

```json
{"schema":1,"scene":2,"frame":"P01","actions":[
  {"name":"P02","kind":"path","heading_rad":0.0,"points":[[0,0],[0.04,0]]}
]}
```

Headings are absolute relative to P01. A path begins at the previous path's
endpoint and keeps the heading established by the preceding turn (initially
zero). Explicit turn actions contain `name`, `kind: "turn"`, and `heading_rad`.
The runner preserves all supplied intermediate points; it never loads the
console's hard-coded route or inserts a final half-turn. Full frontend input
normally ends at P15; a reviewed prefix ends at its final supplied action.
Paths require nonzero edges of at most 5 cm and total length at most 2 m per
action. The backend does not read stdin or silently continue after an error.

Every invocation performs the existing precise preparation with right-wall
heading, right-wall distance 26 cm and rear-wall distance 28 cm. It waits for
the paused C++ controller's verified A state, sends `finish`, waits for a
successful process exit, then records the fresh stopped pose as P01. There is
no skip-preparation option. Existing 5 mm / 0.5 degree preparation tolerances,
bounded retries and 60 second overall preparation guard remain in force.

Route execution reuses `Console.move`, `PathTracker`, stopped-pose verification,
and `TeachingBook`. Fresh odometry, exclusive command ownership and the 4 cm
turn-center drift guard remain required. A path following a turn requires the
existing 2 cm / 1 degree handoff gate; only its first point changes to the actual
post-turn position, while all subsequent supplied points and the endpoint stay
fixed. If the new first edge exceeds 5 cm, only that edge receives collinear
interpolation points to restore the spacing limit; its geometry stays the same.
This is a bounded odometry correction, not a clearance proof.

Optional `--speed` defaults to 0.04 m/s (range 0.01–0.12); `--turn-speed` defaults
to 0.10 rad/s (range 0.02–0.60). Missing/stale odometry, competing publishers,
motion timeout, excessive drift, failed final verification, SIGINT or SIGTERM
abort the route. Owned motion publishes zero for one second before releasing
its publisher. Cancellation stays latched across the preparation handoff.

The output `.json` stores same-session points and `.jsonl` stores the complete
input actions, initialization handoff, per-action checks, observations and final
status. Neither output may already exist. Exit 0 and `route_completed` require
all actions, recording, stopped feedback and cleanup to succeed. Failure or
cancellation returns 2 with `route_failed` when the audit file is available;
the frontend propagates the backend's status.

## Local verification

In Ubuntu-22.04 with Humble and the workspace sourced:

```bash
python3 -m unittest discover -s test -p test_task6_route_runner.py
python3 test/verify_task6_route_runner.py
```

The second script sets localhost-only ROS domain 189 and removes external
CycloneDDS configuration. It creates synthetic odometry, laser scans and a
static sensor transform, then launches the installed runner with closed stdin.
It checks preparation plus a supplied bent route, stale-odom failure, excessive
turn drift, route cancellation and preparation cancellation. Logs remain under
`/tmp/task6_route_test/`; this fixture is not a hardware test.
