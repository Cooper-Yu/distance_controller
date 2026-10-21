# Supervised reverse of a completed formal route

This engineering tool returns one **completed action** at a time. It reverses
path geometry and restores each turn's original absolute start heading; it does
not replay velocity samples. It supports a completed formal prefix such as P12,
but never a failed/partial P13 turn. It is not automatic recovery or Task7/8
acceptance. No physical robot validation is implied by local fixtures.

## Required provenance and stopped handoff

Use the exact JSON and matching JSONL from the just-completed formal runner.
The loader requires matching P01/point records, a complete ordered action list,
accepted final checks, route_completed with exit0, and no failure/partial event.
It retains the original P01 odom origin and the executed path bends. It rejects
arbitrary motion_console sessions and interrupted audits.

Before the outward process exits, retain the single /odometry/filtered publisher
GID. Confirm that odometry has not reset and the robot has not been relocated.
The return entry requires the exact source session UUID, the same current GID,
odom/base_link frames, non-regressing odom stamp, and stopped pose within 2 cm /
1 deg of the original final record. It continuously checks publisher identity
and existing timestamp/frame/jump guards. GID/pose agreement cannot prove an
undetectable internal reset; the operator's uninterrupted session observation
is required, not inferred. Only continuous-yaw representation is restored by
multiples of 2pi; neither x/y nor the original P01 is changed.

## Explicit engineering entry

```bash
ros2 run distance_controller route_return_runner \
  --source /absolute/path/to/this_run.json \
  --confirm-same-odom SOURCE_SESSION_UUID \
  --odom-publisher-gid PREVIOUSLY_OBSERVED_GID \
  --output /absolute/path/to/new_return.json \
  --laser-log-only
```

Choose exactly one policy: `--laser-log-only` explicitly provides **no laser
obstacle stop**, or `--laser-enforced` uses the separately validated guard.
Observation-only is supervised engineering operation and cannot count as Task6
avoidance acceptance. No unknown ray is reclassified to enable the enforced
policy. Maximum linear/turn speeds are 0.03 m/s and 0.10 rad/s.

The entry never calls preparation or records the current midroute pose as P01.
After attaching, it waits. `plan`/`status` show the next original action and target;
`next` executes **one** reverse action. Inspect the stopped pose, live feedback,
and physical path after each action before requesting the next one. There is no
batch/run command and no automatic advance. During motion, `stop` + Enter,
`quit` + Enter, or Ctrl+C cancels the whole return attempt and cannot resume. Translation follows the reversed
polyline with bounded connections to current and original actual stops; turns
restore their original started.start yaw. Turn drift remains limited to 4 cm;
after return turns, position must also be within 2 cm of the original start to
continue. Translation return acceptance remains 5 mm / .01 rad.

Before the first motion the tool exclusively creates source.return_claim.json
and fsyncs it. Once claimed, the source cannot start another return process.
Failures, partial turns, odom loss/reset, identity changes, cancellations, and
process crashes cannot be resumed. Do not delete the claim to bypass this gate.
`quit` during a partially completed return ends that attempt; keep the same
process alive while inspecting successive stopped segments. Closing before the
first motion does not claim the source.

Success of all reverse actions emits return_completed with original_p01 and
actual pose. Exit0 after an early quit only means orderly closure; inspect the
audit event/cursor. Faults exit nonzero, send final zeros, and release the owned
publisher and process lock. This tool never emits route_completed/Task6 PASS.

## Local verification, 2026-10-22

Ubuntu-22.04 / ROS2 Humble; localhost domain 193, synthetic sensors only.
`bash /tmp/route_return_ros.bash` built the installed return entry, ran the real
formal outbound runner, kept the same sensor publisher alive, and started the
return entry against the completed output. Evidence:
`/tmp/route_return_ros/1792618108036899160/`.

Five actual ROS cases passed:

- success: reverse bent path -> restore turn start heading -> reverse first path,
  exit0; final original-P01 error 4.881 mm and yaw .001267 rad.
- turn_drift: 45 mm injected while reversing a turn, original 4 cm guard, exit2.
- stale: odometry withheld during return, exit2.
- publisher_changed: odom publisher replaced while moving, identity rejection, exit2.
- cancel: SIGINT during return, exit2 and cancelled audit.

Every case verified final zero, no remaining cmd_vel publisher, released lock,
no preparation/rebase events and no false route_completed. Every case also
re-launched the same source and observed immediate exit2/restart refusal due to
the durable claim. The normal case preserved the original P01 and all three
completed action identities in reverse order.

The first integration attempt found a missing installed import of
`task6_route_runner.py`; CMake now installs that module alongside its executable.
The five-case rerun above passed after this packaging fix. Final checks:
`bash /tmp/route_return_final_checks.bash` rebuilt, passed both registered CTests
(14 session + 9 return unit cases), 13 existing backend tests, Ruff and git diff
checks, then imported all five actual completed source audits through the final
installed validator. Final ordering/provenance checks were tested without
rerunning unchanged motion code.

No real robot, cloud deployment, production route/threshold change, commit or
push was performed by this implementation agent. The existing return deadline
uses 30 + 3 * (polyline_length / speed + abs(path_heading) / turn_speed);
the absolute path-heading term can make the allowance conservative but only
adds time. Feedback and 4 cm drift stops remain independently active.
