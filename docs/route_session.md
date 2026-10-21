# Supervised route session (experimental)

The default scene2 entry still executes its approved prefix and exits. It cannot
continue or return after exit. `--supervised` must be selected before preparation:

```bash
ros2 run pid_maze_solver pid_maze_solver 2 --destination P12 --supervised
```

This is a local-tested engineering mode, not a verified real-robot return tool.
It prepares P01 once and keeps the same runner, original P01, odometry monitoring,
and exclusive process lock alive. The supplied YAML gate still applies: a P12
prefix cannot continue to P13. No old JSON, checkpoint, survey_return coordinates,
or process-restart adoption is supported. Do not restart this command midroute.

Commands are `next`, `status`, `back_plan`, `back TOKEN`, and `quit`. Each `next`
executes exactly one approved action and waits stopped. There is no automatic run.
`back_plan` emits the complete proposed geometry to JSONL and prints its unique
token. After inspecting current feedback, rear/side clearance, and the planned
segment, `back TOKEN` returns only the last completed translation. The reverse
path retains all supplied bends and both actual endpoints in the original P01
frame; heading is held, not reversed by 180 degrees. The token is single use.
Success removes one history item; a repeated forward traversal gets a new visit
record instead of overwriting evidence.

A completed turn is a hard return boundary. Partial actions, failed returns,
changed stopped poses, stale previews and odometry faults are refused. Motion
failure preserves the cursor/partial evidence and terminates without retry.
This mode cannot return from P12 all the way to P01 across intervening turns and
cannot recover the interrupted P13 drift event.

The mode requires an **enforced** RouteLaserGuard (shadow is refused), caps speed
at 0.03 m/s and turn speed at 0.10 rad/s, and retains Console's feedback, arrival,
4 cm turn-center, acceleration, timeout, cancellation and final-zero protections.
Supervised path deadlines use the complete polyline length, so a return to P01
gets a travel budget even though its endpoint coordinates are zero. Default
Console deadlines remain unchanged.
The guard's current strict unknown-ray policy remains: one missing ray during
rotation can stop the action. No ray is deleted and no unknown region is treated
as clear. This availability limitation needs separate real-sensor validation.

`quit` or EOF closes the stopped session. Exit 0 means orderly session closure,
not full-route success: the audit ends `supervised_stopped`, never `route_completed`.
Inspect `supervised_session_closed.cursor` and `prefix_complete` for progress.
The operator must inspect each segment; no automatic next/back chain exists.

## Local checks

- `python3 -m unittest discover -s test -p 'test_route_session*.py'`: geometry,
  immutable-origin history, single-use preview, failure and command semantics.
- `test/verify_route_session.py`: installed pid_maze_solver -> installed backend,
  synthetic scan/odom/TF only, localhost domain 192. Normal return, return odom
  loss, return scan expiry, return cancellation, and completed-turn rejection.
- The fixture verifies final zero, no remaining cmd_vel publisher, released lock,
  unchanged P01, no false route_completed and enforced guard audit.

No physical robot or cloud execution is part of these checks.

## 2026-10-22 supervised single-segment local verification

Environment: WSL Ubuntu-22.04 / Humble, localhost-only domain 192, synthetic
scan/odom/TF; no cloud or physical robot. The installed official frontend was
used with a complete temporary YAML and `--supervised`, not a fake stdin success
parser. `bash /tmp/route_session_ros.bash` builds both packages and executes
`distance_controller/test/verify_route_session.py` with a bounded timeout.

Evidence `/tmp/route_session_ros/1792617367284358424/`:

- success: outward translation, token-confirmed return to original P01, exit 0.
- return_stale: odometry removed during return, exit 2, no returned event.
- return_scan_stale: scans withheld during return, exit 2, no returned event.
- return_cancel: SIGINT during return, frontend exit 130, no returned event.
- turn_boundary: completed turn then back_plan rejected without moving, quit exit 0.

Every case verified final zero, no remaining cmd_vel publisher, released process
lock, enforced guard audit, and absence of `route_completed`. Exit 0 in this mode
means orderly session closure, not full-route acceptance. The final optional
polyline-length deadline change was unit-tested (0.8 m return gets 110 s, default
unchanged at 30 s) and the rebuilt installed roundtrip rechecked at
`/tmp/route_session_ros/1792617536181642942/success/runtime.log`.

14 new state/runtime tests passed and are registered as CTest test_route_session;
13 existing backend and 10 console tests passed. Frontend tests now total 11.
Humble build, Ruff formatting/check and git diff --check passed. The five-case
fixture's 13-branch dispatch has a narrow PLR0912 exemption because it describes
explicit fault-injection states; production code passes without size exemptions.

The guard's strict missing-ray restriction is unchanged. No turn undo, partial
recovery, old-session adoption, P12-to-P01 full return or physical route safety
is claimed. No production waypoint, PID, drift threshold, task tag or validation
gate changed. No commit or push was performed by this implementation agent.
