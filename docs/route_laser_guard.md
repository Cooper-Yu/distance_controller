# Optional directional laser guard

This module is **not enabled by any executable or CLI option**. The current
`task6_route_runner` remains laser-observation-only. `Console` accepts an optional
`command_guard` callable; a future explicit adapter may pass a `RouteLaserGuard`
after precise preparation. Hardware use and official Task6 avoidance acceptance
have not been established.

## Interfaces and ownership

`route_laser_guard.evaluate(snapshot, candidate, measured, policy)` is pure math.
It returns a `GuardDecision`: `clear`, `slowed`, `blocked`, `unknown`, or `stopped`.
`ScanSnapshot` contains every finite, in-range point in current base coordinates
and every scheduled ray with its validity flag. No near point is deleted.

`route_laser_runtime.RouteLaserGuard(runner, emit=None, policy=GuardPolicy())` is
callable. It reuses `WallRunner.geometry()` and `scan_pose()` for fresh stamped
laser geometry and odometry pairing, transports the point cloud from scan-time
base axes to current base axes, and evaluates both proposed and measured motion.
It never invokes `fit_wall`; rough surfaces, isolated points and corners can be
obstacles without being valid walls. Blocked/unknown decisions raise
`LaserGuardError`, so the existing action `finally` sends zeros and aborts the
route. There is no retry, obstacle bypass or waypoint modification.

`ShadowRouteLaserGuard` is a separate diagnostic adapter with the same snapshot
and evaluation. It prints `SHADOW ONLY` and emits `mode: "shadow"`,
`enforced: false`, the hypothetical decision and the unchanged `applied_command`.
Unknown, blocked and slowed laser decisions do not change the command in shadow
mode. Existing odom guards and cancellation remain active. Audit I/O failures
still propagate like other Console audit failures. Shadow success is **not**
obstacle-protection acceptance and cannot substitute for enforced tests. Neither
adapter is selected by the current production entrypoint.

The optional hook runs after the existing acceleration limiter and before
publishing. Its returned command becomes the next limiter state, so a later
release cannot jump back to the unfiltered velocity. Decreasing speed or
stopping is immediate; increasing speed still uses the existing limiter. The
default `None` hook preserves old behavior. Cleanup zero commands bypass the
hook, and a zero command is always accepted even if scans are unavailable.
Existing odom validity, ownership checks, endpoint verification and the 4 cm
turn-center drift check remain in their original locations.

## Directional decisions

- All finite points participate. Distant points may use an analytic lower bound
  when it already exceeds the slowdown margin; their returns are not discarded.
- Both rectangles of the existing chassis/wheel union are checked. For an
  already-close but nonoverlapping point, a pure translation may proceed only
  when its initial distance derivative to **each** convex rectangle is
  nondecreasing. Convexity then makes the entire constant translation
  nonapproaching. Tangential translation is allowed under the same condition.
  Another obstacle in the new direction still blocks the motion.
- This exemption does not cover rotation, even a small heading correction.
  A negative model clearance blocks all nonzero commands; it is not silently
  interpreted as sensor noise or permission to escape.
- The measured-velocity scenario must be safe independently. Reversing the
  requested command does not excuse momentum still directed toward an obstacle.
- Candidate speeds are checked at factors `1, 0.5, 0.25, 0.125`, keeping all axis
  signs and their ratio. A predicted margin below the slow threshold allows at
  most half speed. Each candidate must retain the hard margin; if none passes,
  the result is blocked. The guard never inserts a different direction.

## Parameters and evidence boundary

| Parameter | Value | Basis |
|---|---:|---|
| Hard margin | 0.020 m | Existing `WallRunner` translation/turn guards |
| Slow margin | 0.035 m | Existing turn slowdown threshold; its extension to all route motion is new |
| Prediction horizon | 0.5 s | Existing directional sweep horizon |
| Maximum scan stamp/receipt age | 0.5 s | Existing `geometry()` checks; up to 0.1 s future clock tolerance |
| Maximum scan/odom pairing offset | 0.15 s | Existing `pose_at_scan()` bound, no extrapolation |
| Maximum boundary travel per sample | 0.002 m | New numerical policy; yields at most 0.001 m between-sample clearance allowance |
| Maximum relevant angular ray gap | 1 degree | New conservative coverage policy, not hardware-calibrated |
| Maximum prediction intervals | 800 | Computation bound; exceeding it returns unknown |

Mixed translation/rotation uses exact constant body-twist SE(2) integration.
The maximum footprint boundary speed is bounded by
`hypot(vx,vy) + footprint_corner_radius * abs(wz)`. Subtracting half one sample's
travel from the sampled minimum bounds missed minima for this fixed-point,
constant-velocity planar model. It does not bound braking error, sensor error,
moving obstacles, TF/odom uncertainty or unmodelled physical attachments.

## Unknown coverage and limitations

Translation requires valid returns throughout its leading hemisphere. Any
nonzero rotation requires all angular directions, including diagonal corners.
These requirements apply to the union of commanded and measured motion. A
relevant NaN, Inf, out-of-range return, missing angular sector, stale/missing
scan, invalid mounting or failed odom pairing is unknown and blocks motion.
No-return rays are not converted to maximum range. This is intentionally stricter
than the old four-sector count test and may stop in physically open regions.
An unknown ray behind a pure forward translation is retained but does not
establish or invalidate forward coverage; reversing would require that ray.

Angular coverage is not an occupancy proof. Finite ray spacing can miss thin
objects; scan-time motion compensation transforms a cloud but does not deskew
individual rays or observe newly uncovered space. Laser height is approximately
17.3 cm in the local model, and does not establish low obstacle clearance or
vertical wall continuity. The footprint rectangles are the existing approximate
local ROSbot XL model, not a measurement of the current robot's attachments.
The 0.5 s prediction window is not a measured stopping-distance guarantee.

Before enabling this hook, require an explicit frontend/backend contract,
isolated real-node failure tests and review of real scan validity/coverage. Do
not relax missing-ray checks, delete points, alter route geometry, or infer
collision-free operation merely to make an obstructed route complete.

## Local tests

```bash
python3 -m unittest discover -s test -p test_route_laser_guard.py
python3 test/verify_route_laser_guard.py
```

The unit tests cover exact mixed motion, dense-reference sampling bounds, receding and
opposing obstacles, model overlap, measured momentum, all-axis scaling,
irregular returns, side/diagonal/turn corners, relevant invalid/absent sectors,
motion compensation, adapter failures, zero bypass, and the actual Console
limiter/hook ordering. These unit tests have no ROS or hardware side effects.

The second command is a real ROS-node fixture: it sets localhost-only domain
193, removes external CycloneDDS configuration, publishes synthetic room/odom/TF
feedback and explicitly injects an adapter into the **installed** Console and
route backend through a test-only child. It checks enforced success, stale scan,
one unknown forward ray, a model-overlapping return, cancellation and a single
unknown diagonal ray during a turn. Shadow runs demonstrate unknown/stale
evaluation is logged without altering commands. Each run checks final zero,
publisher disappearance, lock release and success/failure status. Logs remain
under `/tmp/route_laser_ros/`. This does not operate or qualify real hardware.

In particular, the single-invalid-ray turn case is an availability limitation,
not a bug to suppress: real filtered scans can contain persistent blind sectors.
The current strict policy is a diagnostic version until representative evidence
supports an explicit reviewed coverage model. No unknown ray is relaxed to free
space by shadow mode; only enforcement is absent and labelled accordingly.
