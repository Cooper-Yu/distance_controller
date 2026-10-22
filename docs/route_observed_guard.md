# Experimental observed-obstacle decision layer

`tools/route_observed_guard.py` is a pure recommendation module. It creates no ROS node,
subscribes to nothing, publishes nothing, and is not installed or connected to Console.
The existing strict guard is unchanged. Every result has `experimental=true`,
`executable=false`, and `certified_free=false`, including `observed_clear`.
These flags must not be changed to bypass independent integration and physical acceptance.

## Input contract

Create one `ObservedGuard(explicit_policy, attached_epoch)` per attached feedback epoch.
`ObservedPolicy` requires every field, including an explicitly constructed `GuardPolicy`.
There are no quality, speed or freshness defaults. Values in the test fixture only exercise
branches; they are not settings approved for the robot. Existing maximum feedback/pairing
bounds remain enforced by validation.

`ObservedScan` contains all finite in-range obstacle points in **scan-time base axes**,
scheduled ray bearings/finite flags, angular step, paired odometry pose/stamp, original scan
stamp, first monotonic receipt and epoch. `bound_obstacle_points(bind_scan_pair(...))` keeps
both raw and filtered points, including raw points removed by a box filter and model-overlap
points. It never decides that a filter-box member is a self return. Nonfinite/out-of-range
returns remain unknown rays; they are not filled with maximum range or neighbors.

The future adapter must verify actual source identity, mounting/filter configuration,
raw/filtered pairing, correct complete point sets and ray flags before constructing inputs.
A diagnostic binding does not establish these live facts. A valid flag means an actual
in-range return from the bound acquisition, not a neighboring ray or inferred free wedge.
`ObservedFeedback` supplies current pose, source stamp, first monotonic receipt, current wall
and monotonic clocks, attached epoch, explicit odom/base_link frames and TF availability.
Measured velocity is the current body-frame velocity, not the last requested command.

`decide(scan, feedback, candidate, measured)` transforms every stored point into current base
axes, so a single 5–10 Hz scan can be assessed on multiple faster controller ticks. Equal scan
stamps may be reused with different candidate/current poses. Repeated callbacks cannot refresh
the internally retained first receipt. Same-stamp content/scan-pose-binding changes, backward
scan/time progression or epoch changes latch rejection; a new command or direction does not
clear it. Source age and original receipt age are checked on every assessment. Unavailable TF,
stale scan/odom, bad pairing, invalid geometry and invalid velocities return a zero STOP.
Current odometry also cannot move backward or change pose/measured velocity at the same source
stamp. Its first receipt is retained across reassessments; measured velocity must belong to that
odometry sample. Zero commands remain zero and do not refresh observation state.

## Decision order and scope

1. Validate feedback and observation continuity.
2. Run existing `select_command` against **all** observed points for candidate and measured
   motion. A measured-motion violation or model overlap stops even if coverage looks good.
   Pure translation away from nearby points retains the existing nondecreasing-distance rule.
3. Assess angular observation quality. Any nonzero translation requires its leading
   hemisphere; candidate and measured directions are combined. Any nonzero rotation uses all
   directions but does **not** require every ray to be valid. There is no velocity deadband.
4. Use the worst intersecting fixed angular sector's finite fraction, plus the largest
   continuous unknown gap across sector seams. Below stop quality: STOP. Between explicit
   stop/normal limits: immediate scale-down. Above normal limits: retain the obstacle-limited
   command. There is no persistence/grace timer that repeated stamps or changing directions
   can extend. Angular cells describe measurement availability, not free space.
5. Recheck the scaled command's sweep without applying the slow-band rule twice. No direction,
   route, waypoint, controller gain or odometry/drift protection is changed.

This first slice deliberately keeps the conservative full-direction quality region for
rotation. It does not yet implement outward-boundary-specific angular demand. The separate
swept-coverage diagnostic remains non-executable and is not a mandatory sampled-free proof.
Quality rotates scan-time ray directions into current axes but does not translate the ray
origin or reconstruct visibility from the current base origin. Thus it measures acquisition
direction availability, not current-origin visible/free-space coverage.
Finite points constrain only observed obstacles. Hidden, between-ray, low or nonreflective
objects remain outside this guarantee. A planar footprint is not a measurement of attachments.
The inherited constant-twist horizon/margins do not qualify braking distance: actual latency,
minimum deceleration and geometry error require bounded physical testing. No independent timer
runs here; the eventual caller must retain its watchdogs and invoke/reassess before sending.

## Local checks

Ubuntu-22.04, no ROS nodes or motion lock:

```bash
/home/cooper/.local/bin/ruff check tools/route_observed_guard.py test/test_route_observed_guard.py
/home/cooper/.local/bin/ruff format --check tools/route_observed_guard.py test/test_route_observed_guard.py
python3 -m pytest -q test/test_route_observed_guard.py \
  test/test_route_laser_guard.py::GeometryTests test/test_route_observation_quality.py \
  test/test_route_scan_binding.py
```

Tests include a single close point, a collision only in the middle of a turn, actual
`bind_scan_pair` raw-masked exterior point retention, a locally blind sector hidden by a high
global finite fraction, seam-spanning gaps, bounded degradation, retreat/model overlap,
measured speed above candidate, tiny nonzero motion, scan reuse with changing pose/command,
unchanged first receipt, same-stamp mutation, epoch/time reversal, repeated direction changes,
stale/invalid inputs and zero requests. They do not establish robot coverage or stopping bounds.

## Closed-bag numerical adapter

`tools/route_observed_guard_replay.py` is a source-only, offline CLI. It imports ROS message
serialization/TF utilities but never initializes ROS, creates a node or changes a command.
Supply `--confirm-closed` only after the recorder has exited and closed `metadata.yaml`.
The policy JSON must explicitly contain every `ObservedPolicy` and nested `GuardPolicy` field;
there are no CLI defaults or calibrated production values.

```bash
source /opt/ros/humble/setup.bash
python3 tools/route_observed_guard_replay.py --bag /absolute/closed/bag \
  --confirm-closed --policy /absolute/experimental_policy.json \
  --candidate .03 0 0 --output /absolute/new_report.jsonl
```

Use `--recorded-cmd` instead of `--candidate` to select the latest recorded command at/before
each matched scan pair's receipt. Its exact original bag timestamp is reported; repeated zero
commands do not cause repeated geometric evaluations. Missing/stale commands are unavailable.
Every new filtered scan is assessed once. Equal source-stamp messages retain their first receipt;
changed same-stamp payloads, backward source stamps or changing static TF reject the input.
Raw/filtered identities must match exactly; missing pairs/TF/bounded odom produce unavailable
rows. All in-range finite raw and filtered points survive, including masked and overlapping
points. Quality uses the filtered scheduled rays. Current measured twist comes from the same
current odom record as the feedback pose; scan geometry uses a separate bounded nearest pairing.

Optional `--filter-params FILE` and `--source-manifest FILE` hash actual supplied bytes. Missing
files remain `null`, not invented configuration/GID evidence. Even supplied files do not prove
active configuration, epoch continuity, or per-packet identity. Bag receipt timestamps are a
replay time surrogate, not real monotonic callback times; static TF only is supported. Thus
all results remain `numerical_only=true`, `executable=false`, and `certified_free=false`.
Exit 0 means reports were generated without unavailable rows, not protection acceptance.

Local full03 numerical evidence: `/tmp/task6_observed_replay_full03_20261022_final/` contains an
explicit experimental policy and six reports for translations ±.03 m/s and turns ±.10 rad/s.
Of 218 scans, one lacks an earlier recorded odom. Forward/left/right produce 216 stop and one
degraded recommendation; backward and both turns produce 217 stop recommendations. Most stops
come from measured-motion/model-margin checks on retained points (for example a rear point
near [-.18634, -.02622] m has predicted gap .01634 m against the experiment's .02 m margin).
Filter/source inputs are absent. This is not proof of collision or self-return identity, and
cannot justify deleting points or enabling protection. The next collection must resolve actual
mounting/filter/footprint evidence. Pure adapter tests cover duplicate receipts, identity/TF
changes, raw masked near points, command selection, incomplete inputs and measured-twist binding.

闭包门禁：显式 --confirm-closed、metadata.yaml 和无 SQLite WAL；同 session 存在 result.json 时必须 closed_metadata 为布尔 true。无 result 的标准包仅依赖显式关闭确认，报告 recorder_closed_metadata_verified=false。归档 supervisor.pid 不用于本机进程探测，避免跨主机 PID 复用假拒绝；该门禁不认证来源身份。
