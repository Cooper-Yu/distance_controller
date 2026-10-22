"""Shadow-only swept-area sampling model. No ROS, control, endpoint or file mutation.

A finite ray contributes its measured prefix to a <=1 degree angular sampling
model. This is NOT a continuous free-space certificate: between-ray objects,
nonreflective/low obstacles and unqualified braking remain outside this model.
Missing rays are unknown. Historical support is at most 0.5 s and never erases
finite occupied evidence. Every result is non-executable, including supported.
"""

from dataclasses import asdict, dataclass
import math

import numpy as np

from route_laser_guard import GuardPolicy, RADIUS, select_command, sweep_samples
from wall_geometry import BOXES


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class LaserMount:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


@dataclass(frozen=True)
class ScanEvidence:
    stamp_ns: int
    received_s: float
    odom_stamp_ns: int
    odom_pose: Pose2
    mount: LaserMount
    epoch: str
    odom_frame: str
    base_frame: str
    laser_frame: str
    range_min: float
    range_max: float
    angular_step: float
    rays: tuple  # laser-frame angle, finite in-range distance OR explicit None (unknown)


@dataclass(frozen=True)
class CurrentState:
    pose: Pose2
    stamp_ns: int
    received_s: float
    now_ns: int
    now_s: float
    epoch: str
    odom_frame: str
    base_frame: str


def finite_pose(pose):
    if pose is None or not all(math.isfinite(v) for v in asdict(pose).values()):
        raise ValueError('Explicit finite pose/mount required')


def rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array(((c, -s), (s, c)))


def transform(points, pose):
    return np.asarray(points) @ rotation(pose.yaw).T + (pose.x, pose.y)


def untransform(points, pose):
    return (np.asarray(points) - (pose.x, pose.y)) @ rotation(pose.yaw)


def clearance(points):
    """Vector equivalent of the existing chassis/wheel union signed distance."""
    gaps = []
    for hx, hy in BOXES:
        delta = np.abs(points) - (hx, hy)
        outside = np.linalg.norm(np.maximum(delta, 0), axis=1)
        inside = np.maximum(delta[:, 0], delta[:, 1])
        gaps.append(np.where(np.any(delta > 0, axis=1), outside, inside))
    return np.minimum(*gaps)


def validate_scan(frame):
    finite_pose(frame.odom_pose)
    finite_pose(frame.mount)
    if abs(frame.mount.roll) > 1e-6 or abs(frame.mount.pitch) > 1e-6:
        raise ValueError('First slice requires verified planar laser mounting')
    if (
        not frame.epoch
        or not frame.laser_frame
        or (frame.odom_frame, frame.base_frame) != ('odom', 'base_link')
    ):
        raise ValueError('Explicit epoch and odom/base_link/laser identities required')
    if any(type(v) is not int or v <= 0 for v in (frame.stamp_ns, frame.odom_stamp_ns)):
        raise ValueError('Explicit positive integer nanosecond stamps required')
    if abs(frame.stamp_ns - frame.odom_stamp_ns) > 150_000_000:
        raise ValueError('Scan-time odometry pairing exceeds 150 ms')
    if (
        not all(
            math.isfinite(v)
            for v in (frame.received_s, frame.range_min, frame.range_max, frame.angular_step)
        )
        or not 0 < frame.range_min < frame.range_max
        or not 0 < abs(frame.angular_step)
    ):
        raise ValueError('Invalid scan timing/range/step')
    if not 1 <= len(frame.rays) <= 4000:
        raise ValueError('Missing or excessive scheduled rays')
    angles = [a % (2 * math.pi) for a, _ in frame.rays]
    if len(set(angles)) != len(angles):
        raise ValueError('Duplicate scheduled ray direction')
    for angle, distance in frame.rays:
        if not math.isfinite(angle) or (
            distance is not None
            and (not math.isfinite(distance) or not frame.range_min <= distance <= frame.range_max)
        ):
            raise ValueError('Invalid rays must be explicit unknown, never a synthetic range')


def changed(previous, frame):
    if frame.epoch != previous.epoch:
        return 'epoch_changed'
    if frame.mount != previous.mount:
        return 'mount_changed'
    if (frame.odom_frame, frame.base_frame, frame.laser_frame) != (
        previous.odom_frame,
        previous.base_frame,
        previous.laser_frame,
    ):
        return 'frame_changed'
    dt = (frame.stamp_ns - previous.stamp_ns) / 1e9
    receipt_dt = frame.received_s - previous.received_s
    if dt <= 0 or receipt_dt <= 0:
        return 'nonincreasing_scan_or_receipt'
    if dt > 0.5 or receipt_dt > 0.5:
        return 'feedback_gap'
    distance = math.hypot(
        frame.odom_pose.x - previous.odom_pose.x, frame.odom_pose.y - previous.odom_pose.y
    )
    yaw = abs(math.remainder(frame.odom_pose.yaw - previous.odom_pose.yaw, 2 * math.pi))
    if distance > 0.02 + 0.5 * dt or yaw > 0.02 + dt:
        return 'odom_discontinuity'
    return None


def demand_points(candidate, measured, policy, step):
    """Grid centers whose footprint distance decreases into the buffered future sweep.

    Includes the original hard margin, grid half-diagonal and temporal sweep error.
    Current body interior is excluded only from visibility demand; all measured
    points, including body overlaps, still enter the independent hard constraint.
    """
    if any(math.hypot(*v[:2]) > 0.5 or abs(v[2]) > 1.0 for v in (candidate, measured)):
        raise ValueError('Shadow velocity exceeds bounded analysis budget')
    radius = RADIUS + policy.horizon_s * max(math.hypot(*v[:2]) for v in (candidate, measured))
    radius += policy.hard_m + 2 * step
    axis = np.arange(-radius, radius + step, step)
    points = np.stack(np.meshgrid(axis, axis), axis=-1).reshape(-1, 2)
    baseline = clearance(points)
    required = np.zeros(len(points), dtype=bool)
    for velocity in (candidate, measured):
        samples, error, _ = sweep_samples(velocity, policy)
        minimum = baseline.copy()
        for dx, dy, yaw in samples[1:]:
            minimum = np.minimum(minimum, clearance(untransform(points, Pose2(dx, dy, yaw))))
        required |= (minimum < baseline - 1e-9) & (
            minimum <= policy.hard_m + step / math.sqrt(2) + error
        )
    return points[required & (baseline >= 0)]


def minimum_range_body_proof(frame, current_pose):
    """Analytic disk containment in the CURRENT rectangle union, not its bounding box.

    Rectangle edges partition the plane into cells wholly inside or outside the
    union. Distance to the closures of all outside cells is the exact geometric
    distance to its complement. Strict containment abstains at tangency; a small
    roundoff guard only makes the proof harder, never enlarges the body.
    """
    center = untransform(
        transform(((frame.mount.x, frame.mount.y),), frame.odom_pose), current_pose
    )[0]
    xs = [-math.inf, *sorted({sign * x for x, _ in BOXES for sign in (-1, 1)}), math.inf]
    ys = [-math.inf, *sorted({sign * y for _, y in BOXES for sign in (-1, 1)}), math.inf]
    distance = math.inf
    for x0, x1 in zip(xs, xs[1:]):
        for y0, y1 in zip(ys, ys[1:]):
            if any(x0 >= -hx and x1 <= hx and y0 >= -hy and y1 <= hy for hx, hy in BOXES):
                continue
            dx = max(x0 - center[0], 0.0, center[0] - x1)
            dy = max(y0 - center[1], 0.0, center[1] - y1)
            distance = min(distance, math.hypot(dx, dy))
    scale = max(
        1.0,
        *(abs(v) for pose in (frame.odom_pose, current_pose) for v in (pose.x, pose.y)),
        *(abs(v) for v in center),
    )
    roundoff_guard = 64 * math.ulp(scale)
    margin = distance - frame.range_min
    return dict(
        center_current_base=center.tolist(),
        range_min_m=frame.range_min,
        distance_to_body_exterior_m=distance,
        containment_margin_m=margin,
        roundoff_guard_m=roundoff_guard,
        strictly_contained=margin > roundoff_guard,
    )


def frame_support(frame, points_world, uncertainty, current_pose=None):
    in_base = untransform(points_world, frame.odom_pose)
    local = untransform(in_base, frame.mount)
    radii = np.linalg.norm(local, axis=1)
    angles = np.arctan2(local[:, 1], local[:, 0]) % (2 * math.pi)
    rays = sorted((a % (2 * math.pi), d) for a, d in frame.rays)
    beam_angles = np.array([a for a, _ in rays])
    ranges = np.array([math.nan if d is None else d for _, d in rays])
    right = np.searchsorted(beam_angles, angles) % len(rays)
    left = (right - 1) % len(rays)
    left_error = np.abs((angles - beam_angles[left] + math.pi) % (2 * math.pi) - math.pi)
    right_error = np.abs((angles - beam_angles[right] + math.pi) % (2 * math.pi) - math.pi)
    index = np.where(left_error < right_error, left, right)
    angular_error = np.minimum(left_error, right_error)
    # Nearest SCHEDULED ray, not nearest finite ray: an invalid ray keeps its aperture unknown.
    half_step = min(abs(frame.angular_step), math.radians(1)) / 2
    lower_supported = radii >= frame.range_min + uncertainty
    if (
        current_pose is not None
        and minimum_range_body_proof(frame, current_pose)['strictly_contained']
    ):
        # Only the current-body exterior is demanded. A proven wholly internal
        # blind disk cannot intersect that set; interior queries get no exemption.
        lower_supported |= clearance(untransform(points_world, current_pose)) >= 0
    return (
        (angular_error <= half_step + 1e-12)
        & np.isfinite(ranges[index])
        & lower_supported
        & (radii <= ranges[index] - uncertainty)
    )


def frame_points(frame, current):
    finite = [(a, d) for a, d in frame.rays if d is not None]
    local = np.array([(d * math.cos(a), d * math.sin(a)) for a, d in finite]).reshape(-1, 2)
    return untransform(transform(transform(local, frame.mount), frame.odom_pose), current.pose)


class CoverageHistory:
    """At most six observations, each <=0.5s old; explicit epoch required at every query."""

    def __init__(self):
        self.frames = []
        self.discard_reasons = []
        self.policy = GuardPolicy()
        self.step = 0.005

    def discard(self, reason):
        self.frames.clear()
        self.discard_reasons = (self.discard_reasons + [reason])[-16:]

    def add(self, frame):
        try:
            validate_scan(frame)
        except ValueError:
            self.discard('invalid_scan_input')
            raise
        reason = changed(self.frames[-1], frame) if self.frames else None
        if reason:
            self.discard(reason)
        self.frames.append(frame)
        self.frames = self.frames[-6:]

    def prune(self, current):
        finite_pose(current.pose)
        if any(type(v) is not int or v <= 0 for v in (current.stamp_ns, current.now_ns)):
            self.discard('invalid_current_timestamp')
            raise ValueError('Current feedback requires integer nanosecond stamps')
        if not current.epoch or (current.odom_frame, current.base_frame) != ('odom', 'base_link'):
            self.discard('current_frame_or_epoch_invalid')
            raise ValueError('Current feedback identity invalid')
        ages = ((current.now_ns - current.stamp_ns) / 1e9, current.now_s - current.received_s)
        if not all(math.isfinite(v) and -0.05 <= v <= 0.5 for v in ages):
            self.discard('current_feedback_stale')
            raise ValueError('Current feedback stale/future')
        if self.frames and current.epoch != self.frames[-1].epoch:
            self.discard('query_epoch_changed')
        if self.frames:
            last = self.frames[-1]
            dt = (current.stamp_ns - last.odom_stamp_ns) / 1e9
            displacement = math.hypot(
                current.pose.x - last.odom_pose.x, current.pose.y - last.odom_pose.y
            )
            yaw_delta = abs(math.remainder(current.pose.yaw - last.odom_pose.yaw, 2 * math.pi))
            if dt < -0.15 or displacement > 0.02 + 0.5 * abs(dt) or yaw_delta > 0.02 + abs(dt):
                self.discard('current_odom_discontinuity')
        kept = [
            f
            for f in self.frames
            if 0 <= current.now_s - f.received_s <= 0.5
            and 0 <= (current.now_ns - f.stamp_ns) / 1e9 <= 0.5
        ]
        if len(kept) != len(self.frames):
            self.discard_reasons = (self.discard_reasons + ['history_expired_or_future'])[-16:]
        self.frames = kept

    def supported_points(self, current, points):
        self.prune(current)
        if not len(points):
            return []
        world = transform(points, current.pose)
        supported = np.zeros(len(points), dtype=bool)
        for frame in self.frames:
            supported |= frame_support(frame, world, self.step / math.sqrt(2), current.pose)
        return supported.tolist()

    def assess(self, current, candidate, measured):
        report = dict(
            mode='swept_coverage_shadow',
            enforced=False,
            executable=False,
            certified_free=False,
            proposed_command=candidate,
            applied_command=candidate,
            frames_used=0,
            state='unknown',
            reason='No fresh observation',
        )
        try:
            if any(
                len(v) != 3 or not all(math.isfinite(x) for x in v) for v in (candidate, measured)
            ):
                report['proposed_command'] = report['applied_command'] = None
                raise ValueError('Invalid candidate/measured velocity')
            self.prune(current)
            points = demand_points(candidate, measured, self.policy, self.step)
            supported = np.array(self.supported_points(current, points), dtype=bool)
            clouds = [frame_points(f, current) for f in self.frames]
            obstacles = np.concatenate(clouds) if clouds else np.empty((0, 2))
            constraint = (
                select_command(
                    tuple(tuple(float(v) for v in p) for p in obstacles),
                    candidate,
                    measured,
                    self.policy,
                )
                if len(obstacles)
                else None
            )
            unknown = points[~supported]
            result = (
                'occupied'
                if constraint and constraint.state == 'blocked'
                else ('unknown' if len(unknown) or not self.frames else 'sampled_supported')
            )
            report.update(
                state=result,
                reason={
                    'occupied': 'Finite observed obstacle violates existing hard prediction',
                    'unknown': 'Required sweep samples lack finite-prefix model support',
                    'sampled_supported': 'Required samples supported only under stated sensing model',
                }[result],
                frames_used=len(self.frames),
                demand_count=len(points),
                unknown_count=len(unknown),
                unknown_examples=unknown[:12].tolist(),
                unknown_bounds=[unknown.min(axis=0).tolist(), unknown.max(axis=0).tolist()]
                if len(unknown)
                else None,
                demand_bounds=[points.min(axis=0).tolist(), points.max(axis=0).tolist()]
                if len(points)
                else None,
                observed_point_constraint=asdict(constraint) if constraint else None,
                latest_observed_point_count=len(clouds[-1]) if clouds else 0,
                retained_observed_point_count=len(obstacles),
                minimum_range_body_proofs=[
                    minimum_range_body_proof(f, current.pose) for f in self.frames
                ],
            )
        except ValueError as error:
            report['reason'] = str(error)
        report.update(
            discard_reasons=list(self.discard_reasons),
            model=dict(
                grid_m=self.step,
                max_ray_cell_deg=1.0,
                history_max_s=0.5,
                max_frames=6,
                scan_odom_pair_max_s=0.15,
                motion_horizon_s=self.policy.horizon_s,
                visibility='Finite prefixes only; planar sensor sampling model',
                limitation='No guarantee for between-ray/low/nonreflective obstacles; '
                'braking not qualified; never permission to move',
            ),
        )
        return report
