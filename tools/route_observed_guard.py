"""Experimental observed-obstacle recommendations; no ROS or production injection.

Explicit policies are unqualified until independent replay/physical acceptance.
Unknown rays never become free space. No recommendation is executable authority.
"""

from copy import deepcopy
from dataclasses import dataclass, replace
import math

from route_laser_guard import GuardPolicy, ZERO, select_command, velocity_clearance
from route_laser_runtime import current_points
from route_observation_quality import TAU, arc, difference, intersections, merge


@dataclass(frozen=True)
class ObservedPolicy:
    """All parameters required. These limits are not calibrated production defaults."""

    obstacle: GuardPolicy
    sector_deg: float
    stop_fraction: float
    normal_fraction: float
    stop_gap_deg: float
    normal_gap_deg: float
    degraded_scale: float
    scan_age_s: float
    receipt_age_s: float
    odom_age_s: float
    pair_age_s: float
    future_s: float

    def validate(self):
        self.obstacle.validate()
        values = tuple(v for k, v in vars(self).items() if k != 'obstacle')
        if not all(type(v) in (int, float) and math.isfinite(v) for v in values):
            raise ValueError('Nonfinite or nonnumeric policy')
        if not (
            0 < self.sector_deg <= 90
            and abs(360 / self.sector_deg - round(360 / self.sector_deg)) < 1e-9
            and 0 < self.stop_fraction <= self.normal_fraction <= 1
            and 0 < self.normal_gap_deg <= self.stop_gap_deg <= 180
            and 0 < self.degraded_scale < 1
            and 0 < self.scan_age_s <= 0.5
            and 0 < self.receipt_age_s <= 0.5
            and 0 < self.odom_age_s <= 0.5
            and 0 < self.pair_age_s <= 0.15
            and 0 <= self.future_s <= 0.05
        ):
            raise ValueError('Unsupported experimental policy')


@dataclass(frozen=True)
class ObservedScan:
    """Immutable scan-time base points/ray directions; caller binds real TF and epoch.

    Points must include every valid raw/filtered return; no body-exterior mask.
    Receipt is monotonic callback time, not the time of this assessment.
    """

    points: tuple
    rays: tuple
    step_rad: float
    pose: object
    stamp_ns: int
    paired_odom_stamp_ns: int
    received_s: float
    epoch: str


@dataclass(frozen=True)
class ObservedFeedback:
    """Current odometry and clocks, independently checked by the eventual adapter."""

    pose: object
    stamp_ns: int
    received_s: float
    now_ns: int
    now_s: float
    epoch: str
    frames: tuple
    tf_valid: bool


@dataclass(frozen=True)
class ObservedDecision:
    command: tuple
    state: str
    reason: str
    obstacle: object = None
    sectors: tuple = ()
    experimental: bool = True
    executable: bool = False
    certified_free: bool = False


def bound_obstacle_points(binding):
    """Keep BOTH point sets from bind_scan_pair; still in scan-time base axes.

    This consumes diagnostic provenance only; it does not validate live config,
    remove self returns or make an unverified binding executable.
    """
    if binding.get('schema') != 1 or binding.get('mode') != 'scan_binding_shadow':
        raise ValueError('Expected scan binding diagnostic')
    points = []
    for ray in binding['rays']:
        for key in ('raw_point_base_xyz', 'filtered_point_base_xyz'):
            point = ray[key]
            if point is not None:
                if len(point) != 3 or not all(math.isfinite(v) for v in point):
                    raise ValueError('Invalid bound point')
                points.append(tuple(point[:2]))
    return tuple(points)


def motion_regions(candidate, measured):
    """No velocity deadband. Rotation requires distributed quality, not every ray."""
    regions = []
    for vx, vy, wz in (candidate, measured):
        if wz != 0:
            return [(0.0, TAU)]
        if vx != 0 or vy != 0:
            regions.extend(arc(math.atan2(vy, vx), math.pi))
    return merge(regions)


def largest_gap(intervals):
    widths = [b - a for a, b in intervals]
    if len(intervals) > 1 and intervals[0][0] < 1e-10 and intervals[-1][1] > TAU - 1e-10:
        widths.append(intervals[0][1] + TAU - intervals[-1][0])
    return math.degrees(max(widths, default=0.0))


def sector_quality(rays, step, candidate, measured, policy):
    """Worst fixed angular sector, plus uncut relevant gap across sector seams."""
    required = motion_regions(candidate, measured)
    cells = merge(segment for a, valid in rays if valid for segment in arc(a, abs(step)))
    rows = []
    width = math.radians(policy.sector_deg)
    for i in range(round(360 / policy.sector_deg)):
        region = intersections(required, [(i * width, (i + 1) * width)])
        total = sum(b - a for a, b in region)
        if total <= 1e-12:
            continue
        observed = intersections(region, cells)
        fraction = min(1.0, sum(b - a for a, b in observed) / total)
        rows.append((i, fraction, largest_gap(difference(region, cells))))
    gap = largest_gap(difference(required, cells))
    worst = min((fraction for _, fraction, _ in rows), default=0.0)
    if worst < policy.stop_fraction or gap > policy.stop_gap_deg:
        return 'stop', tuple(rows)
    if worst < policy.normal_fraction or gap > policy.normal_gap_deg:
        return 'degraded', tuple(rows)
    return 'observed_clear', tuple(rows)


def validate_geometry(scan, feedback):
    for pose in (scan.pose, feedback.pose):
        if not all(math.isfinite(v) for v in (pose.x, pose.y, pose.yaw)):
            raise ValueError('Invalid pose')
    if not scan.points or any(
        len(p) != 2 or not all(math.isfinite(v) for v in p) for p in scan.points
    ):
        raise ValueError('Missing or invalid observed points')
    if not math.isfinite(scan.step_rad) or not 0 < abs(scan.step_rad) <= math.radians(5):
        raise ValueError('Invalid ray step')
    if not scan.rays or any(
        len(r) != 2 or not math.isfinite(r[0]) or type(r[1]) is not bool for r in scan.rays
    ):
        raise ValueError('Missing or invalid rays')
    angles = sorted(a % TAU for a, _ in scan.rays)
    circular = [*angles[1:], angles[0] + TAU]
    if any(b - a < 1e-10 for a, b in zip(angles, circular)):
        raise ValueError('Duplicate ray bearing')
    if feedback.frames != ('odom', 'base_link') or feedback.tf_valid is not True:
        raise ValueError('Unavailable frame or TF binding')


def check_ages(scan, feedback, policy):
    stamps = (scan.stamp_ns, scan.paired_odom_stamp_ns, feedback.stamp_ns, feedback.now_ns)
    if any(type(v) is not int or v <= 0 for v in stamps):
        raise ValueError('Invalid source timestamp')
    if not all(math.isfinite(v) for v in (scan.received_s, feedback.received_s, feedback.now_s)):
        raise ValueError('Invalid monotonic time')
    for stamp, limit in (
        (scan.stamp_ns, policy.scan_age_s),
        (feedback.stamp_ns, policy.odom_age_s),
    ):
        if not -policy.future_s <= (feedback.now_ns - stamp) / 1e9 <= limit:
            raise ValueError('Source stale or future')
    for receipt, limit in (
        (scan.received_s, policy.receipt_age_s),
        (feedback.received_s, policy.odom_age_s),
    ):
        if not 0 <= feedback.now_s - receipt <= limit:
            raise ValueError('Receipt stale or future')
    if abs(scan.stamp_ns - scan.paired_odom_stamp_ns) / 1e9 > policy.pair_age_s:
        raise ValueError('Scan odometry pairing unavailable')


class ObservedGuard:
    """Pure recommendation state: repeated scans are reusable, never refreshed.

    Construct once per explicitly attached epoch. No persistence grace or retry.
    Caller must continue invoking/independently watch feedback; this is no timer.
    """

    def __init__(self, policy, epoch):
        policy.validate()
        if not isinstance(epoch, str) or not epoch.strip():
            raise ValueError('Explicit epoch required')
        self.policy, self.epoch = policy, epoch
        self.scan = None
        self.feedback = self.measured = None
        self.last_now = None
        self.identity_fault = None

    def freeze_odom(self, feedback, measured):
        """Bind measured twist to current odometry; duplicate callbacks cannot renew age."""
        if self.feedback:
            if feedback.stamp_ns < self.feedback.stamp_ns:
                self.identity_fault = 'Odometry stamp moved backward'
            elif feedback.stamp_ns == self.feedback.stamp_ns:
                feedback = replace(feedback, received_s=self.feedback.received_s)
                original_times = replace(
                    feedback,
                    now_ns=self.feedback.now_ns,
                    now_s=self.feedback.now_s,
                )
                if original_times != self.feedback or tuple(measured) != self.measured:
                    self.identity_fault = 'Same odometry stamp changed pose or measured velocity'
        if self.identity_fault:
            raise ValueError(self.identity_fault)
        return feedback

    def bind(self, scan, feedback, measured):
        if self.identity_fault:
            raise ValueError(self.identity_fault)
        if scan.epoch != self.epoch or feedback.epoch != self.epoch:
            self.identity_fault = 'Epoch changed'
        elif self.last_now and (
            feedback.now_ns < self.last_now[0] or feedback.now_s < self.last_now[1]
        ):
            self.identity_fault = 'Clock moved backward'
        elif self.scan and scan.stamp_ns < self.scan.stamp_ns:
            self.identity_fault = 'Scan stamp moved backward'
        elif self.scan and scan.stamp_ns == self.scan.stamp_ns:
            # Assessment time and duplicate callbacks cannot refresh the first receipt.
            scan = replace(scan, received_s=self.scan.received_s)
            if scan != self.scan:
                self.identity_fault = 'Same stamp changed scan content or pose binding'
        if self.identity_fault:
            raise ValueError(self.identity_fault)
        feedback = self.freeze_odom(feedback, measured)
        check_ages(scan, feedback, self.policy)
        self.scan = deepcopy(scan)
        self.feedback, self.measured = deepcopy(feedback), tuple(measured)
        self.last_now = feedback.now_ns, feedback.now_s
        return scan, feedback

    def decide(self, scan, feedback, candidate, measured):
        """Return experimental STOP/scale; no path changes or physical authorization."""
        try:
            if any(
                len(v) != 3 or not all(math.isfinite(x) for x in v) for v in (candidate, measured)
            ):
                raise ValueError('Invalid velocity')
            if not any(candidate):
                return ObservedDecision(ZERO, 'stopped', 'Zero command requested')
            validate_geometry(scan, feedback)
            scan, feedback = self.bind(scan, feedback, measured)
            points = current_points(tuple((*p, 0) for p in scan.points), scan.pose, feedback.pose)
            obstacle = select_command(points, candidate, measured, self.policy.obstacle)
            if obstacle.state == 'blocked':
                return ObservedDecision(ZERO, 'stop', obstacle.reason, obstacle)
            rays = tuple((a + scan.pose.yaw - feedback.pose.yaw, valid) for a, valid in scan.rays)
            quality, sectors = sector_quality(rays, scan.step_rad, candidate, measured, self.policy)
            if quality == 'stop':
                return ObservedDecision(
                    ZERO, 'stop', 'Insufficient directional observations', obstacle, sectors
                )
            scale = self.policy.degraded_scale if quality == 'degraded' else 1.0
            limited = tuple(v * scale for v in obstacle.command)
            # Recheck the scaled path without applying the slow-band rule twice.
            safe, gap, point = velocity_clearance(points, limited, self.policy.obstacle)
            if not safe:
                return ObservedDecision(
                    ZERO, 'stop', 'Scaled path violates margin', obstacle, sectors
                )
            final = replace(
                obstacle,
                command=limited,
                scale=obstacle.scale * scale,
                limiting_point=point,
                predicted_gap_m=gap if math.isfinite(gap) else None,
            )
            state = (
                'degraded'
                if quality == 'degraded' or final.command != tuple(candidate)
                else 'observed_clear'
            )
            return ObservedDecision(
                final.command,
                state,
                'Observed-point and directional-quality limits',
                final,
                sectors,
            )
        except (ValueError, TypeError, AttributeError, ArithmeticError) as error:
            return ObservedDecision(ZERO, 'stop', str(error))
