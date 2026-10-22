"""Shadow-only directional observation quality; never synthesizes free space or velocities.

Angular cells describe whether scheduled measurements returned finite ranges.
They are NOT measured free-space wedges. Thresholds are uncalibrated examples.
"""

from collections import Counter
from dataclasses import asdict, dataclass
import math

TAU = 2 * math.pi


@dataclass(frozen=True)
class QualityPolicy:
    """Example thresholds, requiring separate physical qualification before any use."""

    minimum_fraction: float = 0.75
    maximum_gap_deg: float = 10.0
    persistence_s: float = 0.3
    maximum_frame_gap_s: float = 0.25
    translation_deadband: float = 0.005
    rotation_deadband: float = 0.01
    direction_reset_deg: float = 5.0

    def validate(self):
        if not all(math.isfinite(v) for v in asdict(self).values()):
            raise ValueError('Nonfinite quality policy')
        if not (
            0 < self.minimum_fraction <= 1
            and 0 < self.maximum_gap_deg <= 180
            and 0 < self.persistence_s <= 5
            and 0 < self.maximum_frame_gap_s <= 1
            and 0 <= self.translation_deadband <= 0.02
            and 0 <= self.rotation_deadband <= 0.05
            and 0 < self.direction_reset_deg <= 30
        ):
            raise ValueError('Unsupported quality policy')


def merge(intervals):
    result = []
    for lo, hi in sorted(intervals):
        if hi <= lo:
            continue
        if result and lo <= result[-1][1] + 1e-12:
            result[-1] = (result[-1][0], max(hi, result[-1][1]))
        else:
            result.append((lo, hi))
    return result


def arc(center, width):
    if width >= TAU:
        return [(0.0, TAU)]
    lo = (center - width / 2) % TAU
    hi = lo + width
    return [(lo, min(hi, TAU))] + ([(0.0, hi - TAU)] if hi > TAU else [])


def intersections(left, right):
    return merge((max(a, c), min(b, d)) for a, b in left for c, d in right)


def difference(regions, covered):
    result = []
    for lo, hi in regions:
        cursor = lo
        for a, b in covered:
            if b <= cursor or a >= hi:
                continue
            if a > cursor:
                result.append((cursor, min(a, hi)))
            cursor = max(cursor, b)
        if cursor < hi:
            result.append((cursor, hi))
    return result


def relevant_regions(candidate, measured, policy):
    regions, signatures = [], []
    for velocity in (candidate, measured):
        if len(velocity) != 3 or not all(math.isfinite(v) for v in velocity):
            raise ValueError('Invalid velocity')
        x, y, yaw = velocity
        if abs(yaw) > policy.rotation_deadband:
            return [(0.0, TAU)], ('rotation',)
        if math.hypot(x, y) > policy.translation_deadband:
            angle = math.atan2(y, x)
            regions.extend(arc(angle, math.pi))
            signatures.append(angle)
    return merge(regions), tuple(signatures)


def metrics(rays, step, candidate, measured, policy):
    """Input angles are scan-time base-axis bearings; TF must be supplied by caller."""
    policy.validate()
    if not math.isfinite(step) or not 0 < abs(step) <= math.radians(5):
        raise ValueError('Invalid scheduled angular step')
    ordered = sorted((a % TAU, d, k) for a, d, k in rays)
    if any(
        not math.isfinite(a)
        or ((k == 'finite') != (d is not None))
        or (d is not None and (not math.isfinite(d) or d <= 0))
        for a, d, k in ordered
    ):
        raise ValueError('Invalid ray')
    if any(b[0] - a[0] < 1e-10 for a, b in zip(ordered, ordered[1:])):
        raise ValueError('Duplicate bearing')
    regions, signature = relevant_regions(candidate, measured, policy)
    # No requested/measured motion: still report all-direction quality, tagged idle.
    required = bool(regions)
    regions = regions or [(0.0, TAU)]
    cells = merge(segment for a, _, k in ordered if k == 'finite' for segment in arc(a, abs(step)))
    observed = intersections(regions, cells)
    unknown = difference(regions, observed)
    widths = [b - a for a, b in unknown]
    if len(unknown) > 1 and unknown[0][0] < 1e-10 and unknown[-1][1] > TAU - 1e-10:
        widths.append(unknown[0][1] + TAU - unknown[-1][0])
    total = sum(b - a for a, b in regions)
    fraction = min(1.0, sum(b - a for a, b in observed) / total)
    return dict(
        relevant_deg=math.degrees(total),
        angular_finite_fraction=fraction,
        maximum_unknown_deg=math.degrees(max(widths, default=0.0)),
        unknown_intervals_rad=unknown,
        kinds=dict(Counter(k for _, _, k in ordered)),
        motion_required=required,
        scheduled_rays=len(ordered),
        finite_rays=sum(k == 'finite' for _, _, k in ordered),
        unknown_is_free=False,
    ), signature


class QualityTracker:
    """Consecutive bad-quality duration, not persistence of one particular obstacle."""

    def __init__(self, policy=QualityPolicy()):
        policy.validate()
        self.policy = policy
        self.last_time = self.bad_since = self.epoch = self.signature = None

    def _changed(self, signature):
        if self.signature is None or len(signature) != len(self.signature):
            return True
        for a, b in zip(signature, self.signature):
            if isinstance(a, str) or isinstance(b, str):
                if a != b:
                    return True
            elif abs(math.atan2(math.sin(a - b), math.cos(a - b))) > math.radians(
                self.policy.direction_reset_deg
            ):
                return True
        return False

    def observe(self, timestamp_s, rays, step, candidate, measured, epoch):
        """Use one explicit continuous clock/epoch; callers own live freshness checks."""
        if not math.isfinite(timestamp_s) or timestamp_s < 0 or not epoch:
            self.bad_since = self.last_time = self.signature = None
            raise ValueError('Invalid observation time/epoch')
        try:
            result, signature = metrics(rays, step, candidate, measured, self.policy)
        except ValueError:
            self.bad_since = self.last_time = self.signature = None
            raise
        reset = (
            self.last_time is None
            or self.epoch != epoch
            or timestamp_s <= self.last_time
            or timestamp_s - self.last_time > self.policy.maximum_frame_gap_s
            or self._changed(signature)
        )
        if reset:
            self.bad_since = None
        reasons = []
        if result['angular_finite_fraction'] < self.policy.minimum_fraction:
            reasons.append('low_fraction')
        if result['maximum_unknown_deg'] > self.policy.maximum_gap_deg:
            reasons.append('large_gap')
        if reasons and result['motion_required']:
            if self.bad_since is None:
                self.bad_since = timestamp_s
        else:
            self.bad_since = None
        duration = 0.0 if self.bad_since is None else timestamp_s - self.bad_since
        state = (
            'idle'
            if not result['motion_required']
            else 'unavailable'
            if not result['finite_rays']
            else 'persistent_degradation'
            if reasons and duration + 1e-9 >= self.policy.persistence_s
            else 'transient_degradation'
            if reasons
            else 'within_example_limits'
        )
        self.last_time, self.epoch, self.signature = timestamp_s, epoch, signature
        return dict(
            result,
            state=state,
            reasons=reasons,
            bad_duration_s=duration,
            continuity_reset=reset,
            policy=asdict(self.policy),
            calibrated=False,
            enforced=False,
            executable=False,
            certified_free=False,
            obstacle_constraints='must evaluate all finite points independently',
            proposed_command=candidate,
            applied_command=candidate,
        )
