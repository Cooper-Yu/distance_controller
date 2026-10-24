"""Pure P09 lane placement and P10 front-clearance control; no ROS or actuation.

Inputs are one time-aligned current-base planar scan. Visibility is conditional
on planar, static surfaces between adjacent rays; it is not a safety certificate.
Every close/overlapping return remains in collision and escape checks.
"""

from bisect import bisect_right
from dataclasses import asdict, dataclass
import math

from wall_geometry import BOXES, point_clearance
from route_laser_guard import GuardPolicy, sweep_samples, velocity_clearance

TAU = 2 * math.pi


@dataclass(frozen=True)
class CornerPolicy:
    front_target_m: float
    front_tolerance_m: float
    final_speed_cap_mps: float
    max_lane_correction_m: float
    clearance_m: float
    lookahead_m: float
    observation_margin_m: float
    stable_scans: int
    stable_span_s: float

    def validate(self):
        numbers = tuple(v for k, v in asdict(self).items() if k != 'stable_scans')
        if not all(type(v) in (int, float) and math.isfinite(v) for v in numbers):
            raise ValueError('Nonfinite corner policy')
        if not (
            0.06 <= self.front_target_m <= 0.15
            and 0.005 <= self.front_tolerance_m <= 0.01
            and 0 < self.final_speed_cap_mps <= 0.02
            and 0 < self.max_lane_correction_m <= 0.08
            and 0.02 <= self.clearance_m <= 0.04
            and 0.10 <= self.lookahead_m <= 0.40
            and 0.003 <= self.observation_margin_m <= 0.015
            and type(self.stable_scans) is int
            and 3 <= self.stable_scans <= 10
            and 0.3 <= self.stable_span_s <= 1.2
        ):
            raise ValueError('Unsupported corner policy')


def _answer(state, reason, evidence_id=None, **details):
    return dict(
        state=state,
        reason=reason,
        evidence_id=evidence_id,
        motion_permission=False,
        certified_free=False,
        **details,
    )


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class _Visibility:
    def __init__(self, rays, origin, margin):
        if len(origin) != 2 or not all(_finite(v) for v in origin) or not 16 <= len(rays) <= 4096:
            raise ValueError('Missing planar laser origin or bounded scheduled rays')
        self.origin, self.margin = origin, margin
        merged = {}
        for angle, valid, distance in rays:
            if (
                not _finite(angle)
                or type(valid) is not bool
                or (valid and not (_finite(distance) and distance > 0))
            ):
                raise ValueError('Invalid scheduled ray schema')
            angle = angle % TAU
            if angle in merged:
                old_valid, old_distance = merged[angle]
                valid = valid and old_valid
                distance = min(distance, old_distance) if valid else None
            merged[angle] = valid, distance
        self.rows = sorted((a, *v) for a, v in merged.items())
        self.angles = [r[0] for r in self.rows]

    def cell_supported(self, corners):
        ox, oy = self.origin
        vectors = [(x - ox, y - oy) for x, y in corners]
        if min(x for x, _ in vectors) <= 0 <= max(x for x, _ in vectors) and (
            min(y for _, y in vectors) <= 0 <= max(y for _, y in vectors)
        ):
            return False
        angles = sorted(math.atan2(y, x) % TAU for x, y in vectors)
        gaps = [(angles[(i + 1) % 4] - angles[i]) % TAU for i in range(4)]
        cut = max(range(4), key=gaps.__getitem__)
        start = angles[(cut + 1) % 4]
        end = start + TAU - gaps[cut]
        radius = max(math.hypot(x, y) for x, y in vectors)
        index = bisect_right(self.angles, start) - 1
        position = start
        for _ in range(len(self.rows) + 1):
            i = index % len(self.rows)
            a, valid, distance = self.rows[i]
            b, next_valid, next_distance = self.rows[(i + 1) % len(self.rows)]
            width = (b - a) % TAU
            if not valid or not next_valid or not 0 < width <= math.radians(1):
                return False
            cap = min(distance, next_distance) * math.cos(width / 2) - self.margin
            if radius > cap:
                return False
            # Unwrap the next sector boundary relative to the current location.
            advance = (b - position) % TAU
            if advance < 1e-12:
                advance = width
            position += advance
            if position >= end - 1e-12:
                return True
            index += 1
        return False


def _corners(box):
    x0, x1, y0, y1 = box
    return ((x0, y0), (x0, y1), (x1, y0), (x1, y1))


def _initial_cell(corners):
    return any(all(abs(x) <= hx and abs(y) <= hy for x, y in corners) for hx, hy in BOXES)


def _visible_boxes(boxes, visibility):
    cells, checked = set(), 0
    for x0, x1, y0, y1 in boxes:
        for ix in range(math.floor(x0 / 0.01), math.ceil(x1 / 0.01)):
            for iy in range(math.floor(y0 / 0.01), math.ceil(y1 / 0.01)):
                cells.add((ix, iy))
                if len(cells) > 12000:
                    return False, 'Visibility cell budget exceeded', ()
    for ix, iy in sorted(cells):
        corners = _corners((ix * 0.01, (ix + 1) * 0.01, iy * 0.01, (iy + 1) * 0.01))
        if _initial_cell(corners):
            continue
        checked += 1
        if not visibility.cell_supported(corners):
            return False, 'Unsupported swept cell', (ix * 0.01, iy * 0.01)
    return True, 'Complete outer-cell observation support', checked


def _gap_box(point, box):
    x, y = point
    x0, x1, y0, y1 = box
    dx, dy = max(x0 - x, x - x1), max(y0 - y, y - y1)
    return math.hypot(max(dx, 0), max(dy, 0)) if dx > 0 or dy > 0 else max(dx, dy)


def _translation_boxes(y, forward, padding):
    lateral, strip = [], []
    for hx, hy in BOXES:
        lateral.append(
            (-hx - padding, hx + padding, min(0, y) - hy - padding, max(0, y) + hy + padding)
        )
        strip.append((-hx - padding, forward + hx + padding, y - hy - padding, y + hy + padding))
    return lateral, strip


def _introduced_translation(y, forward):
    boxes = []
    for hx, hy in BOXES:
        if y < 0:
            boxes.append((-hx, hx, y - hy, -hy))
        elif y > 0:
            boxes.append((-hx, hx, hy, y + hy))
        if forward > 0:
            boxes.append((hx, hx + forward, y - hy, y + hy))
    return boxes


def _monotone_escape(points, y, margin):
    near = [p for p in points if point_clearance(*p) < margin]
    if not near:
        return True, False
    improved = False
    for x, py in near:
        baseline = point_clearance(x, py)
        previous = baseline
        for i in range(1, 11):
            gap = point_clearance(x, py - y * i / 10)
            if gap < previous - 1e-10:
                return False, False
            previous = gap
        improved = improved or previous > baseline + 0.001
    return improved, any(point_clearance(*p) < 0 for p in near)


def _candidate(points, y, forward, visibility, policy, *, escape=False):
    padding = policy.clearance_m + policy.observation_margin_m
    lateral, strip = _translation_boxes(y, forward, padding)
    monotone, overlaps = _monotone_escape(points, y, padding)
    for point in points:
        baseline = point_clearance(*point)
        receding = monotone and baseline < padding
        if baseline < 0 and not escape:
            return 'BLOCKED', 'Unclassified initial model overlap', point, False
        if any(_gap_box(point, box) <= 0 for box in lateral) and not receding:
            return 'BLOCKED', 'Lateral sweep intersects observed obstacle', point, False
        if forward and any(_gap_box(point, box) <= 0 for box in strip):
            return 'BLOCKED', 'Forward strip intersects observed obstacle', point, False
    if escape and (not monotone or abs(y) > 0.0200000001):
        return 'BLOCKED', 'Near returns do not improve monotonically', None, overlaps
    visible, reason, details = _visible_boxes(_introduced_translation(y, forward), visibility)
    return ('SUPPORTED' if visible else 'UNKNOWN'), reason, details, overlaps


def _rails(points, policy):
    # Local longitudinal pieces: a global +/-20deg wall fit need not succeed.
    selected = sorted((y, x) for x, y in points if 0.17 <= x <= 0.17 + policy.lookahead_m)
    groups = []
    for y, x in selected:
        if not groups or y - groups[-1][0][0] > 0.024:
            groups.append([])
        groups[-1].append((y, x))
    return [
        (min(y for y, _ in g), max(y for y, _ in g), tuple((x, y) for y, x in g))
        for g in groups
        if len(g) >= 8
        and max(x for _, x in g) - min(x for _, x in g) >= 0.08
        and max(y for y, _ in g) - min(y for y, _ in g) <= 0.024
    ]


def _forward_limit(points, y, policy):
    gaps = [
        x - hx
        for x, py in points
        for hx, hy in BOXES
        if x >= hx and abs(py - y) <= hy + policy.clearance_m
    ]
    if not gaps:
        return policy.lookahead_m
    return min(policy.lookahead_m, max(0.0, min(gaps) - policy.front_target_m))


def select_lane(points_xy, rays, *, nominal_y, laser_origin, policy, evidence_id):
    """Select a supported corridor center or <=2cm partial approach, never an action.

    nominal_y is the remaining nominal P09 lateral coordinate in current base
    axes. The adapter freezes targets in odom and tracks the original heading.
    All raw finite returns, including overlap points, must be supplied.
    """
    policy.validate()
    if not _finite(nominal_y) or not isinstance(evidence_id, str) or not evidence_id:
        raise ValueError('Finite nominal lateral prior and source evidence required')
    if (
        not points_xy
        or len(points_xy) > 8192
        or any(len(p) != 2 or not all(_finite(v) for v in p) for p in points_xy)
    ):
        return _answer('UNKNOWN', 'Missing or invalid complete observed points', evidence_id)
    visibility = _Visibility(rays, laser_origin, policy.observation_margin_m)
    rails = _rails(points_xy, policy)
    width = max(hy for _, hy in BOXES) + policy.clearance_m + policy.observation_margin_m
    candidates = []
    for right, left in zip(rails, rails[1:]):
        low, high = right[1] + width, left[0] - width
        allowed_low = max(low, nominal_y - policy.max_lane_correction_m)
        allowed_high = min(high, nominal_y + policy.max_lane_correction_m)
        if allowed_low <= allowed_high:
            center = max(allowed_low, min(allowed_high, (low + high) / 2))
            candidates.append((abs(center - nominal_y), center, low, high, right, left))
    failures = []
    for _, y, low, high, right, left in sorted(candidates)[:32]:
        forward = _forward_limit(points_xy, y, policy)
        state, reason, detail, _ = _candidate(points_xy, y, forward, visibility, policy)
        if state == 'SUPPORTED':
            return _answer(
                'TARGET_SELECTED',
                'Observed two-rail forward corridor',
                evidence_id,
                target=dict(x=0.0, y=y, yaw=0.0),
                supported_interval=(low, high),
                limiting_points=(right[2], left[2]),
                full_footprint_checked=True,
                forward_strip_m=forward,
                endpoint_accepted=False,
            )
        failures.append(dict(state=state, reason=reason, limiting_point=detail))
    if abs(nominal_y) > 0.005:
        y = math.copysign(min(0.02, abs(nominal_y)), nominal_y)
        state, reason, detail, overlaps = _candidate(
            points_xy, y, 0.0, visibility, policy, escape=True
        )
        if state == 'SUPPORTED':
            return _answer(
                'APPROACH_SELECTED',
                'Bounded approach; reobserve, not P09 arrival',
                evidence_id,
                target=dict(x=0.0, y=y, yaw=0.0),
                endpoint_accepted=False,
                escape_model_overlap_ambiguous=overlaps,
                limiting_points=tuple(sorted(points_xy, key=lambda p: point_clearance(*p))[:4]),
                coverage_detail=detail,
                full_footprint_checked=True,
                candidate_failures=failures,
            )
        failures.append(dict(state=state, reason=reason, limiting_point=detail))
    state = (
        'UNKNOWN' if not candidates or any(f['state'] == 'UNKNOWN' for f in failures) else 'BLOCKED'
    )
    return _answer(
        state,
        'No supported final corridor or bounded approach',
        evidence_id,
        candidate_failures=failures,
        limiting_points=(),
        endpoint_accepted=False,
    )


def front_command(front_gap_m, cross_error_m, heading_error_rad, *, policy):
    """No old odom endpoint input: current front body clearance is the stop residual."""
    policy.validate()
    if not all(_finite(v) for v in (front_gap_m, cross_error_m, heading_error_rad)):
        return _answer('UNKNOWN', 'Front/cross/heading unavailable', command=(0.0, 0.0, 0.0))
    residual = front_gap_m - policy.front_target_m
    if front_gap_m < policy.clearance_m or residual < -policy.front_tolerance_m:
        return _answer(
            'BLOCKED',
            'Front below accepted target; no automatic backing',
            command=(0.0, 0.0, 0.0),
            residual_m=residual,
        )
    if abs(cross_error_m) > 0.04 or abs(heading_error_rad) > 0.08:
        return _answer('BLOCKED', 'Original line/heading drift exceeded', command=(0.0, 0.0, 0.0))
    if residual <= policy.front_tolerance_m:
        return _answer(
            'VERIFY_STOP',
            'Front band reached; hold ALL axes zero',
            command=(0.0, 0.0, 0.0),
            residual_m=residual,
        )
    vx = min(policy.final_speed_cap_mps, 0.7 * residual)
    vy = max(-0.01, min(0.01, -0.7 * cross_error_m)) if abs(cross_error_m) > 0.005 else 0.0
    wz = max(-0.08, min(0.08, 1.2 * heading_error_rad)) if abs(heading_error_rad) > 0.01 else 0.0
    return _answer(
        'COMMAND',
        'Front residual control, independent of old odom endpoint',
        command=(vx, vy, wz),
        residual_m=residual,
    )


def check_front_motion(
    points_xy, rays, *, command, measured, laser_origin, policy, evidence_id, horizon_s
):
    """Check both actual twists, with coverage only for newly swept body volume.

    No broad hemisphere quality test. Invalid rays intersecting required volume
    still reject. Mixed turning is retained; no change to P15 or other actions.
    """
    policy.validate()
    if not points_xy or any(len(p) != 2 or not all(_finite(v) for v in p) for p in points_xy):
        return _answer('UNKNOWN', 'Missing complete obstacle points', evidence_id)
    if not _finite(horizon_s) or not 0.5 <= horizon_s <= 1:
        raise ValueError('Explicit front horizon must be .5 through 1 seconds')
    visibility = _Visibility(rays, laser_origin, policy.observation_margin_m)
    guard = GuardPolicy(
        hard_m=policy.clearance_m + policy.observation_margin_m,
        slow_m=max(0.035, policy.clearance_m + policy.observation_margin_m + 0.005),
        horizon_s=horizon_s,
    )
    details = []
    for velocity in (tuple(command), tuple(measured)):
        if len(velocity) != 3 or not all(_finite(v) for v in velocity):
            return _answer('UNKNOWN', 'Invalid command or measured twist', evidence_id)
        safe, gap, point = velocity_clearance(points_xy, velocity, guard)
        if not safe:
            return _answer(
                'BLOCKED',
                'Short actual/measured sweep violates clearance',
                evidence_id,
                limiting_points=(point,),
                predicted_gap_m=gap,
            )
        boxes = _front_motion_boxes(velocity, guard)
        supported, reason, detail = _visible_boxes(boxes, visibility)
        if not supported:
            return _answer(
                'UNKNOWN', reason, evidence_id, limiting_points=(), coverage_detail=detail
            )
        details.append(
            dict(
                velocity=velocity,
                covered_cells=detail,
                predicted_gap_m=gap if math.isfinite(gap) else None,
            )
        )
    return _answer(
        'SUPPORTED',
        'Both short sweeps preserve observed clearance and volume support',
        evidence_id,
        command=tuple(command),
        limiting_points=(),
        sweeps=details,
    )


def _front_motion_boxes(velocity, guard):
    vx, vy, wz = velocity
    if not any(velocity):
        return []
    if vy == 0 and wz == 0 and vx > 0:
        return _introduced_translation(0.0, vx * guard.horizon_s)
    samples, error, _ = sweep_samples(velocity, guard)
    boxes = []
    for dx, dy, yaw in samples:
        c, s = math.cos(yaw), math.sin(yaw)
        for hx, hy in BOXES:
            points = [
                (dx + c * x - s * y, dy + s * x + c * y) for x, y in _corners((-hx, hx, -hy, hy))
            ]
            boxes.append(
                (
                    min(x for x, _ in points) - error,
                    max(x for x, _ in points) + error,
                    min(y for _, y in points) - error,
                    max(y for _, y in points) + error,
                )
            )
    return boxes


class StoppedPlacementGate:
    def __init__(self, policy):
        policy.validate()
        self.policy, self.samples, self.last_stamp = policy, [], None
        self.last_payload = None
        self.fault = None

    def add(self, stamp_ns, gap, *, stopped, heading_error_rad, cross_error_m):
        if type(stamp_ns) is not int or stamp_ns <= 0:
            self.fault = 'Invalid source stamp'
        if self.last_stamp is not None and stamp_ns < self.last_stamp:
            self.fault = 'Source time rewind'
        if self.fault:
            return _answer('UNKNOWN', self.fault, accepted=False)
        sample = (gap, stopped, heading_error_rad, cross_error_m)
        if stamp_ns == self.last_stamp:
            if sample != self.last_payload:
                self.fault = 'Changed duplicate scan'
            return _answer(
                'UNKNOWN' if self.fault else 'VERIFY_STOP',
                self.fault or 'Repeated scan cannot advance gate',
                accepted=False,
            )
        if self.last_stamp is not None and stamp_ns - self.last_stamp > 500_000_000:
            self.samples.clear()
        self.last_stamp = stamp_ns
        self.last_payload = sample
        valid = (
            type(stopped) is bool
            and stopped
            and all(_finite(v) for v in (gap, heading_error_rad, cross_error_m))
            and (
                abs(gap - self.policy.front_target_m) <= self.policy.front_tolerance_m
                and abs(heading_error_rad) <= 0.01
                and abs(cross_error_m) <= 0.005
            )
        )
        if not valid:
            self.samples.clear()
            return _answer(
                'VERIFY_STOP', 'Stopped front/heading/cross band not stable', accepted=False
            )
        self.samples.append((stamp_ns, sample))
        self.samples = self.samples[-32:]
        accepted = len(self.samples) >= self.policy.stable_scans and (
            self.samples[-1][0] - self.samples[0][0] >= self.policy.stable_span_s * 1e9
        )
        return _answer(
            'TARGET_ACCEPTED' if accepted else 'VERIFY_STOP',
            'Distinct stopped source window' if accepted else 'Collecting stopped window',
            accepted=accepted,
            distinct_scans=len(self.samples),
            front_target_m=self.policy.front_target_m,
        )
