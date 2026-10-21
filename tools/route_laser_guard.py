"""Optional directional scan guard mathematics; no ROS, wall fitting or route editing.

Coverage denotes observed laser rays, not a certified free-space volume. The
footprint is the existing planar model; unseen heights and between-ray objects
remain outside this model. No invalid ray is promoted to a free-space return.
"""

from dataclasses import dataclass
import math

from wall_geometry import BOXES, point_clearance

ZERO = (0.0, 0.0, 0.0)
RADIUS = max(math.hypot(x, y) for x, y in BOXES)


@dataclass(frozen=True)
class GuardPolicy:
    """Existing margins plus explicit experimental coverage/numerical limits."""

    hard_m: float = 0.02
    slow_m: float = 0.035
    horizon_s: float = 0.5
    sample_motion_m: float = 0.002
    max_ray_gap_rad: float = math.radians(1)

    def validate(self):
        values = (
            self.hard_m,
            self.slow_m,
            self.horizon_s,
            self.sample_motion_m,
            self.max_ray_gap_rad,
        )
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Nonfinite guard policy')
        if not (
            0.02 <= self.hard_m < self.slow_m
            and 0.5 <= self.horizon_s <= 1
            and 0 < self.sample_motion_m <= 0.002
            and 0 < self.max_ray_gap_rad <= math.radians(1)
        ):
            raise ValueError('Unsupported guard policy bounds')


@dataclass(frozen=True)
class ScanSnapshot:
    """All finite valid returns in current base axes, plus every scheduled ray."""

    points: tuple  # (x, y); no close points removed
    rays: tuple  # (bearing in current base axes, finite/in-range return)


@dataclass(frozen=True)
class GuardDecision:
    command: tuple
    state: str
    reason: str
    scale: float = 0.0
    limiting_point: tuple | None = None
    predicted_gap_m: float | None = None


def body_displacement(velocity, duration):
    """Exact constant body-twist SE(2) integration, including simultaneous turning."""
    vx, vy, wz = velocity
    angle = wz * duration
    if abs(wz) < 1e-10:
        return vx * duration, vy * duration, angle
    sine, cosine = math.sin(angle) / wz, (1 - math.cos(angle)) / wz
    return sine * vx - cosine * vy, cosine * vx + sine * vy, angle


def receding_translation(point, velocity):
    """Certify nondecreasing point distance to both convex rectangles for pure XY motion.

    Outside each rectangle, nonnegative initial distance derivative suffices for
    constant translation. Requiring both rectangles preserves their union. This
    exemption does not apply to rotating motion or model-overlapping points.
    """
    vx, vy, wz = velocity
    if abs(wz) > 1e-10:
        return False
    x, y = point
    for hx, hy in BOXES:
        dx, dy = x - max(-hx, min(hx, x)), y - max(-hy, min(hy, y))
        if dx * vx + dy * vy > 1e-12:
            return False
    return True


def sweep_samples(velocity, policy):
    """Bound between-sample clearance error using maximum footprint boundary speed."""
    speed = math.hypot(*velocity[:2]) + RADIUS * abs(velocity[2])
    count = max(1, math.ceil(policy.horizon_s * speed / policy.sample_motion_m))
    if count > 800:
        raise ValueError('Velocity exceeds guard numerical budget')
    samples = tuple(
        body_displacement(velocity, policy.horizon_s * i / count) for i in range(count + 1)
    )
    error = speed * policy.horizon_s / count / 2
    return samples, error, speed * policy.horizon_s


def predicted_gap(point, baseline, samples, error, travel_bound, policy):
    # Every point participates. Far returns use a conservative analytic bound,
    # avoiding costly sampling only when even that bound exceeds the slow margin.
    if baseline - travel_bound >= policy.slow_m:
        return baseline - travel_bound
    x, y = point
    result = math.inf
    for dx, dy, yaw in samples:
        c, s = math.cos(yaw), math.sin(yaw)
        px, py = x - dx, y - dy
        result = min(result, point_clearance(c * px + s * py, -s * px + c * py))
    return result - error


def velocity_clearance(points, velocity, policy):
    """Return safe flag, approaching-point margin and limiting point for one scenario."""
    samples, error, travel_bound = sweep_samples(velocity, policy)
    limiting, minimum = None, math.inf
    for point in points:
        baseline = point_clearance(*point)
        if baseline < 0:
            return False, baseline, point
        if receding_translation(point, velocity):
            continue
        gap = predicted_gap(point, baseline, samples, error, travel_bound, policy)
        if gap < minimum:
            minimum, limiting = gap, point
        if gap < policy.hard_m:
            return False, gap, point
    return True, minimum, limiting


def requires_ray(bearing, velocity):
    vx, vy, wz = velocity
    return abs(wz) > 1e-10 or vx * math.cos(bearing) + vy * math.sin(bearing) > 1e-12


def required_gap(start, end, velocity):
    """Does a circular angular hole intersect the leading hemisphere or a rotation?"""
    if abs(velocity[2]) > 1e-10:
        return True
    if math.hypot(*velocity[:2]) <= 1e-12:
        return False
    if requires_ray(start, velocity) or requires_ray(end, velocity):
        return True
    direction = math.atan2(velocity[1], velocity[0]) % (2 * math.pi)
    return any(start <= direction + offset <= end for offset in (0, 2 * math.pi))


def coverage_error(rays, scenarios, policy):
    """Invalid relevant rays and missing angular coverage are unknown, never free.

    Translation requires its leading hemisphere; any rotation requires all
    directions because different body corners sweep outward simultaneously.
    """
    if not rays:
        return 'No scheduled laser rays'
    for bearing, valid in rays:
        if not math.isfinite(bearing):
            return 'Invalid ray geometry'
        if not valid and any(requires_ray(bearing, v) for v in scenarios):
            return 'Unknown laser return in motion coverage'
    angles = sorted(bearing % (2 * math.pi) for bearing, _ in rays)
    for start, end in zip(angles, (*angles[1:], angles[0] + 2 * math.pi)):
        if end - start > policy.max_ray_gap_rad + 1e-9 and any(
            required_gap(start, end, v) for v in scenarios
        ):
            return 'Missing angular coverage in motion direction'
    return None


def evaluate(snapshot, candidate, measured, policy=GuardPolicy()):
    """Scale only the supplied velocity, or return a blocked/unknown zero decision.

    Measured motion is checked independently. A near point that recedes under
    both pure translations cannot veto backing away; model overlap still blocks.
    No retry, synthetic obstacle removal, direction change or route change occurs.
    """
    policy.validate()
    if len(candidate) != 3 or not all(math.isfinite(v) for v in candidate):
        return GuardDecision(ZERO, 'unknown', 'Invalid candidate velocity')
    if not any(candidate):
        return GuardDecision(ZERO, 'stopped', 'Zero command always permitted', 1.0)
    if len(measured) != 3 or not all(math.isfinite(v) for v in measured):
        return GuardDecision(ZERO, 'unknown', 'Invalid measured velocity')
    if not snapshot.points or any(
        len(p) != 2 or not all(math.isfinite(x) for x in p) for p in snapshot.points
    ):
        return GuardDecision(ZERO, 'unknown', 'No usable obstacle points or invalid point geometry')
    reason = coverage_error(snapshot.rays, (candidate, measured), policy)
    if reason:
        return GuardDecision(ZERO, 'unknown', reason)
    try:
        return select_command(snapshot.points, candidate, measured, policy)
    except ValueError as error:
        return GuardDecision(ZERO, 'unknown', str(error))


def select_command(points, candidate, measured, policy):
    safe, measured_gap, point = velocity_clearance(points, measured, policy)
    if not safe:
        return GuardDecision(
            ZERO,
            'blocked',
            'Measured motion or model overlap violates margin',
            limiting_point=point,
            predicted_gap_m=measured_gap,
        )
    last_gap, last_point = None, None
    for scale in (1.0, 0.5, 0.25, 0.125):
        command = tuple(v * scale for v in candidate)
        safe, gap, point = velocity_clearance(points, command, policy)
        last_gap, last_point = gap, point
        # Enter the slow band at no more than half of the proposed speed. Every
        # candidate is checked again; smaller positive scales never relax 2 cm.
        if safe and (scale < 1 or min(gap, measured_gap) >= policy.slow_m):
            return GuardDecision(
                command,
                'clear' if scale == 1 else 'slowed',
                'Directional prediction accepted',
                scale,
                point,
                gap if math.isfinite(gap) else None,
            )
    return GuardDecision(
        ZERO,
        'blocked',
        'No bounded positive speed preserves margin',
        limiting_point=last_point,
        predicted_gap_m=last_gap if math.isfinite(last_gap) else None,
    )
