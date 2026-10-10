"""Planar scan geometry and model-based body clearances; no ROS side effects."""

from dataclasses import dataclass
import math
from statistics import median

DIRECTIONS = {'front': 0.0, 'left': math.pi / 2, 'right': -math.pi / 2, 'rear': math.pi}
# Union of chassis and wheel rectangles from the local ROSbot XL model.
BOXES = ((0.170, 0.135), (0.135, 0.160))


def support(nx, ny):
    """Farthest body projection along a wall normal, including wheels, in meters."""
    return max(abs(nx) * x + abs(ny) * y for x, y in BOXES)


def point_clearance(x, y):
    """Signed distance of a base_link laser point from the chassis/wheel envelope."""
    gaps = []
    for hx, hy in BOXES:
        dx, dy = abs(x) - hx, abs(y) - hy
        gaps.append(math.hypot(max(dx, 0), max(dy, 0)) if dx > 0 or dy > 0 else max(dx, dy))
    return min(gaps)


def scan_points(scan, transform):
    """Transform finite ranges from laser to base_link using full mounting rotation/offset."""
    q, t = transform.rotation, transform.translation
    norm = math.sqrt(sum(v * v for v in (q.x, q.y, q.z, q.w)))
    if not math.isfinite(norm) or abs(norm - 1) > 0.01:
        raise ValueError('Invalid laser transform quaternion')
    x, y, z, w = (v / norm for v in (q.x, q.y, q.z, q.w))
    if abs(1 - 2 * (x * x + y * y)) < math.cos(math.radians(5)):
        raise ValueError('Laser plane tilt exceeds 5 degrees')
    if not all(
        math.isfinite(v)
        for v in (
            t.x,
            t.y,
            t.z,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
        )
    ):
        raise ValueError('Nonfinite laser geometry')
    if scan.angle_increment == 0 or not 0 < scan.range_min < scan.range_max:
        raise ValueError('Invalid scan bounds')
    points, counts = [], dict.fromkeys((*DIRECTIONS, 'left_wide', 'right_wide', 'front_narrow'), 0)
    for i, distance in enumerate(scan.ranges):
        angle = scan.angle_min + i * scan.angle_increment
        c, s = math.cos(angle), math.sin(angle)
        dx = (1 - 2 * (y * y + z * z)) * c + 2 * (x * y - z * w) * s
        dy = 2 * (x * y + z * w) * c + (1 - 2 * (x * x + z * z)) * s
        bearing = math.atan2(dy, dx)
        for name, center in DIRECTIONS.items():
            if abs(
                math.atan2(math.sin(bearing - center), math.cos(bearing - center))
            ) <= math.radians(20):
                counts[name] += 1
        if abs(bearing) <= math.radians(12):
            counts['front_narrow'] += 1
        for side in ('left', 'right'):
            if abs(
                math.atan2(
                    math.sin(bearing - DIRECTIONS[side]), math.cos(bearing - DIRECTIONS[side])
                )
            ) <= math.radians(30):
                counts[side + '_wide'] += 1
        if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
            points.append((t.x + distance * dx, t.y + distance * dy, bearing))
    return points, counts


@dataclass(frozen=True)
class Wall:
    """Fitted wall clearance/normal relative to the entire body, meters and radians."""

    gap: float
    angle: float
    rms: float
    span: float
    count: int
    raw_count: int


def dominant_wall(points, minimum_baseline, tolerance=0.012):
    """Select >=80% consensus within the supplied residual band (default 12 mm).

    This selection is for wall estimation only. Raw obstacle points remain unchanged.
    """
    best = []
    stride = max(1, (len(points) + 31) // 32)
    for i in range(0, len(points), stride):
        for j in range(i + stride, len(points), stride):
            dx, dy = points[j][0] - points[i][0], points[j][1] - points[i][1]
            length = math.hypot(dx, dy)
            if length < minimum_baseline:
                continue
            inliers = [
                (x, y)
                for x, y in points
                if abs(-dy * (x - points[i][0]) + dx * (y - points[i][1])) / length <= tolerance
            ]
            if len(inliers) > len(best):
                best = inliers
    if len(best) < 8 or len(best) * 5 < len(points) * 4:
        raise ValueError('No dominant wall with 80 percent consensus')
    return best


def fit_wall(points, counts, side, *, tolerance=0.012):
    """TLS line fit in a +/-20 degree window; reject sparse, short, rough or angled surfaces."""
    center = DIRECTIONS[side]
    pts = [
        (x, y)
        for x, y, a in points
        if abs(math.atan2(math.sin(a - center), math.cos(a - center))) <= math.radians(20)
    ]
    if len(pts) < 8 or len(pts) < 0.6 * counts[side]:
        raise ValueError(f'{side}: insufficient wall returns')
    raw_count = len(pts)
    pts = dominant_wall(pts, 0.08, tolerance)
    if len(pts) < 0.6 * counts[side]:
        raise ValueError(f'{side}: insufficient inlier coverage')
    mx = sum(x for x, y in pts) / len(pts)
    my = sum(y for x, y in pts) / len(pts)
    xx = sum((x - mx) ** 2 for x, y in pts)
    yy = sum((y - my) ** 2 for x, y in pts)
    xy = sum((x - mx) * (y - my) for x, y in pts)
    tangent = 0.5 * math.atan2(2 * xy, xx - yy)
    nx, ny = -math.sin(tangent), math.cos(tangent)
    if nx * mx + ny * my < 0:
        nx, ny = -nx, -ny
    angle = math.atan2(math.sin(math.atan2(ny, nx) - center), math.cos(math.atan2(ny, nx) - center))
    residuals = [nx * (x - mx) + ny * (y - my) for x, y in pts]
    rms = math.sqrt(sum(v * v for v in residuals) / len(pts))
    along = [ny * x - nx * y for x, y in pts]
    span = max(along) - min(along)
    if abs(angle) > math.radians(12) or rms > 0.012 or span < 0.08:
        raise ValueError(
            f'{side}: unreliable wall angle={math.degrees(angle):.1f} rms={rms:.4f} span={span:.3f}'
        )
    return Wall(nx * mx + ny * my - support(nx, ny), angle, rms, span, len(pts), raw_count)


def swept_clearance(points, vx=0.0, vy=0.0, wz=0.0, duration=0.5):
    """Sample observed-point clearance during a short body motion; not a proof for unseen obstacles."""
    if not points:
        raise ValueError('No obstacle returns')
    best = float('inf')
    for i in range(6):
        dt = duration * i / 5
        c, s = math.cos(wz * dt), math.sin(wz * dt)
        for x, y, _ in points:
            px, py = x - vx * dt, y - vy * dt
            best = min(best, point_clearance(c * px + s * py, -s * px + c * py))
    return best


def side_distance(points, counts, side, heading_error):
    """Estimate side offset in a +/-30 degree window using the held odom direction.

    Require 80% within 20 mm of the median projection, 60% scan coverage,
    12 mm RMS and 8 cm longitudinal span. This is a distance estimator, not
    a new heading observation. All original returns remain available to guards.
    """
    if side not in ('left', 'right') or not math.isfinite(heading_error):
        raise ValueError('Invalid constrained side direction')
    center = DIRECTIONS[side]
    nx, ny = math.cos(center + heading_error), math.sin(center + heading_error)
    selected = [
        (x, y)
        for x, y, bearing in points
        if abs(math.atan2(math.sin(bearing - center), math.cos(bearing - center)))
        <= math.radians(30)
    ]
    expected = counts.get(side + '_wide', counts[side])
    if len(selected) < 8 or len(selected) < 0.6 * expected:
        raise ValueError(f'{side}: insufficient wide-window returns')
    offset = median(nx * x + ny * y for x, y in selected)
    inliers = [(x, y) for x, y in selected if abs(nx * x + ny * y - offset) <= 0.020]
    if len(inliers) * 5 < len(selected) * 4 or len(inliers) < 0.6 * expected:
        raise ValueError(f'{side}: constrained distance lacks 80 percent consensus')
    offset = median(nx * x + ny * y for x, y in inliers)
    rms = math.sqrt(sum((nx * x + ny * y - offset) ** 2 for x, y in inliers) / len(inliers))
    along = [ny * x - nx * y for x, y in inliers]
    span = max(along) - min(along)
    if rms > 0.012 or span < 0.08:
        raise ValueError(f'{side}: constrained distance rough or short')
    return Wall(offset - support(nx, ny), heading_error, rms, span, len(inliers), len(selected))


def front_distance(points, counts):
    """Fit the stopping wall in +/-12 degrees, with actual scheduled-ray coverage.

    Front inliers allow 18 mm residual, but TLS RMS still must be <=12 mm.
    Only the estimator input is narrowed. Travel/swept-obstacle guards retain all
    original scan points and their wider coverage checks. Near-wall short spans
    are rejected by the unchanged 8 cm minimum rather than guessed from one ray.
    """
    selected = [p for p in points if abs(p[2]) <= math.radians(12)]
    return fit_wall(selected, {'front': counts['front_narrow']}, 'front', tolerance=0.018)


def bounded_follow_velocity(points, planned, rotation):
    """Reduce only planned lateral correction; preserve forward speed and heading control.

    Keep 2.5 cm predictive margin where feasible, or the uncorrected command's
    clearance when smaller. Never choose a reduced correction below 2 cm. All
    scan points are retained; the caller must still check coverage and motion.
    """
    vx, vy, wz = planned
    c, s = math.cos(rotation), math.sin(rotation)

    def body(scale):
        return (c * vx - s * vy * scale, s * vx + c * vy * scale, wz)

    original = body(1.0)
    if vy == 0 or swept_clearance(points, *original) >= 0.025:
        return original, 1.0
    base_gap = swept_clearance(points, *body(0.0))
    if base_gap < 0.02:
        # Removing correction cannot establish a safe fallback; let the full guard decide.
        return original, 1.0
    goal = min(0.025, base_gap)
    if swept_clearance(points, *original) >= goal:
        return original, 1.0
    for scale in (0.75, 0.5, 0.25, 0.0):
        candidate = body(scale)
        if swept_clearance(points, *candidate) >= goal:
            return candidate, scale
    return original, 1.0


def transported_gaps(walls, previous, current):
    """Predict fixed-wall gaps after bounded odom motion, including body support rotation.

    This does not re-anchor a lost wall. Missing/time-misaligned odometry must be
    rejected by the caller; large motion cannot be excused as a recovery adjustment.
    """
    dx, dy = current.x - previous.x, current.y - previous.y
    turn = math.remainder(current.yaw - previous.yaw, 2 * math.pi)
    if math.hypot(dx, dy) > 0.08 or abs(turn) > 0.15:
        raise RuntimeError('WALL_REFERENCE: motion too large for continuity compensation')
    expected = {}
    for side, wall in walls.items():
        angle = DIRECTIONS[side] + wall.angle
        world_angle = previous.yaw + angle
        new_angle = angle - turn
        expected[side] = (
            wall.gap
            - math.cos(world_angle) * dx
            - math.sin(world_angle) * dy
            + support(math.cos(angle), math.sin(angle))
            - support(math.cos(new_angle), math.sin(new_angle))
        )
    return expected
