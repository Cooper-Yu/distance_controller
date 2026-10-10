"""Pure relative interpolation and holonomic feedback for supervised trials."""

import math
from action_plan import Pose, errors


def compile_path(origin, data):
    """All endpoints derive from preceding planned endpoints, including local offsets."""
    spacing = float(data['spacing_m'])
    if not math.isfinite(spacing) or not 0.01 <= spacing <= 0.10:
        raise ValueError('Spacing must be .01-.10 m')
    anchor = origin
    result = []
    for row in data['segments']:
        points = []
        if row['kind'] == 'turn':
            angle = math.radians(float(row['angle_deg']))
            if not math.isfinite(angle) or not 0 < abs(angle) <= math.pi:
                raise ValueError('Turn must be nonzero and within 180 degrees')
            anchor = Pose(anchor.x, anchor.y, anchor.yaw + angle)
            points = [anchor]
        elif row['kind'] == 'translate':
            for dx, dy in row['offsets']:
                if not all(math.isfinite(v) for v in (dx, dy)) or not 0 < math.hypot(dx, dy) <= 2:
                    raise ValueError('Invalid offset')
                c, s = math.cos(anchor.yaw), math.sin(anchor.yaw)
                end = Pose(anchor.x + c * dx - s * dy, anchor.y + s * dx + c * dy, anchor.yaw)
                count = math.ceil(math.hypot(dx, dy) / spacing)
                points.extend(
                    Pose(
                        anchor.x + (end.x - anchor.x) * i / count,
                        anchor.y + (end.y - anchor.y) * i / count,
                        anchor.yaw,
                    )
                    for i in range(1, count + 1)
                )
                anchor = end
        else:
            raise ValueError('Unknown segment kind')
        if not points or not row['name']:
            raise ValueError('Empty segment')
        result.append(dict(name=row['name'], kind=row['kind'], points=points))
    if not result:
        raise ValueError('Empty route')
    return result


def track(actual, segment, index, speed):
    """Advance intermediate targets continuously; convert odom errors into body velocities."""
    points = segment['points']
    while index < len(points) - 1 and errors(actual, points[index])[0] < 0.015:
        index += 1
    goal, end = points[index], points[-1]
    yaw_error = end.yaw - actual.yaw
    wz = max(-0.20, min(0.20, 1.5 * yaw_error)) if abs(yaw_error) > 0.01 else 0.0
    distance = errors(actual, end)[0]
    ready = distance < 0.01 and abs(yaw_error) < 0.01
    if segment['kind'] == 'turn':
        if distance > 0.04:
            raise RuntimeError('Turn center drift exceeded 4 cm')
        return index, (0.0, 0.0, wz), distance <= 0.03 and abs(yaw_error) < 0.01
    vx, vy = 2 * (goal.x - actual.x), 2 * (goal.y - actual.y)
    scale = min(
        1.0, speed / max(math.hypot(vx, vy), 1e-9), 0.8 * distance / max(math.hypot(vx, vy), 1e-9)
    )
    c, s = math.cos(actual.yaw), math.sin(actual.yaw)
    vx, vy = scale * (c * vx + s * vy), scale * (-s * vx + c * vy)
    if abs(yaw_error) > 0.08 or distance < 0.01:
        vx = vy = 0.0
    return index, (vx, vy, wz), ready


def limited_velocity(previous, requested, dt):
    """Bound vector linear acceleration and angular acceleration."""
    dt = max(0.0, min(dt, 0.1))
    dx, dy = requested[0] - previous[0], requested[1] - previous[1]
    scale = min(1.0, 0.12 * dt / max(math.hypot(dx, dy), 1e-9))
    dw = max(-0.4 * dt, min(0.4 * dt, requested[2] - previous[2]))
    return previous[0] + scale * dx, previous[1] + scale * dy, previous[2] + dw
