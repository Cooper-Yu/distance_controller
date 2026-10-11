"""Bounded P01-relative near-straight teaching path; no old odom coordinates."""

import json
import math
from pathlib import Path

from motion_model import HeadingPid, Motion, TRANSLATION_KP, local_delta


def interpolate(points, spacing=0.05):
    """Keep measured anchors exactly; subdivide each edge into at most spacing meters."""
    if not math.isfinite(spacing) or not 0.01 <= spacing <= 0.05:
        raise ValueError('Path spacing must be .01-.05 m')
    if not 2 <= len(points) <= 100:
        raise ValueError('Expected 2-100 path anchors')
    if any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in points):
        raise ValueError('Path anchors must be finite xy pairs')
    if math.hypot(*points[0]) > 1e-9:
        raise ValueError('Path must start at P01=(0,0)')
    result, length = [tuple(points[0])], 0.0
    for a, b in zip(points, points[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        distance = math.hypot(dx, dy)
        if dx <= 0 or abs(math.atan2(dy, dx)) > math.radians(10):
            raise ValueError('P02 trial supports forward near-straight paths only')
        length += distance
        if length > 2:
            raise ValueError('Path exceeds 2 m')
        count = math.ceil(distance / spacing)
        result.extend((a[0] + dx * i / count, a[1] + dy * i / count) for i in range(1, count))
        result.append(tuple(b))
    return tuple(result)


def load_p02():
    """Load the reviewed relative teaching data from the installed package."""
    from ament_index_python.packages import get_package_share_directory

    path = (
        Path(get_package_share_directory('distance_controller')) / 'config/task6_p01_p02_path.json'
    )
    data = json.loads(path.read_text())
    if data['schema'] != 1 or data['frame'] != 'P01' or data['heading'] != 'hold_p01':
        raise ValueError('Unsupported path reference or heading policy')
    points = interpolate([(p['x'], p['y']) for p in data['points']], data['max_spacing_m'])
    return Motion('path', x=points[-1][0], y=points[-1][1], points=points)


def path_ready(origin, actual, points):
    """Endpoint and heading acceptance use the fixed P01, not the last interpolation point."""
    p = local_delta(origin, actual)
    return (
        math.hypot(p['x_m'] - points[-1][0], p['y_m'] - points[-1][1]) < 0.005
        and abs(math.radians(p['yaw_deg'])) < 0.01
    )


class PathTracker:
    """Monotonic arc progress and speed-scaled lookahead; interpolation points are not stops."""

    def __init__(self, origin, points):
        self.origin, self.points, self.progress = origin, points, 0.0
        self.heading_pid = HeadingPid()
        self.arc = [0.0]
        for a, b in zip(points, points[1:]):
            self.arc.append(self.arc[-1] + math.dist(a, b))

    def project(self, position):
        best = (math.inf, 0.0)
        for i, (a, b) in enumerate(zip(self.points, self.points[1:])):
            length = self.arc[i + 1] - self.arc[i]
            ux, uy = (b[0] - a[0]) / length, (b[1] - a[1]) / length
            t = max(0.0, min(length, (position[0] - a[0]) * ux + (position[1] - a[1]) * uy))
            distance = math.hypot(position[0] - a[0] - ux * t, position[1] - a[1] - uy * t)
            best = min(best, (distance, self.arc[i] + t))
        if best[0] > 0.05:
            raise RuntimeError('Path deviation exceeds 5 cm; stopped, no automatic resume')
        self.progress = max(self.progress, best[1])
        return best[0]

    def ahead(self, lookahead=0.05):
        distance = min(self.arc[-1], self.progress + lookahead)
        for i in range(len(self.points) - 1):
            if distance <= self.arc[i + 1]:
                fraction = (distance - self.arc[i]) / (self.arc[i + 1] - self.arc[i])
                a, b = self.points[i : i + 2]
                return a[0] + fraction * (b[0] - a[0]), a[1] + fraction * (b[1] - a[1])
        return self.points[-1]

    def feedback(self, actual, speed, turn_speed, dt=0.05, yaw_rate=0.0):
        p = local_delta(self.origin, actual)
        self.project((p['x_m'], p['y_m']))
        tx, ty = self.ahead(max(0.05, speed / TRANSLATION_KP))
        vx, vy = TRANSLATION_KP * (tx - p['x_m']), TRANSLATION_KP * (ty - p['y_m'])
        scale = min(1.0, speed / max(math.hypot(vx, vy), 1e-12))
        yaw = math.radians(p['yaw_deg'])
        c, s = math.cos(yaw), math.sin(yaw)
        vx, vy = scale * (c * vx + s * vy), scale * (-s * vx + c * vy)
        if abs(yaw) > 0.08:
            vx = vy = 0.0
        wz = self.heading_pid.update(-yaw, yaw_rate, dt, turn_speed)
        if path_ready(self.origin, actual, self.points):
            self.heading_pid.reset()
        return (vx, vy, wz), path_ready(self.origin, actual, self.points)
