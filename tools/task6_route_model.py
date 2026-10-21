"""Validate the complete scene-2 manifest before ROS ownership or preparation."""

from dataclasses import dataclass, replace
import math

from motion_model import Motion, local_delta


@dataclass(frozen=True)
class RouteAction:
    name: str
    kind: str
    heading_rad: float
    points: tuple = ()

    def path_motion(self):
        return Motion(
            'path',
            x=self.points[-1][0],
            y=self.points[-1][1],
            angle=self.heading_rad,
            points=self.points,
        )


def finite(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('Route coordinates and headings must be finite numbers')
    return float(value)


def path_points(raw):
    if not isinstance(raw, list) or not 2 <= len(raw) <= 1000:
        raise ValueError('Each path requires 2-1000 points')
    points = []
    for point in raw:
        if not isinstance(point, list) or len(point) != 2:
            raise ValueError('Path points must be xy pairs')
        points.append(tuple(finite(v) for v in point))
    lengths = [math.dist(a, b) for a, b in zip(points, points[1:])]
    if any(not 1e-9 < length <= 0.05 + 1e-9 for length in lengths):
        raise ValueError('Path edges must be nonzero and at most 5 cm')
    if sum(lengths) > 2 + 1e-9:
        raise ValueError('Each path must be at most 2 m')
    return tuple(points)


def validate_manifest(data):
    """Return immutable actions without generating, deleting or reordering route points."""
    if not isinstance(data, dict) or set(data) != {'schema', 'scene', 'frame', 'actions'}:
        raise ValueError('Expected schema, scene, frame and actions only')
    if type(data['schema']) is not int or data['schema'] != 1:
        raise ValueError('Expected schema 1')
    if type(data['scene']) is not int or data['scene'] != 2 or data['frame'] != 'P01':
        raise ValueError('Expected scene 2 in P01 frame')
    if not isinstance(data['actions'], list) or not 1 <= len(data['actions']) <= 100:
        raise ValueError('Expected 1-100 actions')
    actions, names, endpoint, heading = [], {'P01'}, (0.0, 0.0), 0.0
    for raw in data['actions']:
        action = validate_action(raw)
        if action.name in names:
            raise ValueError('Action names must be unique and cannot be P01')
        names.add(action.name)
        if action.kind == 'path':
            if math.dist(action.points[0], endpoint) > 1e-9:
                raise ValueError('Paths must form a continuous P01 route')
            if abs(math.remainder(action.heading_rad - heading, 2 * math.pi)) > 1e-9:
                raise ValueError('Path heading requires an explicit preceding turn')
            endpoint = action.points[-1]
        elif not actions:
            raise ValueError('The first action must be a path from P01')
        heading = action.heading_rad
        actions.append(action)
    return tuple(actions)


def validate_action(raw):
    if not isinstance(raw, dict) or raw.get('kind') not in ('path', 'turn'):
        raise ValueError('Each action must be path or turn')
    expected = {'name', 'kind', 'heading_rad'} | ({'points'} if raw['kind'] == 'path' else set())
    if set(raw) != expected:
        raise ValueError('Unexpected or missing action fields')
    name = raw['name']
    if not isinstance(name, str) or not name or len(name) > 80 or not name.isascii():
        raise ValueError('Expected a nonempty ASCII action name of at most 80 characters')
    if any(not (c.isalnum() or c == '_') for c in name):
        raise ValueError('Action names may contain letters, numbers and underscores')
    heading = finite(raw['heading_rad'])
    if abs(heading) > 20 * math.pi:
        raise ValueError('Heading outside the bounded route range')
    points = path_points(raw['points']) if raw['kind'] == 'path' else ()
    return RouteAction(name, raw['kind'], heading, points)


def reconnect_route(motion, origin, actual):
    """Apply the existing 2 cm / 1 deg gate, preserving every subsequent supplied point."""
    delta = local_delta(origin, actual)
    position = (delta['x_m'], delta['y_m'])
    error = math.remainder(math.radians(delta['yaw_deg']) - motion.angle, 2 * math.pi)
    if math.dist(position, motion.points[0]) > 0.02 or abs(error) > math.radians(1):
        raise RuntimeError('Post-turn reconnect requires start within 2 cm / 1 deg')
    next_point = motion.points[1]
    distance = math.dist(position, next_point)
    if distance <= 1e-9:
        raise RuntimeError('Post-turn reconnect would duplicate a path point')
    count = math.ceil(distance / 0.05)
    # Subdivide only the new first edge; keep every supplied later point exactly.
    prefix = tuple(
        (
            position[0] + (next_point[0] - position[0]) * i / count,
            position[1] + (next_point[1] - position[1]) * i / count,
        )
        for i in range(count)
    )
    return replace(motion, points=(*prefix, *motion.points[1:]))
