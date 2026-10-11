"""Pure command parsing, frozen-reference feedback and teaching-point geometry."""

from dataclasses import dataclass
import math
import shlex

from action_plan import Pose


@dataclass(frozen=True)
class Motion:
    """Relative translation in meters, or signed turn in radians."""

    kind: str
    x: float = 0.0
    y: float = 0.0
    angle: float = 0.0
    heading: bool = True
    lateral: bool = True
    points: tuple = ()


def number(text):
    value = float(text)
    if not math.isfinite(value):
        raise ValueError('Numbers must be finite')
    return value


def parse_motion(words):
    """Reject malformed options before any batch action starts."""
    if len(words) < 3:
        raise ValueError('Expected move DIRECTION METERS or turn left|right DEGREES')
    kind, direction = words[:2]
    if kind == 'turn':
        if len(words) != 3 or direction not in ('left', 'right'):
            raise ValueError('Expected turn left|right DEGREES')
        angle = number(words[2])
        if not 0 < angle <= 360:
            raise ValueError('Turn angle must be (0, 360] degrees')
        return Motion('turn', angle=math.radians(angle) * (1 if direction == 'left' else -1))
    vectors = {'forward': (1, 0), 'backward': (-1, 0), 'left': (0, 1), 'right': (0, -1)}
    if direction == 'xy' and len(words) >= 4:
        x, y, tail = number(words[2]), number(words[3]), words[4:]
    elif direction in vectors:
        distance = number(words[2])
        if distance <= 0:
            raise ValueError('Directional distance must be positive')
        x, y = (distance * v for v in vectors[direction])
        tail = words[3:]
    else:
        raise ValueError('Unknown move direction or missing xy coordinate')
    if not 0 < math.hypot(x, y) <= 2:
        raise ValueError('Translation length must be (0, 2] meters')
    options = {}
    while tail:
        if (
            len(tail) < 2
            or tail[0] not in ('--heading', '--lateral')
            or tail[1] not in ('on', 'off')
        ):
            raise ValueError('Expected --heading on|off and/or --lateral on|off')
        key = tail[0][2:]
        if key in options:
            raise ValueError('Duplicate option: ' + key)
        options[key] = tail[1] == 'on'
        tail = tail[2:]
    return Motion('move', x=x, y=y, **options)


def parse_batch(line):
    """Semicolons mean sequential stopped actions, never shell execution."""
    result = []
    for part in line.split(';'):
        words = shlex.split(part)
        if not words:
            raise ValueError('Empty command in batch')
        if words[0] in ('move', 'turn'):
            result.append(parse_motion(words))
            continue
        arity = {
            'path': 2,
            'path_plan': 1,
            'record': 2,
            'delta': 3,
            'measure': 1,
            'status': 1,
            'points': 1,
            'stop': 1,
            'quit': 1,
            'help': 1,
        }
        if words[0] not in arity or len(words) != arity[words[0]]:
            raise ValueError('Unknown command or wrong arguments; type help')
        if words[0] == 'path' and words[1] != 'P02':
            raise ValueError('Only path P02 is currently supported')
        result.append(tuple(words))
    if len(result) > 32:
        raise ValueError('At most 32 commands per batch')
    return result


def local_delta(start, end):
    """Express endpoint displacement in start heading axes (forward, left)."""
    dx, dy = end.x - start.x, end.y - start.y
    c, s = math.cos(start.yaw), math.sin(start.yaw)
    yaw = math.atan2(math.sin(end.yaw - start.yaw), math.cos(end.yaw - start.yaw))
    return dict(
        x_m=c * dx + s * dy,
        y_m=-s * dx + c * dy,
        distance_m=math.hypot(dx, dy),
        yaw_deg=math.degrees(yaw),
    )


def target_pose(start, motion):
    c, s = math.cos(start.yaw), math.sin(start.yaw)
    return Pose(
        start.x + c * motion.x - s * motion.y,
        start.y + s * motion.x + c * motion.y,
        start.yaw + motion.angle,
    )


TRANSLATION_KP = 1.5


class HeadingPid:
    """Task5 angular gains, signed measured-rate damping and conditional integration."""

    def __init__(self):
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def update(self, error, rate, dt, limit):
        if not all(math.isfinite(v) for v in (error, rate, dt, limit)) or not 0 < dt <= 0.2:
            self.reset()
            raise RuntimeError('Invalid PID timing or feedback')
        candidate = max(-0.5, min(0.5, self.integral + error * dt))
        requested = 1.8 * error + 0.03 * candidate - 0.35 * rate
        if abs(requested) <= limit or error * requested < 0:
            self.integral = candidate
        return max(-limit, min(limit, 1.8 * error + 0.03 * self.integral - 0.35 * rate))


def feedback(start, actual, motion, speed, turn_speed, pid=None, dt=0.05, yaw_rate=0.0):
    """Track a line in frozen start axes; lateral-off checks along-track arrival only.

    Heading-off disables angular correction, not the transformation to the current
    body frame. Turns retain signed continuous yaw, including turns over 180 deg.
    """
    yaw_error = start.yaw + motion.angle - actual.yaw
    pid = pid if pid is not None else HeadingPid()
    if motion.kind == 'turn' or motion.heading:
        wz = pid.update(yaw_error, yaw_rate, dt, turn_speed)
    else:
        pid.reset()
        wz = 0.0
    if motion.kind == 'turn':
        if math.hypot(actual.x - start.x, actual.y - start.y) > 0.04:
            raise RuntimeError('Turn center drift exceeds 4 cm')
        return (0.0, 0.0, wz), abs(yaw_error) < 0.01
    length = math.hypot(motion.x, motion.y)
    ux, uy = motion.x / length, motion.y / length
    delta = local_delta(start, actual)
    along = delta['x_m'] * ux + delta['y_m'] * uy
    cross = -delta['x_m'] * uy + delta['y_m'] * ux
    forward = TRANSLATION_KP * (length - along)
    sideways = -TRANSLATION_KP * cross if motion.lateral else 0.0
    vx, vy = ux * forward - uy * sideways, uy * forward + ux * sideways
    scale = min(1.0, speed / max(math.hypot(vx, vy), 1e-12))
    angle = start.yaw - actual.yaw
    c, s = math.cos(angle), math.sin(angle)
    vx, vy = scale * (c * vx - s * vy), scale * (s * vx + c * vy)
    ready = abs(length - along) < 0.005 and (not motion.lateral or abs(cross) < 0.005)
    if motion.heading and abs(yaw_error) > 0.08:
        vx = vy = 0.0
    ready = ready and (not motion.heading or abs(yaw_error) < 0.01)
    return (vx, vy, wz if motion.heading else 0.0), ready
