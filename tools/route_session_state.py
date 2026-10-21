"""Live-session single-action history; no deserialization or odom re-adoption API."""

from dataclasses import asdict, dataclass
import hashlib
import json
import math
import uuid

from motion_model import Motion, local_delta, target_pose


def checked_pose(pose):
    if not all(math.isfinite(v) for v in (pose.x, pose.y, pose.yaw)):
        raise ValueError('Nonfinite session pose')
    return pose


def near(actual, expected, distance=0.005, angle=0.01):
    checked_pose(actual)
    checked_pose(expected)
    return (
        math.hypot(actual.x - expected.x, actual.y - expected.y) <= distance + 1e-9
        and abs(math.remainder(actual.yaw - expected.yaw, 2 * math.pi)) <= angle + 1e-9
    )


def xy(origin, pose):
    delta = local_delta(origin, checked_pose(pose))
    return delta['x_m'], delta['y_m']


def subdivide(anchors):
    """Keep every distinct anchor, including both actual stops, with edges <= 5 cm."""
    points = [anchors[0]]
    for target in anchors[1:]:
        start = points[-1]
        length = math.dist(start, target)
        if length <= 1e-9:
            continue
        count = math.ceil(length / 0.05)
        points.extend(
            tuple(a + (b - a) * i / count for a, b in zip(start, target)) for i in range(1, count)
        )
        points.append(target)
    if len(points) < 2:
        raise RuntimeError('Return has no translation')
    return tuple(points)


@dataclass(frozen=True)
class CompletedAction:
    index: int
    name: str
    motion: Motion
    start: object
    end: object


@dataclass(frozen=True)
class ReturnPlan:
    token: str
    session_id: str
    route_hash: str
    index: int
    name: str
    source: object
    target: object
    motion: Motion


class LiveRouteState:
    """Retain one immutable P01 frame and require an explicit stop after every action.

    Call begin/finish around the actual controller and fail on every controller exception.
    Faults latch permanently in this object; motion recovery requires a separate reviewed design.
    This object intentionally cannot import TeachingBook JSON or an old checkpoint.
    """

    def __init__(self, origin, actions, session_id=None):
        self.origin = checked_pose(origin)
        self.actions = tuple(actions)
        self.session_id = session_id or str(uuid.uuid4())
        raw = json.dumps([asdict(a) for a in actions], sort_keys=True, allow_nan=False)
        self.route_hash = hashlib.sha256(raw.encode()).hexdigest()
        self.history = []
        self.last_stop = origin
        self.pending = None
        self.preview = None
        self.failure = None

    @property
    def cursor(self):
        return len(self.history)

    def check_waiting(self, actual):
        if self.failure or self.pending:
            raise RuntimeError('Session fault or partial action; no automatic recovery')
        if not near(actual, self.last_stop):
            self.fail('Stopped pose changed; live-session continuity refused')
            raise RuntimeError(self.failure)

    def begin_forward(self, motion, actual):
        self.check_waiting(actual)
        if self.cursor >= len(self.actions):
            raise RuntimeError('Route already complete')
        action = self.actions[self.cursor]
        if motion.kind != action.kind:
            raise ValueError('Action kind mismatch')
        # The route adapter may reconnect only the first edge after a completed turn.
        if motion.kind == 'path':
            if not motion.points or math.dist(motion.points[-1], action.points[-1]) > 1e-9:
                raise ValueError('Fixed route endpoint changed')
            if abs(math.remainder(motion.angle - action.heading_rad, 2 * math.pi)) > 1e-9:
                raise ValueError('Fixed route heading changed')
        self.preview = None
        self.pending = ('forward', action, motion, actual)

    def finish_forward(self, actual):
        if self.failure or not self.pending or self.pending[0] != 'forward':
            raise RuntimeError('No active forward action')
        _, action, motion, start = self.pending
        target = target_pose(self.origin if motion.kind == 'path' else start, motion)
        limit = 0.04 if motion.kind == 'turn' else 0.005
        if not near(actual, target, distance=limit):
            self.fail('Forward endpoint failed acceptance')
            raise RuntimeError(self.failure)
        self.history.append(CompletedAction(self.cursor, action.name, motion, start, actual))
        self.last_stop, self.pending = actual, None

    def preview_back(self, actual):
        self.check_waiting(actual)
        if not self.history:
            raise RuntimeError('Already at original P01')
        record = self.history[-1]
        if record.motion.kind != 'path':
            raise RuntimeError('Return across a turn is unsupported; inspect and stop')
        heading = record.start.yaw - self.origin.yaw
        if abs(math.remainder(actual.yaw - record.start.yaw, 2 * math.pi)) > math.radians(1):
            raise RuntimeError('Return needs a turn; refused')
        points = subdivide(
            (
                xy(self.origin, actual),
                *reversed(record.motion.points),
                xy(self.origin, record.start),
            )
        )
        motion = Motion('path', x=points[-1][0], y=points[-1][1], angle=heading, points=points)
        self.preview = ReturnPlan(
            str(uuid.uuid4()),
            self.session_id,
            self.route_hash,
            record.index,
            record.name,
            actual,
            record.start,
            motion,
        )
        return self.preview

    def begin_back(self, token, actual):
        self.check_waiting(actual)
        plan = self.preview
        if plan is None or token != plan.token or plan.index != self.cursor - 1:
            raise RuntimeError('Missing or stale return preview')
        # A preview is executable exactly once and only at its inspected stopped position.
        if not near(actual, plan.source, distance=0.002, angle=0.005):
            raise RuntimeError('Pose changed since return preview')
        self.pending, self.preview = ('back', plan), None
        return plan.motion

    def finish_back(self, actual):
        if self.failure or not self.pending or self.pending[0] != 'back':
            raise RuntimeError('No active return')
        plan = self.pending[1]
        if not near(actual, plan.target):
            self.fail('Return endpoint failed acceptance')
            raise RuntimeError(self.failure)
        self.history.pop()
        self.last_stop, self.pending = actual, None
        return plan.index

    def fail(self, reason):
        """Preserve the partial record and cursor; there is no clear-fault operation."""
        self.failure = str(reason) or 'Motion failed'
        self.preview = None
