"""Validate a completed formal audit and reverse actions in its unchanged P01 frame."""

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

from action_plan import Pose
from motion_model import Motion, target_pose
from route_session_state import checked_pose, near, subdivide, xy
from task6_route_model import validate_manifest


def pose(raw):
    if not isinstance(raw, dict) or set(raw) != {'x', 'y', 'yaw'}:
        raise ValueError('Expected explicit finite xyz/yaw pose')
    return checked_pose(Pose(**raw))


def one(rows, event):
    values = [row for row in rows if row.get('event') == event]
    if len(values) != 1:
        raise ValueError(f'Expected exactly one {event}')
    return values[0]


@dataclass(frozen=True)
class ReturnAction:
    name: str
    kind: str
    start: Pose
    end: Pose
    points: tuple


@dataclass(frozen=True)
class ReturnSource:
    session_id: str
    origin: Pose
    last_stop: Pose
    stamp_ns: int
    source_hash: str
    actions: tuple


def load_source(path):
    path = Path(path)
    raw, audit = path.read_bytes(), path.with_suffix('.jsonl').read_bytes()
    source = validate_source(json.loads(raw), [json.loads(line) for line in audit.splitlines()])
    return ReturnSource(
        *source,
        hashlib.sha256(raw + b'\n' + audit).hexdigest(),
        tuple(source_actions(json.loads(raw), [json.loads(line) for line in audit.splitlines()])),
    )


def validate_source(book, rows):
    if book.get('schema') != 1 or book.get('frame') != 'odom':
        raise ValueError('Only formal odom-frame TeachingBook schema 1 is accepted')
    if not isinstance(book.get('session_id'), str) or not book['session_id']:
        raise ValueError('Missing original session identity')
    if not rows or rows[-1].get('event') != 'route_completed':
        raise ValueError('Source route lacks final completed boundary')
    if any(row.get('event') in ('route_failed', 'stopped', 'incomplete') for row in rows):
        raise ValueError('Failed or partial source cannot be returned automatically')
    origin = pose(book['origin'])
    points = book['points']
    p01 = one([r for r in rows if r.get('name') == 'P01'], 'record')['point']
    if p01 != points['P01'] or pose(p01['pose']) != origin:
        raise ValueError('P01 source identity/pose mismatch')
    actions = tuple(source_actions(book, rows))
    done = one(rows, 'route_completed')
    if not actions or done.get('exit_code') != 0 or done['destination'] != actions[-1].name:
        raise ValueError('Destination completion mismatch')
    last = points[actions[-1].name]
    stamps = [points[name]['odom_stamp_ns'] for name in ['P01', *(a.name for a in actions)]]
    if any(type(s) is not int or s <= 0 for s in stamps) or any(
        b < a for a, b in zip(stamps, stamps[1:])
    ):
        raise ValueError('Invalid source odom chronology')
    return book['session_id'], origin, pose(last['pose']), stamps[-1]


def source_actions(book, rows):
    started = one(rows, 'route_started')
    if started.get('scene') != 2 or started.get('frame') != 'P01':
        raise ValueError('Expected formal scene2 source')
    raw_actions = []
    for raw in started['actions']:
        item = dict(raw)
        if item.get('kind') == 'turn' and item.get('points') == []:
            item.pop('points')
        raw_actions.append(item)
    actions = validate_manifest(dict(schema=1, scene=2, frame='P01', actions=raw_actions))
    names = ['P01', *(a.name for a in actions)]
    if set(book['points']) != set(names):
        raise ValueError('Unexpected/missing point records')
    boundaries = [i for i, row in enumerate(rows) if row.get('event') == 'route_action_started']
    if len(boundaries) != len(actions):
        raise ValueError('Incomplete action count')
    for index, action in enumerate(actions):
        start_index = boundaries[index]
        end_index = boundaries[index + 1] if index + 1 < len(boundaries) else len(rows)
        block = rows[start_index:end_index]
        if block[0].get('index') != index or block[0].get('name') != action.name:
            raise ValueError('Action order mismatch')
        previous = pose(book['points'][names[index]]['pose'])
        yield validate_completed(action, block, book, previous)


def validate_completed(action, block, book, previous):
    started, completed, recorded = (one(block, e) for e in ('started', 'completed', 'record'))
    checks = [r for r in block if r.get('event') == 'final_check']
    if not checks or checks[-1].get('ready') is not True:
        raise ValueError('Action has no accepted stopped pose')
    if (
        not block.index(started)
        < block.index(checks[-1])
        < block.index(completed)
        < block.index(recorded)
    ):
        raise ValueError('Action evidence is not in execution order')
    if recorded.get('name') != action.name or recorded['point'] != book['points'][action.name]:
        raise ValueError('JSON/JSONL point mismatch')
    raw = dict(started['motion'])
    raw['points'] = tuple(tuple(p) for p in raw['points'])
    motion = Motion(**raw)
    if motion.kind != action.kind or not motion.heading or not motion.lateral:
        raise ValueError('Unsupported source motion')
    end = pose(recorded['point']['pose'])
    start = pose(started['start']) if action.kind == 'turn' else previous
    anchor = pose(book['origin']) if action.kind == 'path' else start
    expected = target_pose(anchor, motion)
    # Turns calculate their relative angle before Console samples the actual start.
    # Audit that recorded start exactly below; retain the original heading acceptance.
    heading_limit = 0.01 if action.kind == 'turn' else 1e-8
    if action.kind == 'turn' and (
        motion.x != 0 or motion.y != 0 or abs(motion.angle) > math.pi + 1e-8
    ):
        raise ValueError('Unsupported source turn geometry')
    if (
        abs(
            math.remainder(
                expected.yaw - pose(book['origin']).yaw - action.heading_rad, 2 * math.pi
            )
        )
        > heading_limit
    ):
        raise ValueError('Recorded action heading differs from fixed route')
    if action.kind == 'path' and (
        not near(pose(started['start']), anchor, 1e-8, 1e-8)
        or math.dist((motion.x, motion.y), action.points[-1]) > 1e-9
    ):
        raise ValueError('Recorded path origin or fixed endpoint changed')
    if not near(pose(started['target']), expected, distance=1e-8, angle=1e-8):
        raise ValueError('Motion target mismatch')
    if not near(pose(completed['actual']), end) or not near(start, previous, 0.02, math.radians(1)):
        raise ValueError('Stopped action boundaries mismatch')
    limit = 0.04 if action.kind == 'turn' else 0.005
    if not near(pose(completed['actual']), expected, limit, 0.01):
        raise ValueError('Source endpoint outside original acceptance')
    if action.kind == 'path':
        suffix = tuple(action.points[1:])
        if (
            motion.points[-len(suffix) :] != suffix
            or abs(math.remainder(motion.angle - action.heading_rad, 2 * math.pi)) > 1e-9
        ):
            raise ValueError('Executed path changed fixed route bends/heading')
    return ReturnAction(action.name, action.kind, start, end, motion.points)


class ReturnSequence:
    """Reverse completed real actions, with no automatic advance or failure recovery."""

    def __init__(self, source):
        self.source, self.cursor = source, len(source.actions)
        self.last_stop, self.pending, self.failure = source.last_stop, None, None

    def begin(self, actual):
        if self.pending or self.failure:
            raise RuntimeError('Partial/failed return is locked; cannot continue')
        if self.cursor == 0:
            raise ValueError('Already returned to original P01')
        action = self.source.actions[self.cursor - 1]
        if not near(actual, self.last_stop, 0.02, math.radians(1)) or not near(
            actual, action.end, 0.02, math.radians(1)
        ):
            self.fail('Return start outside 2 cm / 1 deg; no rebase')
            raise RuntimeError(self.failure)
        if action.kind == 'turn':
            motion = Motion('turn', angle=action.start.yaw - actual.yaw)
        else:
            points = subdivide(
                (
                    xy(self.source.origin, actual),
                    *reversed(action.points),
                    xy(self.source.origin, action.start),
                )
            )
            motion = Motion(
                'path',
                x=points[-1][0],
                y=points[-1][1],
                angle=action.start.yaw - self.source.origin.yaw,
                points=points,
            )
        self.pending = action, actual, motion
        return motion

    def finish(self, actual):
        if self.failure or not self.pending:
            raise RuntimeError('No active return')
        action, start, _motion = self.pending
        if action.kind == 'turn' and math.hypot(actual.x - start.x, actual.y - start.y) > 0.04:
            self.fail('Return turn center drift exceeds 4 cm')
        limit = 0.02 if action.kind == 'turn' else 0.005
        if not near(actual, action.start, limit, 0.01):
            self.fail('Return endpoint outside acceptance; cannot continue')
        if self.failure:
            raise RuntimeError(self.failure)
        self.cursor -= 1
        self.last_stop, self.pending = actual, None

    def fail(self, reason):
        self.failure = str(reason) or 'Return failed'
