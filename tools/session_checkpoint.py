"""Atomic stopped-session checkpoints; restoration never publishes a motion command."""

from dataclasses import asdict
import json
import math
import os
from pathlib import Path
from action_plan import Pose, Session, Step, errors


def read_checkpoint(path):
    """Reject incomplete/unsupported files and nonfinite JSON before creating a backend."""

    def invalid(value):
        raise ValueError(f'Nonfinite checkpoint value: {value}')

    if Path(path).suffix == '.jsonl':
        from session_audit import import_audit

        data = import_audit(path)
    else:
        data = json.loads(Path(path).read_text(), parse_constant=invalid)
    if data.get('version') != 1 or data.get('usable') is not True:
        raise ValueError('Checkpoint is not a verified stopped snapshot')
    steps = [Step(**row).validate() for row in data['steps']]
    if not steps or len({s.name for s in steps}) != len(steps):
        raise ValueError('Invalid checkpoint route')
    cursor = data['cursor']
    if type(cursor) is not int or not 0 <= cursor <= len(steps):
        raise ValueError('Invalid checkpoint cursor')
    if [r['index'] for r in data['active']] != list(range(cursor)):
        raise ValueError('Invalid checkpoint history')
    validate_records(data, steps)
    partial = data['partial']
    if (
        partial
        and partial['index'] != cursor
        and not (partial.get('return_started') and partial['index'] == cursor - 1)
    ):
        raise ValueError('Invalid partial index')
    finite_pose(data['origin'])
    finite_pose(data['last_actual'])
    if not isinstance(data['frame'], str) or not data['frame'] or type(data['stamp']) is not int:
        raise ValueError('Missing odom identity')
    if any(s.wall for s in steps) and not all(s.wall for s in steps):
        raise ValueError('Mixed route unsupported')
    if not math.isfinite(data['wall_speed']) or not 0.01 <= data['wall_speed'] <= 0.08:
        raise ValueError('Invalid saved speed')
    if any(s.wall for s in steps) and not 0.05 <= data['reference'] <= 0.30:
        raise ValueError('Invalid wall reference')
    return data


def validate_records(data, steps):
    """Keep saved history consistent with the editable route and finite geometry."""
    for record in data['active'] + ([data['partial']] if data['partial'] else []):
        index = record['index']
        if not 0 <= index < len(steps) or Step(**record['step']) != steps[index]:
            raise ValueError('History does not match route')
        for key in ('start', 'target', 'end'):
            if key in record:
                finite_pose(record[key])
        if record.get('turn_adjustment'):
            validate_adjustment(record)
        if not math.isfinite(record.get('path_m', 0)) or record.get('path_m', 0) < 0:
            raise ValueError('Invalid travel budget')


def validate_adjustment(record):
    """Reject malformed saved micro-adjustments before any restored motion."""
    state = record['turn_adjustment']
    origin, target = finite_pose(state['origin']), finite_pose(state['target'])
    distance = state['distance']
    if (
        record['step']['kind'] != 'turn'
        or not math.isfinite(distance)
        or not 0.005 <= distance <= 0.03
        or type(state['done']) is not bool
        or not math.isfinite(state['path_m'])
        or state['path_m'] < 0
        or errors(origin, target)[0] > 0.031
        or abs(target.yaw - origin.yaw) > 1e-6
    ):
        raise ValueError('Invalid turn adjustment checkpoint')
    expected = Pose(
        origin.x + math.sin(origin.yaw) * distance,
        origin.y - math.cos(origin.yaw) * distance,
        origin.yaw,
    )
    if errors(expected, target)[0] > 1e-6:
        raise ValueError('Turn adjustment target is not its original right offset')
    if state.get('last_sample'):
        finite_pose(state['last_sample'])


def finite_pose(row):
    pose = Pose(**row)
    if not all(math.isfinite(v) for v in (pose.x, pose.y, pose.yaw)):
        raise ValueError('Nonfinite checkpoint pose')
    return pose


def write_checkpoint(path, session, usable=True):
    """Replace one JSON atomically; a busy marker prevents resuming a crashed motion."""
    b = session.backend
    data = {
        'version': 1,
        'usable': usable and session.last_actual is not None,
        'origin': asdict(session.origin),
        'steps': [asdict(s) for s in session.steps],
        'cursor': session.cursor,
        'revision': session.revision,
        'active': session.active,
        'partial': session.partial,
        'last_actual': asdict(session.last_actual) if session.last_actual else None,
        'frame': b.frame,
        'stamp': b.stamp,
        'reference': getattr(b, 'reference', None),
        'wall_speed': getattr(b, 'max_speed', 0.06),
    }
    path = Path(path)
    temp = path.with_name(path.name + '.tmp')
    with temp.open('w') as stream:
        stream.write(json.dumps(data, allow_nan=False, indent=2) + '\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temp, path)


def restore_checkpoint(data, backend, emit):
    """Check pose/frame/time, restore yaw turns, and enter WAITING without preparation.

    These checks cannot prove epoch continuity after an undetectable reset; the
    CLI also requires the operator to confirm no odom reset or relocation.
    """
    actual = backend.pose()
    saved = finite_pose(data['last_actual'])
    distance, angle = errors(actual, saved)
    if not data.get('legacy_audit') and (
        backend.frame != data['frame'] or backend.stamp < data['stamp']
    ):
        raise RuntimeError('Odom frame/time changed; recovery refused')
    if distance > 0.03 or abs(angle) > 0.05:
        raise RuntimeError('Checkpoint pose mismatch; recovery refused')
    shift = round((saved.yaw - actual.yaw) / (2 * math.pi)) * 2 * math.pi
    backend.continuous_yaw += shift
    backend.latest = (Pose(actual.x, actual.y, actual.yaw + shift), *backend.latest[1:])
    steps = [Step(**row) for row in data['steps']]
    if any(s.wall for s in steps):
        from wall_session import WallSession

        cls = WallSession
        backend.reference = data['reference']
    else:
        cls = Session
    session = cls(finite_pose(data['origin']), steps, backend, emit)
    session.cursor, session.revision = data['cursor'], data['revision']
    session.active, session.partial = data['active'], data['partial']
    session.last_actual = saved
    emit('checkpoint_restored', {'cursor': session.cursor, 'partial': session.partial})
    return session
