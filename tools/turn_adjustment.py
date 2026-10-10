"""Explicit stopped-turn micro-adjustments with fixed, checkpointed odom targets."""

from dataclasses import asdict
import math
import time
from action_plan import Pose, errors
from wall_geometry import swept_clearance
from wall_policy import clamp


def adjust_session(session, distance, escape=False):
    """Move right once at a fixed target; explicit escape also permits a partial turn."""
    record = session.partial
    if (
        record is None
        or record['index'] != session.cursor
        or record['step']['kind'] != 'turn'
        or record.get('return_started')
        or record.get('resume_unavailable')
    ):
        raise ValueError('adjust_turn requires an incomplete outward turn')
    if not math.isfinite(distance) or not 0.005 <= distance <= 0.03:
        raise ValueError('Right adjustment must be within [.005,.03] m')
    if escape and abs(distance - 0.03) > 1e-9:
        raise ValueError('escape_turn requires exactly 0.03 m right')
    actual = session.check_location()
    if not escape and abs(errors(actual, Pose(**record['start']))[1]) > 0.03:
        raise ValueError('Turn already changed heading; adjustment refused')
    key = 'turn_escape' if escape else 'turn_adjustment'
    if escape and record.get('turn_adjustment') and not record['turn_adjustment']['done']:
        raise ValueError('Pending pre-turn adjustment must be resolved first')
    state = record.get(key)
    if state is None:
        target = Pose(
            actual.x + math.sin(actual.yaw) * distance,
            actual.y - math.cos(actual.yaw) * distance,
            actual.yaw,
        )
        state = {
            'origin': asdict(actual),
            'target': asdict(target),
            'distance': distance,
            'path_m': 0.0,
            'done': False,
        }
        if escape:
            state.update(escape=True, cleared=False, final_yaw=record['target']['yaw'])
        record[key] = state
    if abs(state['distance'] - distance) > 1e-9:
        raise ValueError('Existing adjustment target is fixed; repeat the same distance')
    session.emit(key + '_started', state)
    try:
        if not state['done']:
            run_adjustment(session.backend, state)
        session.last_actual = session.backend.pose()
        record['target']['x'] = session.last_actual.x
        record['target']['y'] = session.last_actual.y
        session.emit(key + '_completed', {**state, 'actual': asdict(session.last_actual)})
        session.backend.check_turn(Pose(**record['target']))
        print('TURN_CHECK passed on fresh scan; WAITING, use resume to turn', flush=True)
        return session.last_actual
    except (Exception, KeyboardInterrupt):
        try:
            session.last_actual = session.backend.pose()
        except (Exception, KeyboardInterrupt):
            session.last_actual = None
        session.emit(
            key + '_stopped',
            {**state, 'last_actual': asdict(session.last_actual) if session.last_actual else None},
        )
        raise


def adjustment_command(actual, target):
    """Hold heading and close small XY error; cap translation at 1 cm/s."""
    dx, dy = target.x - actual.x, target.y - actual.y
    c, s = math.cos(actual.yaw), math.sin(actual.yaw)
    vx, vy = 0.7 * (c * dx + s * dy), 0.7 * (-s * dx + c * dy)
    scale = min(1.0, 0.01 / max(math.hypot(vx, vy), 1e-9))
    heading = errors(actual, target)[1]
    ready = math.hypot(dx, dy) <= 0.003 and abs(heading) <= 0.01
    if math.hypot(dx, dy) <= 0.003 or abs(heading) > 0.08:
        vx = vy = 0.0
    wz = 0.0 if abs(heading) <= 0.01 else clamp(1.2 * heading, 0.08)
    return (vx * scale, vy * scale, wz), ready


def guarded_adjustment(runner, state):
    """Run a <=20 s / 6 cm correction with original full-point command/measured guards."""
    target = Pose(**state['target'])
    last = runner.pose()
    deadline = time.monotonic() + 20
    hold, stamp, count = None, None, 0
    while time.monotonic() < deadline:
        runner.pump()
        actual = runner.moving_pose()
        state['path_m'] += errors(actual, last)[0]
        last = actual
        state['last_sample'] = asdict(actual)
        if state['path_m'] > 0.06 or errors(actual, Pose(**state['origin']))[0] > 0.06:
            raise RuntimeError('TURN_ADJUSTMENT_LIMIT: exceeded 6 cm')
        velocity, ready = adjustment_command(actual, target)
        if state.get('escape'):
            from turn_escape import escape_command, guard_escape

            velocity, ready = escape_command(actual, state)
            guard_escape(runner, state, velocity)
        else:
            guard_normal(runner, velocity)
        runner.send(velocity)
        if ready and runner.latest[2] < 0.01 and runner.latest[3] < 0.02:
            hold = time.monotonic() if hold is None else hold
            if runner.cached_stamp != stamp:
                stamp, count = runner.cached_stamp, count + 1
            if count >= 3 and time.monotonic() - hold >= 0.5:
                return
        else:
            hold, stamp, count = None, None, 0
    raise RuntimeError('TURN_ADJUSTMENT_TIMEOUT')


def guard_normal(runner, velocity):
    """Unchanged ordinary adjustment guard; no exception for existing close points."""
    runner.guard_translation(velocity, 'right' if velocity[1] <= 0 else 'left')
    if runner.twist_frame != 'base_link':
        raise RuntimeError('Adjustment requires body-frame velocity')
    points, _ = runner.geometry()
    gap = swept_clearance(points, *runner.body_velocity)
    if gap < 0.02:
        runner.reject_translation(points, runner.body_velocity, 'right', gap, 'measured')


def run_adjustment(runner, state):
    """Own the publisher and guarantee zero commands before returning or raising."""
    runner.exclusive()
    runner.pose()
    runner.observe('turn_adjustment_before')
    if state.get('escape'):
        from turn_escape import preflight

        preflight(runner, state)
    runner.velocity_pub = runner.node.create_publisher(runner.twist, '/cmd_vel', 10)
    try:
        guarded_adjustment(runner, state)
    finally:
        runner.stop_translation()
        actual = runner.pose()
        if state.get('last_sample'):
            state['path_m'] += errors(actual, Pose(**state['last_sample']))[0]
            state['last_sample'] = asdict(actual)
    distance, heading = errors(actual, Pose(**state['target']))
    if distance > 0.003 or abs(heading) > 0.01 or state['path_m'] > 0.06:
        raise RuntimeError('Adjustment moved outside acceptance during final stop')
    state['done'] = True
    runner.observe('turn_adjustment_after')
