"""Adjust one completed front-wall endpoint without advancing the route cursor."""

from copy import deepcopy
from dataclasses import asdict, replace
import math
from action_plan import Pose, Step, errors
from wall_policy import Policy


def pending(session):
    """An interrupted endpoint correction owns movement until its fixed target completes."""
    state = session.active[-1].get('front_adjustment') if session.active else None
    return bool(state and not state['done'])


def adjust_session(session, clearance):
    """Use an absolute body clearance, preserving original endpoint and route evidence."""
    if not math.isfinite(clearance) or not 0.06 <= clearance <= 0.15:
        raise ValueError('Front adjustment clearance must be within [0.06,0.15] m')
    if session.partial or not session.active:
        raise ValueError('adjust_front requires a completed front-stop action, no partial action')
    record = session.active[-1]
    step = Step(**record['step'])
    if step.kind != 'forward' or (step.wall or {}).get('stop') != 'front':
        raise ValueError('adjust_front requires a completed forward/front action')
    actual = session.check_location()
    state = record.get('front_adjustment')
    if state is None:
        state = {
            'origin': asdict(actual),
            'clearance': clearance,
            'path_m': 0.0,
            'done': False,
            'original_end': deepcopy(record['end']),
            'original_target': deepcopy(record['target']),
        }
        record['front_adjustment'] = state
    if abs(state['clearance'] - clearance) > 1e-9:
        raise ValueError('Existing front adjustment is fixed; repeat the same clearance')
    if state['done']:
        print('FRONT_ADJUSTMENT already completed; no motion', flush=True)
        return actual
    session.emit('front_adjustment_started', state)
    try:
        run_adjustment(session.backend, step, state)
        actual = session.backend.pose()
        state['done'] = True
        record['end'] = asdict(actual)
        record['target'].update(x=actual.x, y=actual.y)
        session.last_actual = actual
        session.emit('front_adjustment_completed', record)
        print('FRONT_ADJUSTMENT completed; WAITING; inspect before next', flush=True)
        return actual
    except (Exception, KeyboardInterrupt):
        try:
            session.last_actual = session.backend.pose()
        except (Exception, KeyboardInterrupt):
            session.last_actual = None
        session.emit('front_adjustment_stopped', state)
        raise


def run_adjustment(runner, step, state):
    """Reuse guarded wall control at <=2 cm/s; keep cumulative travel across retries."""
    policy = replace(
        Policy(**step.wall),
        stop_clearance=state['clearance'],
        stop_tolerance=0.005,
        max_travel=0.08,
        capture='none',
        align_follow_first=False,
    )
    correction = Step('front_endpoint_adjustment', 'forward', 0.001, asdict(policy)).validate()
    target = Pose(**state['original_target'])
    origin = Pose(**state['origin'])
    old = {
        key: getattr(runner, key, None)
        for key in ('max_speed', 'resume_origin', 'resume_path', 'endpoint_adjustment_origin')
    }
    runner.max_speed = min(runner.max_speed, 0.02)
    runner.resume_origin, runner.resume_path = origin, state['path_m']
    runner.endpoint_adjustment_origin = origin
    runner.last_report = None
    try:
        runner.execute(correction, target)
        actual = runner.pose()
        if errors(actual, origin)[0] > 0.06 or abs(errors(actual, target)[1]) > 0.02:
            raise RuntimeError('FRONT_ADJUSTMENT: final stopped pose outside bounds')
        # The endpoint window already held all commands at zero. Verify again
        # after stop_translation; do not replace the window with one noisy fit.
        from front_settling import verify_stopped

        runner.velocity_pub = runner.node.create_publisher(runner.twist, '/cmd_vel', 10)
        try:
            final_window = verify_stopped(runner, target, policy)
        finally:
            runner.stop_translation()
        runner.last_report['final_stopped_window'] = final_window
        state['result'] = deepcopy(runner.last_report)
    finally:
        report = runner.last_report or {}
        state['path_m'] = max(state['path_m'], report.get('path_m', 0.0))
        try:
            actual = runner.pose()
            if runner.path_last_pose is not None:
                state['path_m'] += errors(actual, runner.path_last_pose)[0]
            state['last_sample'] = asdict(actual)
        finally:
            for key, value in old.items():
                setattr(runner, key, value)
    if state['path_m'] > 0.08:
        raise RuntimeError('FRONT_ADJUSTMENT: cumulative travel exceeded 8 cm')
