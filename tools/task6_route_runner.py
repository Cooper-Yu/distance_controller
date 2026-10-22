#!/usr/bin/env python3
"""Scene-2 backend: default log-only route or explicit guarded supervised session."""

import argparse
from dataclasses import asdict
import fcntl
import json
import math
from pathlib import Path
import signal
import time

from motion_console import Console, ConsoleInput, TeachingBook
from task6_route_model import reconnect_route, validate_manifest
from task6_terminal import TERMINAL_NAME, terminal_guard, turn_motion
from wall_runtime import WallRunner


class Cancellation:
    """Latch cancellation across preparation, which clears its own transient cancel flag."""

    def __init__(self, runner):
        self.runner, self.signum = runner, None

    def request(self, signum, _frame):
        self.signum = signum
        self.runner.request_cancel()

    def check(self):
        if self.signum is not None:
            raise KeyboardInterrupt(f'Cancelled by signal {self.signum}')

    def poll(self):
        self.check()
        return False, None


def emit(audit, event, **data):
    audit.write(json.dumps(dict(time=time.time(), event=event, **data), allow_nan=False) + '\n')
    audit.flush()


def run_actions(console, actions, cancellation):
    """Execute exactly the supplied actions; a failed action aborts the entire route."""
    after_turn = False
    for index, action in enumerate(actions):
        cancellation.check()
        pause = getattr(console.runner, 'feedback_pause', None)
        if pause is not None:
            pause.check()
        console.emit('route_action_started', index=index, name=action.name)
        if action.kind == 'turn':
            motion = turn_motion(action, console.book.origin, console.runner.pose())
        else:
            motion = action.path_motion()
            if after_turn:
                adjusted = reconnect_route(motion, console.book.origin, console.runner.pose())
                console.emit(
                    'post_turn_reconnect', original=asdict(motion), adjusted=asdict(adjusted)
                )
                motion = adjusted
        with terminal_guard(console, action):
            console.move(motion)
        cancellation.check()
        console.record(action.name)
        console.emit('route_action_completed', index=index, name=action.name)
        after_turn = action.kind == 'turn'
    cancellation.check()
    console.runner.pose()


def configure_route_scan_policy(runner, supervised):
    """Relax only observation geometry after strict preparation has completed."""
    runner.observation_scan_timeout = 0.5 if supervised else 1.0


def prepare_and_run(runner, path, audit, actions, turn_speed, cancellation, supervised=False):
    runner.prepare_options += [
        '-p',
        'positioning_wall:=right',
        '-p',
        'right_target_distance:=0.26',
        '-p',
        'precise_start:=true',
    ]
    runner.exclusive()
    cancellation.check()
    emit(audit, 'preparation_started', precise=True, alignment_wall='right', right_target_m=0.26)
    runner.prepare_start()
    cancellation.check()
    origin = runner.pose()
    if runner.frame != 'odom' or runner.twist_frame != 'base_link':
        raise RuntimeError('Requires odom frame and base_link child frame')
    emit(audit, 'preparation_handoff', origin=asdict(origin))
    configure_route_scan_policy(runner, supervised)
    emit(
        audit,
        'route_scan_policy',
        source_receipt_limit_s=runner.observation_scan_timeout,
        odometry_limit_s=0.5,
        initialization_limit_s=0.5,
    )
    book = TeachingBook(path, origin, runner.frame)
    console = Console(runner, book, cancellation, audit, turn_speed)
    if not supervised:
        from route_feedback_pause import RouteFeedbackPause

        runner.feedback_pause = RouteFeedbackPause(runner, console.emit, cancellation.check)
    console.record('P01')
    if supervised:
        from route_laser_runtime import RouteLaserGuard
        from route_session_runtime import run_supervised

        console.command_guard = RouteLaserGuard(runner, console.emit)
        return run_supervised(console, actions, cancellation, ConsoleInput())
    run_actions(console, actions, cancellation)


def close_runner(runner):
    """Always attempt all cleanup steps, including after a prior cleanup failure."""
    failures = []
    for cleanup in (runner.stop_translation, runner.stop_owned, lambda: runner.close(False)):
        try:
            cleanup()
        except (Exception, KeyboardInterrupt) as error:
            failures.append(str(error) or type(error).__name__)
    if failures:
        raise RuntimeError('Cleanup failed: ' + '; '.join(failures))


def session(args, path, audit, actions):
    supervised = getattr(args, 'supervised', False)
    emit(
        audit,
        'route_started',
        scene=2,
        frame='P01',
        laser_policy='enforced' if getattr(args, 'supervised', False) else 'observation_only',
        implementation=(
            'supervised_experimental'
            if getattr(args, 'supervised', False)
            else 'intermediate_no_obstacle_avoidance'
        ),
        actions=[asdict(a) for a in actions],
        terminal_laser_policy=(
            ('enforced' if supervised else 'feedback_only')
            if actions[-1].name == TERMINAL_NAME
            else 'not_applicable'
        ),
    )
    runner = WallRunner(min(args.speed, 0.03) if supervised else args.speed, alignment_wall='right')
    cancellation = Cancellation(runner)
    handlers = {
        sig: signal.signal(sig, cancellation.request) for sig in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        try:
            if supervised:
                prepare_and_run(
                    runner,
                    path,
                    audit,
                    actions,
                    min(args.turn_speed, 0.10),
                    cancellation,
                    supervised=True,
                )
            else:
                prepare_and_run(runner, path, audit, actions, args.turn_speed, cancellation)
        finally:
            close_runner(runner)
        cancellation.check()
        if supervised:
            emit(audit, 'supervised_stopped', exit_code=0)
        else:
            destination = 'P15' if actions[-1].name == TERMINAL_NAME else actions[-1].name
            emit(
                audit,
                'route_completed',
                destination=destination,
                final_action=actions[-1].name,
                exit_code=0,
            )
    finally:
        for sig, handler in handlers.items():
            signal.signal(sig, handler)


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=int, choices=[2], required=True)
    parser.add_argument('--route-json', type=Path, required=True)
    policy = parser.add_mutually_exclusive_group(required=True)
    policy.add_argument('--laser-log-only', action='store_true')
    policy.add_argument(
        '--supervised',
        action='store_true',
        help='Experimental same-process single actions with enforced laser guard',
    )
    parser.add_argument('--speed', type=float, default=0.04)
    parser.add_argument('--turn-speed', type=float, default=0.10)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    if not math.isfinite(args.speed) or not 0.01 <= args.speed <= 0.12:
        parser.error('speed must be .01-.12 m/s')
    if not math.isfinite(args.turn_speed) or not 0.02 <= args.turn_speed <= 0.60:
        parser.error('turn-speed must be .02-.60 rad/s')
    return args


def main(argv=None):
    args = arguments(argv)
    try:
        actions = validate_manifest(json.loads(args.route_json.read_text()))
        path = (args.output or Path(f'task6_route_{time.time_ns()}.json')).resolve()
        if path.suffix != '.json' or not path.parent.is_dir():
            raise ValueError('Output must end with .json in an existing directory')
        if path.exists() or path.with_suffix('.jsonl').exists():
            raise ValueError('Output already exists; choose a new session file')
        with open('/tmp/distance_controller_survey_return.lock', 'a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with path.with_suffix('.jsonl').open('x') as audit:
                try:
                    session(args, path, audit, actions)
                except (Exception, KeyboardInterrupt) as error:
                    emit(audit, 'route_failed', reason=str(error) or 'Cancelled', exit_code=2)
                    raise
        if args.supervised:
            print('Supervised session stopped; exit0 is not route completion', flush=True)
        else:
            print('Route completed; stopped:', actions[-1].name, flush=True)
        return 0
    except (Exception, KeyboardInterrupt) as error:
        print('STOPPED:', str(error) or 'Cancelled', flush=True)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
