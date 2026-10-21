#!/usr/bin/env python3
"""Supervised reverse actions from one completed formal session; never prepare or rebase."""

import argparse
from dataclasses import asdict
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import time

from action_plan import Pose
from motion_console import Console, ConsoleInput, TeachingBook
from route_return_plan import ReturnSequence, load_source
from route_session_runtime import CheckedInput
from route_session_state import near
from task6_route_runner import Cancellation, close_runner, emit
from wall_runtime import WallRunner


def normalize_gid(value):
    text = ''.join(c for c in value.lower() if c not in '.:- ')
    if len(text) < 32 or len(text) % 2 or any(c not in '0123456789abcdef' for c in text):
        raise ValueError('Expected the previously observed odometry publisher GID in hex')
    return text


class Continuity:
    """Require the same single odometry publisher before and during every action."""

    def __init__(self, runner, expected, guard=None):
        self.runner, self.expected, self.guard = runner, expected, guard

    def check(self):
        infos = self.runner.node.get_publishers_info_by_topic('/odometry/filtered')
        gids = [bytes(info.endpoint_gid).hex() for info in infos]
        if gids != [self.expected]:
            raise RuntimeError('Odom publisher identity/count changed; return refused')
        if self.runner.frame != 'odom' or self.runner.twist_frame != 'base_link':
            raise RuntimeError('Return requires unchanged odom/base_link frames')

    def __call__(self, command):
        self.check()
        return self.guard(command) if self.guard else command


def attach(runner, source, continuity):
    runner.exclusive()
    actual = runner.pose()
    continuity.check()
    if runner.stamp < source.stamp_ns:
        raise RuntimeError('Odom time predates original stopped record')
    if not near(actual, source.last_stop, 0.02, math.radians(1)):
        raise RuntimeError('Attach requires original final pose within 2 cm / 1 deg')
    # Restore only the continuous-yaw representation; position and P01 are untouched.
    shift = round((source.last_stop.yaw - actual.yaw) / (2 * math.pi)) * 2 * math.pi
    runner.continuous_yaw += shift
    runner.latest = (Pose(actual.x, actual.y, actual.yaw + shift), *runner.latest[1:])


def claim_source(path, source):
    """Persist single use before the first motion; crash/fault never permits blind restart."""
    with path.open('x') as stream:
        json.dump(
            dict(
                session_id=source.session_id,
                source_hash=source.source_hash,
                claimed=time.time(),
                policy='one_attempt_no_resume',
            ),
            stream,
        )
        stream.flush()
        os.fsync(stream.fileno())


def run_one(console, sequence, cancellation, continuity, claim):
    continuity.check()
    console.runner.exclusive()
    motion = sequence.begin(console.runner.pose())
    if not claim['done']:
        claim_source(claim['path'], sequence.source)
        claim['done'] = True
    action = sequence.pending[0]
    console.emit(
        'return_action_started',
        index=sequence.cursor - 1,
        name=action.name,
        kind=action.kind,
        original_start=asdict(action.start),
        actual_start=asdict(sequence.pending[1]),
        motion=asdict(motion),
    )
    console.move(motion, use_path_length=True)
    cancellation.check()
    continuity.check()
    sequence.finish(console.runner.pose())
    console.record('RETURN_' + action.name)
    console.emit(
        'return_action_completed',
        name=action.name,
        cursor=sequence.cursor,
        actual=asdict(sequence.last_stop),
    )


def command_loop(console, sequence, cancellation, continuity, claim_path):
    claim = dict(path=claim_path, done=False)
    print('RETURN WAITING: plan | status | next (ONE action) | quit', flush=True)
    while sequence.cursor:
        console.runner.pump()
        cancellation.check()
        continuity.check()
        present, line = console.inputs.poll()
        if not present:
            continue
        if line is None or line == 'quit':
            console.emit('return_closed', cursor=sequence.cursor, complete=False)
            return
        if line in ('status', 'plan'):
            action = sequence.source.actions[sequence.cursor - 1]
            console.emit(
                'return_plan',
                cursor=sequence.cursor,
                action=asdict(action),
                actual=asdict(console.runner.pose()),
                origin=asdict(sequence.source.origin),
            )
            print(f'NEXT reverse {action.name} {action.kind}; target={action.start}', flush=True)
        elif line == 'next':
            try:
                run_one(console, sequence, cancellation, continuity, claim)
            except (Exception, KeyboardInterrupt) as error:
                sequence.fail(str(error) or 'Cancelled')
                console.emit('return_failed', cursor=sequence.cursor, reason=sequence.failure)
                raise
            print(f'RETURN WAITING remaining={sequence.cursor}', flush=True)
        else:
            print('REJECTED: plan | status | next | quit; no batches', flush=True)
    console.emit(
        'return_completed',
        actual=asdict(sequence.last_stop),
        original_p01=asdict(sequence.source.origin),
    )


def execute(args, source, claim_path, audit):
    runner = WallRunner(args.speed, alignment_wall='right')
    cancellation = Cancellation(runner)
    handlers = {
        sig: signal.signal(sig, cancellation.request) for sig in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        continuity = Continuity(runner, normalize_gid(args.odom_publisher_gid))
        attach(runner, source, continuity)
        book = TeachingBook(args.output, source.origin, 'odom')
        inputs = CheckedInput(ConsoleInput(), cancellation)
        console = Console(runner, book, inputs, audit, args.turn_speed, continuity)
        if args.laser_enforced:
            from route_laser_runtime import RouteLaserGuard

            continuity.guard = RouteLaserGuard(runner, console.emit)
        emit(
            audit,
            'return_session_started',
            source_session=source.session_id,
            source_hash=source.source_hash,
            source_origin=asdict(source.origin),
            publisher_gid=continuity.expected,
            laser_policy='enforced' if args.laser_enforced else 'observation_only',
            task6_avoidance_acceptance=False,
        )
        print('RETURN: original P01 retained; NO prepare, NO automatic movement', flush=True)
        if args.laser_log_only:
            print(
                'OBSERVATION ONLY: no laser obstacle stop; not Task6 avoidance acceptance',
                flush=True,
            )
        command_loop(console, ReturnSequence(source), cancellation, continuity, claim_path)
    finally:
        try:
            close_runner(runner)
        finally:
            for sig, handler in handlers.items():
                signal.signal(sig, handler)


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--confirm-same-odom', required=True, metavar='SOURCE_SESSION_UUID')
    parser.add_argument('--odom-publisher-gid', required=True)
    parser.add_argument('--output', type=Path, required=True)
    policy = parser.add_mutually_exclusive_group(required=True)
    policy.add_argument('--laser-log-only', action='store_true')
    policy.add_argument('--laser-enforced', action='store_true')
    parser.add_argument('--speed', type=float, default=0.03)
    parser.add_argument('--turn-speed', type=float, default=0.10)
    args = parser.parse_args(argv)
    if not math.isfinite(args.speed) or not 0.01 <= args.speed <= 0.03:
        parser.error('Return speed must be .01-.03 m/s')
    if not math.isfinite(args.turn_speed) or not 0.02 <= args.turn_speed <= 0.10:
        parser.error('Return turn speed must be .02-.10 rad/s')
    return args


def main(argv=None):
    args = arguments(argv)
    try:
        source = load_source(args.source)
        if args.confirm_same_odom != source.session_id:
            raise ValueError('Confirm the exact original session; do not adopt a different run')
        normalize_gid(args.odom_publisher_gid)
        claim = args.source.with_suffix('.return_claim.json')
        if claim.exists():
            raise ValueError('This source already has a return attempt; restart/resume forbidden')
        args.output = args.output.resolve()
        if args.output.suffix != '.json' or args.output.exists():
            raise ValueError('Use a new .json output')
        with open('/tmp/distance_controller_survey_return.lock', 'a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with args.output.with_suffix('.jsonl').open('x') as audit:
                try:
                    execute(args, source, claim, audit)
                except (Exception, KeyboardInterrupt) as error:
                    emit(audit, 'return_session_failed', reason=str(error) or 'Cancelled')
                    raise
        print('Return session stopped; inspect return_completed for P01 completion', flush=True)
        return 0
    except (Exception, KeyboardInterrupt) as error:
        print('STOPPED:', str(error) or 'Cancelled', flush=True)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
