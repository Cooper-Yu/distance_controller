#!/usr/bin/env python3
"""Explicit single-action recovery of an immutable completed stopped path prefix."""

import argparse
from dataclasses import asdict
import fcntl
import math
from pathlib import Path
import signal
import time

from motion_console import Console, ConsoleInput, TeachingBook
from route_prefix_return_plan import ODOM, load_prefix_run
from route_prefix_return_runtime import PrefixContinuity
from route_return_plan import ReturnSequence
from route_return_runner import attach, command_loop, normalize_gid
from route_session_runtime import CheckedInput
from task6_route_runner import Cancellation, close_runner, emit
from wall_runtime import WallRunner


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-run-dir', type=Path, required=True)
    parser.add_argument('--confirm-same-odom', required=True)
    parser.add_argument('--odom-publisher-gid', required=True)
    parser.add_argument('--raw-odom-publisher-gid', required=True)
    parser.add_argument('--output', type=Path, required=True)
    policy = parser.add_mutually_exclusive_group(required=True)
    policy.add_argument('--laser-log-only', action='store_true')
    policy.add_argument('--laser-enforced', action='store_true')
    parser.add_argument('--speed', type=float, default=0.03)
    parser.add_argument('--turn-speed', type=float, default=0.10)
    args = parser.parse_args(argv)
    if not math.isfinite(args.speed) or not 0.01 <= args.speed <= 0.03:
        parser.error('Prefix return speed must be .01-.03 m/s')
    if not math.isfinite(args.turn_speed) or not 0.02 <= args.turn_speed <= 0.10:
        parser.error('Prefix return turn speed must be .02-.10 rad/s')
    return args


def verify_request(args, source, metadata):
    if args.confirm_same_odom != source.session_id:
        raise ValueError('Confirm the exact failed source session UUID')
    requested = (args.odom_publisher_gid, args.raw_odom_publisher_gid)
    if any([normalize_gid(gid)] != metadata['gids'][topic] for topic, gid in zip(ODOM, requested)):
        raise ValueError('Requested odometry identities differ from original prefix')
    claim = args.source_run_dir / 'points.return_claim.json'
    if claim.exists():
        raise ValueError('This source already has a return attempt; restart/resume forbidden')
    args.output = args.output.resolve()
    if args.output.suffix != '.json' or args.output.exists():
        raise ValueError('Use a new .json output')
    if args.output.is_relative_to(args.source_run_dir.resolve()):
        raise ValueError('Write recovery evidence outside the immutable failed source run')
    return claim


def run_session(args, source, metadata, runner, cancellation, continuity, claim, audit):
    deadline = time.monotonic() + 3
    while continuity.raw is None and time.monotonic() < deadline:
        runner.pump()
        cancellation.check()
    attach(runner, source, continuity)
    book = TeachingBook(args.output, source.origin, 'odom')
    inputs = CheckedInput(ConsoleInput(), cancellation)
    console = Console(runner, book, inputs, audit, args.turn_speed, continuity)
    if args.laser_enforced:
        from route_laser_runtime import RouteLaserGuard

        continuity.guard = RouteLaserGuard(runner, console.emit)
    emit(audit, 'prefix_recovery_admitted', **metadata)
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
    print(
        'PREFIX RETURN: NO prepare/rebase/auto-next. Original failed run remains failed.',
        flush=True,
    )
    if args.laser_log_only:
        print('OBSERVATION ONLY: no laser obstacle stop; path inspection required.', flush=True)
    sequence = ReturnSequence(source)
    command_loop(console, sequence, cancellation, continuity, claim)
    return 0 if sequence.cursor == 0 else 2


def execute(args, source, metadata, claim, audit):
    runner = WallRunner(args.speed, alignment_wall='right')
    cancellation = Cancellation(runner)
    handlers = {
        sig: signal.signal(sig, cancellation.request) for sig in (signal.SIGINT, signal.SIGTERM)
    }
    continuity = None
    try:
        continuity = PrefixContinuity(runner, source, metadata)
        return run_session(args, source, metadata, runner, cancellation, continuity, claim, audit)
    finally:
        try:
            try:
                if continuity is not None:
                    continuity.close()
            finally:
                close_runner(runner)
        finally:
            for sig, handler in handlers.items():
                signal.signal(sig, handler)


def main(argv=None):
    args = arguments(argv)
    try:
        source, metadata = load_prefix_run(args.source_run_dir)
        claim = verify_request(args, source, metadata)
        with open('/tmp/distance_controller_survey_return.lock', 'a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with args.output.with_suffix('.jsonl').open('x') as audit:
                try:
                    return execute(args, source, metadata, claim, audit)
                except (Exception, KeyboardInterrupt) as error:
                    emit(audit, 'return_session_failed', reason=str(error) or 'Cancelled')
                    raise
    except (Exception, KeyboardInterrupt) as error:
        print('STOPPED:', str(error) or 'Cancelled', flush=True)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
