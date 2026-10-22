#!/usr/bin/env python3
"""One supervised <=5 cm backward step, or a nonmoving strict-laser diagnostic."""

import argparse
from dataclasses import asdict
import fcntl
import math
from pathlib import Path
import signal
import time

from current_frame_step_runtime import MANUAL_STEP_BUDGET_S, CurrentEpochGuard
from motion_console import Console, TeachingBook
from motion_model import Motion
from route_prefix_return_plan import ODOM
from route_return_runner import normalize_gid
from task6_route_runner import Cancellation, close_runner, emit
from wall_runtime import WallRunner


class StepCancellation(Cancellation):
    """No waiting input or automatic continuation; monotonic step budget is independent."""

    def __init__(self, runner):
        super().__init__(runner)
        self.deadline = None

    def check(self):
        super().check()
        if self.deadline is not None and time.monotonic() >= self.deadline:
            raise RuntimeError(f'Current-frame step timeout ({MANUAL_STEP_BUDGET_S:g} seconds)')


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--distance', type=float, required=True, help='Positive backward meters, <=.05'
    )
    parser.add_argument('--speed', type=float, default=0.03)
    parser.add_argument('--odom-publisher-gid', required=True)
    parser.add_argument('--raw-odom-publisher-gid', required=True)
    parser.add_argument('--output', type=Path, required=True)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument(
        '--supervised-manual',
        action='store_true',
        help='Operator watches each step; laser observation only',
    )
    mode.add_argument(
        '--strict-dry-run', action='store_true', help='Strict assessment only; no motion'
    )
    args = parser.parse_args(argv)
    if not math.isfinite(args.distance) or not 0 < args.distance <= 0.05:
        parser.error('Only a positive backward distance <= .05 m is supported')
    if not math.isfinite(args.speed) or not 0.01 <= args.speed <= 0.03:
        parser.error('Speed must be .01-.03 m/s')
    return args


def move_once(args, runner, guard, cancellation, audit):
    runner.exclusive()
    origin = guard.bind_stopped(cancellation)
    book = TeachingBook(args.output, origin, 'odom')
    console = Console(runner, book, cancellation, audit, 0.10, guard)
    console.record('LOCAL_START')
    guard.check()
    runner.geometry()
    runner.scan_pose()
    decision, snapshot = guard.laser.assess((-args.speed, 0.0, 0.0))
    guard.laser.report(decision, (-args.speed, 0.0, 0.0), snapshot)
    if args.strict_dry_run:
        emit(audit, 'strict_dry_run', decision=asdict(decision), motion_started=False)
        return
    print(
        'SUPERVISED MANUAL STEP: laser observation only. Operator watches this step; no automatic continuation.',
        flush=True,
    )
    emit(
        audit,
        'local_step_started',
        origin=asdict(origin),
        distance_m=args.distance,
        speed_cap_m_s=args.speed,
        gids=guard.gids,
        laser_policy='observation_only',
        supervised_manual=True,
        operator_supervision_required=True,
        historical_p01=False,
        task6_acceptance=False,
    )
    cancellation.deadline = time.monotonic() + MANUAL_STEP_BUDGET_S
    console.move(Motion('move', x=-args.distance))
    cancellation.check()
    guard.check()
    console.record('LOCAL_END')
    actual = runner.pose()
    cancellation.check()
    guard.check()
    emit(
        audit,
        'local_step_completed',
        actual=asdict(actual),
        local_start=asdict(origin),
        distance_m=args.distance,
        historical_p01=False,
        task6_acceptance=False,
    )


def execute(args, audit):
    runner = WallRunner(args.speed)
    cancellation = StepCancellation(runner)
    handlers = {
        sig: signal.signal(sig, cancellation.request) for sig in (signal.SIGINT, signal.SIGTERM)
    }
    guard = None
    try:
        gids = {
            ODOM[0]: normalize_gid(args.odom_publisher_gid),
            ODOM[1]: normalize_gid(args.raw_odom_publisher_gid),
        }
        guard = CurrentEpochGuard(
            runner,
            gids,
            args.distance,
            args.speed,
            lambda name, **data: emit(audit, name, **data),
            supervised_manual=args.supervised_manual,
        )
        move_once(args, runner, guard, cancellation, audit)
    finally:
        try:
            try:
                if guard is not None:
                    guard.close()
            finally:
                close_runner(runner)
        finally:
            for sig, handler in handlers.items():
                signal.signal(sig, handler)


def main(argv=None):
    args = arguments(argv)
    try:
        args.output = args.output.resolve()
        if args.output.suffix != '.json' or args.output.exists():
            raise ValueError('Use a new .json output for this current-frame step')
        normalize_gid(args.odom_publisher_gid)
        normalize_gid(args.raw_odom_publisher_gid)
        with open('/tmp/distance_controller_survey_return.lock', 'a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with args.output.with_suffix('.jsonl').open('x') as audit:
                try:
                    execute(args, audit)
                except (Exception, KeyboardInterrupt) as error:
                    emit(audit, 'local_step_failed', reason=str(error) or 'Cancelled')
                    raise
        return 0
    except (Exception, KeyboardInterrupt) as error:
        print('STOPPED:', str(error) or 'Cancelled', flush=True)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
