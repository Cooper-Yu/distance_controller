#!/usr/bin/env python3
"""Explicit backward steps in one fixed NEW-epoch frame; manual observation only."""

import argparse
from dataclasses import asdict
import fcntl
import json
import math
from pathlib import Path
import signal
import time

from current_frame_session_runtime import (
    ReadyPairingError,
    ReadyPairingWait,
    SessionGuard,
    SessionRunner,
    StepPermit,
)
from current_frame_step_runtime import MANUAL_STEP_BUDGET_S
from motion_console import Console, ConsoleInput, TeachingBook
from motion_model import local_delta
from route_prefix_return_plan import ODOM
from route_return_runner import normalize_gid
from task6_route_runner import Cancellation, close_runner, emit
from teaching_path import path_ready


class SessionCancellation(Cancellation):
    """The session deadline never resets; each requested step has one shared bounded budget."""

    def __init__(self, runner, seconds):
        super().__init__(runner)
        self.session_deadline = time.monotonic() + seconds
        self.step_deadline = None

    def check(self):
        super().check()
        now = time.monotonic()
        if now >= self.session_deadline:
            raise RuntimeError('Local session deadline exceeded')
        if self.step_deadline is not None and now >= self.step_deadline:
            raise RuntimeError(f'Local session step timeout ({MANUAL_STEP_BUDGET_S:g} seconds)')


class CheckedInput:
    """Keep feedback and cancellation checks active inside Console's motion loop."""

    def __init__(self, source, cancellation, guard):
        self.source, self.cancellation, self.guard = source, cancellation, guard

    def idle_poll(self):
        """Only called immediately after the idle loop independently checks all feedback."""
        self.cancellation.check()
        return self.source.poll()

    def poll(self):
        self.cancellation.check()
        self.guard.check()
        return self.source.poll()


class SessionBook(TeachingBook):
    """Use local-session labels, never the legacy teaching book's P01 field names."""

    def record(self, name, pose, stamp, observation):
        if not name.startswith('LOCAL_') or name in self.data['points']:
            raise ValueError('Use a new LOCAL_ record label')
        row = dict(
            pose=asdict(pose),
            relative_to_session=local_delta(self.origin, pose),
            odom_stamp_ns=stamp,
            recorded=time.time(),
            laser=observation,
        )
        self.data.update(reference='LOCAL_SESSION_START', historical_p01=False, resumable=False)
        self.data['points'][name] = row
        temporary = self.path.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(self.data, indent=2, allow_nan=False) + '\n')
        temporary.replace(self.path)
        return row


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--step-distance', type=float, required=True)
    parser.add_argument('--speed', type=float, default=0.01)
    parser.add_argument('--max-seconds', type=float, default=600)
    parser.add_argument('--odom-publisher-gid', required=True)
    parser.add_argument('--raw-odom-publisher-gid', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--supervised-manual', action='store_true', required=True)
    args = parser.parse_args(argv)
    for value, low, high, label in (
        (args.step_distance, 0, 0.05, 'Step distance'),
        (args.speed, 0.01, 0.03, 'Speed'),
        (args.max_seconds, 1, 1800, 'Session seconds'),
    ):
        if not math.isfinite(value) or not low <= value <= high or value == 0:
            parser.error(f'{label} must be finite in {low}..{high}, nonzero')
    return args


def record_stop(console, guard, label):
    before = guard.check()
    observation = console.measure()
    after = guard.check()
    if not near_record(before, after):
        raise RuntimeError('Pose changed while recording local stop')
    point = console.book.record(label, after, console.runner.stamp, observation)
    console.emit('record', name=label, point=point)
    return after


def near_record(before, after):
    delta = local_delta(before, after)
    return math.hypot(delta['x_m'], delta['y_m']) <= 0.005 and abs(
        delta['yaw_deg']
    ) <= math.degrees(0.01)


def publish_ready(console, guard, permit, cancellation, actual=None, recovered=()):
    cancellation.check()
    actual = guard.check() if actual is None else actual
    token = permit.ready()
    console.emit(
        'session_ready',
        session_id=console.book.data['session_id'],
        next_index=permit.index,
        ready_token=token,
        anchor=asdict(guard.anchor),
        actual=asdict(actual),
        odom_stamp_ns=console.runner.stamp,
        cmd_publishers=0,
        recovered_scan_stamps=list(recovered),
    )
    print(f'READY {permit.index}: step {token} | quit', flush=True)


def run_step(console, guard, permit, cancellation, distance):
    cancellation.step_deadline = time.monotonic() + MANUAL_STEP_BUDGET_S
    cancellation.check()
    motion, actual = guard.begin_step(distance)
    data = dict(
        index=permit.index, session_id=console.book.data['session_id'], anchor=asdict(guard.anchor)
    )
    console.emit('step_started', **data, actual_start=asdict(actual), motion=asdict(motion))
    console.move(motion, reconnect=False, use_path_length=True)
    cancellation.check()
    actual = guard.finish_step(cancellation)
    if not path_ready(guard.anchor, actual, motion.points, 0.0):
        raise RuntimeError('Local step endpoint failed fixed-reference acceptance')
    actual = record_stop(console, guard, f'LOCAL_STEP_{permit.index:04d}')
    cancellation.check()
    if not path_ready(guard.anchor, actual, motion.points, 0.0):
        raise RuntimeError('Local step endpoint changed during record')
    console.emit('step_completed', **data, actual=asdict(actual))
    permit.finish()
    cancellation.step_deadline = None


def discard_idle_commands(console, reason):
    """Bound draining so a queued-input flood cannot starve feedback/deadline checks."""
    for _ in range(64):
        present, line = console.inputs.idle_poll()
        if not present:
            return 'empty'
        if line is None or line in ('quit', 'stop'):
            return 'quit'
        console.emit('command_rejected', reason=reason)
    return 'more'


def idle_check(console, guard, permit, cancellation, waiting):
    cancellation.check()
    waiting.check_deadline(time.monotonic())
    try:
        actual = guard.check(allow_pair_wait=True)
    except ReadyPairingError as error:
        first = waiting.failed(error.stamp, time.monotonic())
        revoked, permit.token = permit.token, None
        if first:
            console.emit(
                'session_waiting',
                session_id=console.book.data['session_id'],
                anchor=asdict(guard.anchor),
                next_index=permit.index,
                reason_code='SCAN_ODOM_BRACKET_TOO_WIDE',
                reason=str(error),
                scan_stamp_ns=error.stamp,
                revoked_ready_token=revoked,
                waiting_started_mono=waiting.started,
                waiting_deadline_mono=waiting.deadline,
            )
        return None
    return actual if waiting.paired(console.runner.cached_stamp, time.monotonic()) else None


def command_loop(console, guard, permit, cancellation, distance):
    waiting = ReadyPairingWait()
    while True:
        console.runner.pump()
        actual = idle_check(console, guard, permit, cancellation, waiting)
        if actual is None or permit.token is None:
            reason = (
                'Pairing wait: no READY token; command discarded'
                if actual is None
                else 'Queued command discarded; no automatic next'
            )
            drained = discard_idle_commands(console, reason)
            if drained == 'quit':
                return
            if actual is None or drained == 'more':
                continue
            actual = idle_check(console, guard, permit, cancellation, waiting)
            if actual is None:
                continue
            publish_ready(console, guard, permit, cancellation, actual, waiting.paired_stamps)
            waiting = ReadyPairingWait()
        present, line = console.inputs.idle_poll()
        if not present:
            continue
        if line is None or line in ('quit', 'stop'):
            return
        if not permit.take(line):
            console.emit('command_rejected', reason='Expected current one-use READY token')
            continue
        run_step(console, guard, permit, cancellation, distance)


def execute(args, audit):
    runner = SessionRunner(args.speed)
    cancellation = SessionCancellation(runner, args.max_seconds)
    handlers = {
        sig: signal.signal(sig, cancellation.request) for sig in (signal.SIGINT, signal.SIGTERM)
    }
    guard, book, failure = None, None, None
    permit = StepPermit()
    try:
        runner.exclusive()
        gids = {
            ODOM[0]: normalize_gid(args.odom_publisher_gid),
            ODOM[1]: normalize_gid(args.raw_odom_publisher_gid),
        }
        guard = SessionGuard(
            runner, gids, args.speed, lambda name, **data: emit(audit, name, **data)
        )
        anchor = guard.bind_session(cancellation)
        book = SessionBook(args.output, anchor, 'odom')
        inputs = CheckedInput(ConsoleInput(), cancellation, guard)
        console = Console(runner, book, inputs, audit, 0.10, guard)
        record_stop(console, guard, 'LOCAL_SESSION_START')
        emit(
            audit,
            'session_started',
            session_id=book.data['session_id'],
            anchor=asdict(anchor),
            gids=gids,
            step_distance_m=args.step_distance,
            speed_cap_m_s=args.speed,
            laser_policy='observation_only',
            supervised_manual=True,
            operator_supervision_required=True,
            historical_p01=False,
            task6_acceptance=False,
        )
        command_loop(console, guard, permit, cancellation, args.step_distance)
    except (Exception, KeyboardInterrupt) as error:
        failure = error
    finally:
        try:
            try:
                if guard is not None:
                    guard.close()
            finally:
                close_runner(runner)
        except (Exception, KeyboardInterrupt) as error:
            emit(audit, 'cleanup_failed', reason=str(error) or 'Cancelled')
            failure = failure or error
        finally:
            for sig, handler in handlers.items():
                signal.signal(sig, handler)
    session_id = book.data['session_id'] if book is not None else None
    if failure is not None:
        emit(audit, 'session_failed', session_id=session_id, reason=str(failure) or 'Cancelled')
        raise failure
    emit(audit, 'session_closed', session_id=session_id, completed_steps=permit.index - 1)


def main(argv=None):
    args = arguments(argv)
    try:
        args.output = args.output.resolve()
        if args.output.suffix != '.json' or args.output.exists():
            raise ValueError('Use a new .json output; local sessions cannot resume')
        normalize_gid(args.odom_publisher_gid)
        normalize_gid(args.raw_odom_publisher_gid)
        with open('/tmp/distance_controller_survey_return.lock', 'a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with args.output.with_suffix('.jsonl').open('x') as audit:
                execute(args, audit)
        return 0
    except (Exception, KeyboardInterrupt) as error:
        print('STOPPED:', str(error) or 'Cancelled', flush=True)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
