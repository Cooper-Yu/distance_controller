#!/usr/bin/env python3
"""One bounded left half-turn from a fresh stopped pose; never resumes a P01 route."""

import argparse
import fcntl
from dataclasses import asdict
import json
import math
from pathlib import Path
import signal
import time

from motion_console import Console
from motion_model import Motion
from task6_route_runner import Cancellation, close_runner
from wall_runtime import WallRunner


def identities(runner):
    result = {}
    for topic in ('/odometry/filtered', '/rosbot_xl_base_controller/odom'):
        publishers = runner.node.get_publishers_info_by_topic(topic)
        if len(publishers) != 1:
            raise RuntimeError(f'Requires exactly one odom publisher: {topic}')
        result[topic] = bytes(publishers[0].endpoint_gid).hex()
    return result


def execute(args, audit):
    runner = WallRunner(0.03, alignment_wall='right')
    cancellation = Cancellation(runner)
    handlers = {
        sig: signal.signal(sig, cancellation.request) for sig in (signal.SIGINT, signal.SIGTERM)
    }
    result, error = {}, None
    started = time.monotonic()
    try:
        runner.exclusive()
        cancellation.check()
        start = runner.pose()
        expected = identities(runner)
        console = Console(runner, None, cancellation, audit, args.turn_speed)

        def feedback_only(command):
            cancellation.check()
            if identities(runner) != expected:
                raise RuntimeError('Odom publisher identity changed')
            runner.moving_pose()
            points, _ = runner.geometry()
            if len(points) < 20:
                raise RuntimeError('Insufficient valid laser returns')
            return command

        console.command_guard = feedback_only
        feedback_only((0.0, 0.0, 0.0))
        console.emit(
            'current_turn_started',
            anchor=asdict(start),
            signed_angle_rad=math.pi,
            reference='fresh_current_pose_not_P01',
            laser_policy='observation_only',
            turn_speed=args.turn_speed,
            odom_publishers=expected,
        )
        remaining = args.max_seconds - (time.monotonic() - started)
        if remaining <= 0:
            raise RuntimeError('Current-turn startup deadline exceeded')
        actual = console.finish_motion(
            start, Motion('turn', angle=math.pi), remaining, initial_exclusive_done=True
        )
        cancellation.check()
        feedback_only((0.0, 0.0, 0.0))
        result = dict(
            anchor=asdict(start),
            actual=asdict(actual),
            signed_rotation_rad=actual.yaw - start.yaw,
            rotation_error_rad=math.pi - (actual.yaw - start.yaw),
            center_drift_m=math.hypot(actual.x - start.x, actual.y - start.y),
        )
        console.emit('current_turn_completed', **result)
    except (Exception, KeyboardInterrupt) as failure:
        error = str(failure) or 'Cancelled'
    finally:
        try:
            close_runner(runner)
        except Exception as failure:
            error = error or f'Cleanup failed: {failure}'
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
    result.update(
        success=error is None,
        error=error,
        elapsed_s=time.monotonic() - started,
        scope='standalone_current_turn_not_full_route_acceptance',
        laser_policy='observation_only',
        swept_envelope_gate=False,
    )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--turn-speed', type=float, default=0.20)
    parser.add_argument('--max-seconds', type=float, default=45.0)
    args = parser.parse_args()
    if not math.isfinite(args.turn_speed) or not 0.02 <= args.turn_speed <= 0.20:
        parser.error('turn-speed must be .02..20 rad/s')
    if not math.isfinite(args.max_seconds) or not 20 <= args.max_seconds <= 60:
        parser.error('max-seconds must be 20..60')
    path = args.output.resolve()
    if path.suffix != '.json' or not path.parent.is_dir():
        parser.error('output requires an existing directory and .json suffix')
    if path.exists() or path.with_suffix('.jsonl').exists():
        parser.error('output already exists')
    with open('/tmp/distance_controller_survey_return.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with path.with_suffix('.jsonl').open('x') as audit:
            result = execute(args, audit)
    with path.open('x') as output:
        json.dump(result, output, indent=2, allow_nan=False)
    print(json.dumps(result, indent=2, allow_nan=False), flush=True)
    return 0 if result['success'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
