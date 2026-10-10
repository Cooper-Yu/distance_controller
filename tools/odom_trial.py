#!/usr/bin/env python3
"""Supervised P01-P05 odom-only trial. Laser records distances; it never stops motion."""

import argparse
from dataclasses import asdict
import fcntl
import json
import math
from pathlib import Path
import signal
import time
from action_plan import Pose, errors
from odom_path import compile_path, track, limited_velocity, waypoint_stop
from wall_runtime import WallRunner


class Trial:
    """Fixed-origin trial; incomplete actions retain progress only within this process."""

    def __init__(self, runner, data, origin, log):
        self.runner, self.route, self.log = runner, compile_path(origin, data), log
        self.cursor, self.index, self.partial = 0, 0, False
        self.last = origin
        self.emit('origin', dict(pose=asdict(origin), definition=data))

    def emit(self, event, data):
        self.log.write(json.dumps(dict(time=time.time(), event=event, data=data)) + '\n')
        self.log.flush()

    def observe(self):
        """Collect only while stopped; laser unavailability never changes route state."""
        self.emit('laser_observation', self.runner.observer.snapshot('odom_trial', timeout=0.3))

    def execute(self):
        r = self.runner
        if self.cursor >= len(self.route):
            print('All configured actions completed; stopped.')
            return
        r.exclusive()
        start = r.pose()
        if (
            self.last is None
            or errors(start, self.last)[0] > 0.03
            or abs(errors(start, self.last)[1]) > 0.05
        ):
            raise RuntimeError('Stopped pose changed; do not rebase the trial')
        segment = self.route[self.cursor]
        length = errors(start, segment['points'][-1])[0]
        budget = length / r.max_speed * 3 + 30 if segment['kind'] == 'translate' else 45
        self.partial = True
        self.emit('started', dict(action=self.cursor + 1, name=segment['name']))
        r.velocity_pub = r.node.create_publisher(r.twist, '/cmd_vel', 10)
        try:
            self.follow(segment, budget)
        finally:
            r.stop_translation()
            try:
                self.last = r.pose()
            except (RuntimeError, KeyboardInterrupt):
                self.last = None
        if (
            self.last is None
            or errors(self.last, segment['points'][-1])[0]
            > (0.03 if segment['kind'] == 'turn' else 0.02)
            or abs(errors(self.last, segment['points'][-1])[1]) > 0.02
        ):
            raise RuntimeError('Final endpoint outside tolerance')
        self.emit('completed', dict(action=self.cursor + 1, pose=asdict(self.last)))
        self.cursor, self.index, self.partial = self.cursor + 1, 0, False
        self.observe()
        print('Reached:', self.last, flush=True)

    def run_to(self, label):
        """Execute the remaining ordered actions to arrival; any error ends this batch."""
        stop = waypoint_stop(self.route, label)
        if self.partial:
            raise RuntimeError('Incomplete action; inspect then resume before run_to')
        if stop < self.cursor:
            raise RuntimeError('Waypoint already passed; run_to does not return or reset origin')
        self.emit('run_to_started', dict(waypoint=label, next_action=self.cursor + 1))
        while self.cursor < stop:
            self.runner.pump()
            self.execute()
        self.emit('run_to_completed', dict(waypoint=label, completed_actions=self.cursor))
        print(f'RUN_TO reached {label}; stopped. Use next or quit.', flush=True)

    def follow(self, segment, budget):
        """Odom-only command loop; stale odom, cancellation and competing publishers stop it."""
        r = self.runner
        deadline, previous_time = time.monotonic() + budget, time.monotonic()
        command, hold, last_log, last_check = (0, 0, 0), None, 0, 0
        while time.monotonic() < deadline:
            r.pump()
            actual = r.moving_pose()
            now = time.monotonic()
            if now - last_check > 0.5:
                if len(r.node.get_publishers_info_by_topic('/cmd_vel')) > 1:
                    raise RuntimeError('Another velocity publisher appeared; stop teleop')
                last_check = now
            self.index, requested, ready = track(actual, segment, self.index, r.max_speed)
            command = limited_velocity(command, requested, now - previous_time)
            previous_time = now
            r.send((0.0, 0.0, 0.0) if ready else command)
            if ready and r.latest[2] < 0.01 and r.latest[3] < 0.02:
                hold = hold or now
                if now - hold >= 0.5:
                    return
            else:
                hold = None
            if now - last_log >= 1:
                self.emit(
                    'progress',
                    dict(
                        action=self.cursor + 1,
                        pose=asdict(actual),
                        target=asdict(segment['points'][self.index]),
                        command=command,
                    ),
                )
                print(
                    f'ODOM_PROGRESS {segment["name"]} target={self.index + 1}/{len(segment["points"])} error={errors(actual, segment["points"][-1])}',
                    flush=True,
                )
                last_log = now
        raise RuntimeError('Odom action timeout')


def report_stop(trial, error):
    """End automatic advancement while keeping this process available for inspection."""
    trial.runner.cancel_requested = False
    trial.emit('stopped', dict(reason=str(error), cursor=trial.cursor, partial=trial.partial))
    print('STOPPED:', error, flush=True)


def interact(trial):
    """Keep key-point stops and explicit resume; no cross-process checkpoint guessing."""
    while True:
        try:
            cmd = input('odom> ').strip()
            if cmd == 'quit':
                return
            if cmd == 'status':
                print(
                    f'Next action={trial.cursor + 1}, partial={trial.partial}, actual={trial.runner.pose()}'
                )
            elif cmd == 'measure':
                trial.observe()
                print('Laser snapshot written to audit (log-only).')
            elif cmd.startswith('run_to '):
                trial.run_to(cmd.split(maxsplit=1)[1].strip())
            elif cmd in ('next', 'resume'):
                if trial.partial and cmd != 'resume':
                    raise RuntimeError('Incomplete action; inspect then resume the same target')
                trial.execute()
            else:
                print('next | run_to WAYPOINT | resume | status | measure | quit')
        except (RuntimeError, ValueError, KeyboardInterrupt) as error:
            report_stop(trial, error)
        except EOFError:
            return


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', type=Path)
    parser.add_argument('--until', help='After initialization execute in order to this waypoint')
    parser.add_argument('--start', choices=['prepare', 'current'])
    parser.add_argument('--laser-log-only', action='store_true')
    parser.add_argument('--speed', type=float, default=0.06)
    args = parser.parse_args()
    if not math.isfinite(args.speed) or not 0.02 <= args.speed <= 0.08:
        parser.error('speed must be .02-.08 m/s')
    if args.route is None:
        from ament_index_python.packages import get_package_share_directory

        args.route = (
            Path(get_package_share_directory('distance_controller'))
            / 'config/task6_odom_p01_p05.json'
        )
    data = json.loads(args.route.read_text())
    preview = compile_path(Pose(0, 0, 0), data)
    if args.until:
        waypoint_stop(preview, args.until)
        if not args.start:
            parser.error('--until requires --start prepare or current')
    for row in preview:
        print(row['name'], row['kind'], 'end=', row['points'][-1])
    if not args.start:
        return 0
    if not args.laser_log_only:
        parser.error('Motion requires --laser-log-only; supervise and stop manually')
    print('ODOM TRIAL: laser does NOT stop motion; watch robot. Ctrl+C stops. No restart resume.')
    with open('/tmp/distance_controller_survey_return.lock', 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        r = WallRunner(args.speed, 'left')
        old = signal.signal(signal.SIGINT, lambda *_: r.request_cancel())
        path = Path(f'odom_trial_{time.time_ns()}.jsonl')
        try:
            r.exclusive()
            if args.start == 'prepare':
                r.prepare_start()
            origin = r.pose()
            with path.open('x') as log:
                trial = Trial(r, data, origin, log)
                print('P01:', origin, 'audit:', path.resolve(), flush=True)
                if args.until:
                    try:
                        trial.run_to(args.until)
                    except (RuntimeError, ValueError, KeyboardInterrupt) as error:
                        report_stop(trial, error)
                interact(trial)
        finally:
            r.stop_translation()
            r.stop_owned()
            r.close(False)
            signal.signal(signal.SIGINT, old)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, OSError, KeyboardInterrupt) as error:
        print('STOPPED:', error)
        raise SystemExit(2) from error
