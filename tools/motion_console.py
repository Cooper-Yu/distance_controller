#!/usr/bin/env python3
"""Supervised relative motion and same-session teaching; laser observations never gate motion."""

import argparse
from dataclasses import asdict
import fcntl
import json
import math
from pathlib import Path
import queue
import signal
import sys
import threading
import time
import uuid

from motion_model import HeadingPid, Motion, feedback, local_delta, parse_batch, target_pose
from odom_path import limited_velocity
from teaching_path import PathTracker, load_p02, load_p03, path_ready
from wall_runtime import WallRunner

HELP = """move forward|backward|left|right METERS [--heading on|off] [--lateral on|off]
move xy FORWARD_METERS LEFT_METERS [--heading on|off] [--lateral on|off]
turn left|right DEGREES
path_plan [P03_turn] | path P02 | path P03_turn (fixed P01; P03 route ends right 90)
record NAME | delta FROM TO | points | status | measure | stop | quit
Use ; for sequential commands. Each motion anchors at its own actual start.
During motion: stop + Enter, quit + Enter, or Ctrl+C cancels the entire batch.
Laser is observation-only: NO obstacle stop. Coordinates are odom estimates."""


class ConsoleInput:
    """Read complete lines in a daemon; only the ROS thread changes motion state."""

    def __init__(self):
        self.lines = queue.Queue()
        threading.Thread(target=self.read, daemon=True).start()

    def read(self):
        for line in sys.stdin:
            if line.strip():
                self.lines.put(line.strip())
        self.lines.put(None)

    def poll(self):
        try:
            return True, self.lines.get_nowait()
        except queue.Empty:
            return False, None


class TeachingBook:
    """Atomic point snapshots; never load old-session coordinates as live targets."""

    def __init__(self, path, origin, frame):
        self.path = path
        self.data = dict(
            schema=1,
            session_id=str(uuid.uuid4()),
            created=time.time(),
            frame=frame,
            origin=asdict(origin),
            points={},
        )
        with path.open('x') as stream:
            json.dump(self.data, stream, indent=2, allow_nan=False)
        self.origin = origin

    def record(self, name, pose, stamp, observation):
        if name in self.data['points']:
            raise ValueError('Name already recorded; use a new name to preserve evidence')
        if name == 'P01' and not self.data['points']:
            self.origin = pose
            self.data['origin'] = asdict(pose)
        row = dict(
            pose=asdict(pose),
            relative_to_p01=local_delta(self.origin, pose),
            odom_stamp_ns=stamp,
            recorded=time.time(),
            laser=observation,
        )
        self.data['points'][name] = row
        temporary = self.path.with_suffix(self.path.suffix + '.tmp')
        try:
            temporary.write_text(json.dumps(self.data, indent=2, allow_nan=False) + '\n')
            temporary.replace(self.path)
        except OSError:
            del self.data['points'][name]
            raise
        return row

    def delta(self, first, second):
        from action_plan import Pose

        points = self.data['points']
        if first not in points or second not in points:
            raise ValueError('Unknown point name')
        a, b = points[first], points[second]
        return dict(
            start_body=local_delta(Pose(**a['pose']), Pose(**b['pose'])),
            p01_axes=dict(
                x_m=b['relative_to_p01']['x_m'] - a['relative_to_p01']['x_m'],
                y_m=b['relative_to_p01']['y_m'] - a['relative_to_p01']['y_m'],
            ),
            odom_axes=dict(
                x_m=b['pose']['x'] - a['pose']['x'], y_m=b['pose']['y'] - a['pose']['y']
            ),
        )


class Console:
    """Single owner of velocity output; motion is bounded and cancelable."""

    def __init__(self, runner, book, inputs, audit, turn_speed):
        self.runner, self.book, self.inputs = runner, book, inputs
        self.audit, self.turn_speed, self.quitting = audit, turn_speed, False

    def emit(self, event, **data):
        self.audit.write(
            json.dumps(dict(time=time.time(), event=event, **data), allow_nan=False) + '\n'
        )
        self.audit.flush()

    def cancel_input(self):
        present, line = self.inputs.poll()
        while present:
            if line is None or line in ('stop', 'quit'):
                self.quitting = line is None or line == 'quit'
                raise KeyboardInterrupt
            print('BUSY: command discarded; use stop or quit on its own line.', flush=True)
            present, line = self.inputs.poll()

    def discard_pending(self):
        present, line = self.inputs.poll()
        while present:
            if line is None or line == 'quit':
                self.quitting = True
            present, line = self.inputs.poll()

    def measure(self):
        """Reuse labelled laser ranges and provisional body model; no motion guard calls."""
        self.runner.observe('console_log_only')
        observation = self.runner.observations[-1]
        self.emit('measure', observation=observation)
        return observation

    def record(self, name):
        r = self.runner
        # Validate stopped feedback on both sides of the bounded laser observation.
        before = r.pose()
        observation = self.measure()
        after = r.pose()
        if (
            math.hypot(after.x - before.x, after.y - before.y) > 0.005
            or abs(after.yaw - before.yaw) > 0.01
        ):
            raise RuntimeError('Pose changed during record; stop then retry')
        row = self.book.record(name, after, r.stamp, observation)
        self.emit('record', name=name, point=row)
        print('Recorded:', name, 'P01-relative:', row['relative_to_p01'], flush=True)

    def move(self, motion):
        r = self.runner
        r.exclusive()
        start = r.pose()
        if motion.kind == 'path':
            delta = local_delta(self.book.origin, start)
            if (
                math.hypot(delta['x_m'] - motion.points[0][0], delta['y_m'] - motion.points[0][1])
                > 0.01
                or abs(delta['yaw_deg']) > 1
            ):
                raise RuntimeError(
                    'Path requires current pose within 1 cm / 1 deg of fixed segment start'
                )
            start = self.book.origin
        if r.frame != 'odom' or r.twist_frame != 'base_link':
            raise RuntimeError('Requires odom frame and base_link child frame')
        self.cancel_input()
        target = target_pose(start, motion)
        self.emit('started', motion=asdict(motion), start=asdict(start), target=asdict(target))
        length = math.hypot(motion.x, motion.y)
        budget = 30 + 3 * (length / r.max_speed + abs(motion.angle) / self.turn_speed)
        r.velocity_pub = r.node.create_publisher(r.twist, '/cmd_vel', 10)
        try:
            self.follow(start, motion, budget)
        finally:
            self.stop_motion()
        actual = r.pose()
        ready = (
            path_ready(start, actual, motion.points)
            if motion.kind == 'path'
            else feedback(start, actual, motion, r.max_speed, self.turn_speed)[1]
        )
        if not ready:
            raise RuntimeError('Final stopped pose outside enabled tolerances')
        self.emit('completed', actual=asdict(actual), displacement=local_delta(start, actual))
        print('Reached (odom estimate):', actual, flush=True)
        self.measure()

    def stop_motion(self):
        """Release the publisher after zeros without clearing a pending cancel signal."""
        r = self.runner
        if r.velocity_pub is None:
            return
        try:
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                r.send((0.0, 0.0, 0.0))
                r.ros.spin_once(r.node, timeout_sec=0.05)
        finally:
            r.node.destroy_publisher(r.velocity_pub)
            r.velocity_pub = None

    def follow(self, start, motion, budget):
        r = self.runner
        deadline, previous = time.monotonic() + budget, time.monotonic()
        command, held, logged = (0.0, 0.0, 0.0), None, 0.0
        tracker = PathTracker(start, motion.points) if motion.kind == 'path' else None
        heading_pid = HeadingPid()
        while time.monotonic() < deadline:
            r.pump()
            self.cancel_input()
            actual = r.moving_pose()
            if r.twist_frame != 'base_link':
                raise RuntimeError('Odom child frame changed')
            if len(r.node.get_publishers_info_by_topic('/cmd_vel')) > 1:
                raise RuntimeError('Competing cmd_vel publisher; stop the other controller')
            now = time.monotonic()
            dt = now - previous
            requested, ready = (
                tracker.feedback(actual, r.max_speed, self.turn_speed, dt, r.body_velocity[2])
                if tracker
                else feedback(
                    start,
                    actual,
                    motion,
                    r.max_speed,
                    self.turn_speed,
                    heading_pid,
                    dt,
                    r.body_velocity[2],
                )
            )
            command = limited_velocity(command, requested, dt, linear=0.4, angular=0.6)
            previous = now
            if ready:
                heading_pid.reset()
                command = (0.0, 0.0, 0.0)
            r.send(command)
            if ready and r.latest[2] < 0.005 and r.latest[3] < 0.015:
                held = held or now
                if now - held >= (1.0 if tracker else 0.5):
                    return
            else:
                held = None
            if now - logged >= 1:
                self.emit('progress', actual=asdict(actual), command=command, ready=ready)
                print('Progress:', local_delta(start, actual), flush=True)
                logged = now
        raise RuntimeError('Motion timeout; remaining batch canceled')

    def execute(self, command):
        if isinstance(command, Motion):
            return self.move(command)
        name, *args = command
        if name in ('path', 'path_plan'):
            return self.path_command(name, args[0] if args else 'P02')
        if name == 'record':
            self.record(args[0])
        elif name == 'delta':
            print(json.dumps(self.book.delta(*args), indent=2), flush=True)
        elif name == 'points':
            print(json.dumps(self.book.data['points'], indent=2), flush=True)
        elif name == 'measure':
            self.measure()
        elif name == 'status':
            print('Actual:', self.runner.pose(), 'file:', self.book.path, flush=True)
        elif name in ('stop', 'quit'):
            self.quitting = name == 'quit'
            raise KeyboardInterrupt
        else:
            print(HELP, flush=True)

    def path_command(self, name, destination='P02'):
        if destination == 'P03_turn':
            return self.p03_route(name)

        motion = load_p02()
        print(
            'P01 path:',
            len(motion.points),
            'points; max spacing 0.05 m; hold P01 yaw;',
            'endpoint=',
            (motion.x, motion.y),
            flush=True,
        )
        self.emit(
            'path_plan',
            origin=asdict(self.book.origin),
            points=motion.points,
            heading='hold_p01',
            target=asdict(target_pose(self.book.origin, motion)),
        )
        if name == 'path_plan':
            return
        if 'P02' in self.book.data['points']:
            raise ValueError(
                'P02 already recorded; use a fresh initialized session for another trial'
            )
        self.move(motion)
        self.record('P02')
        print('P02 comparison:', json.dumps(self.book.delta('P01', 'P02')), flush=True)

    def p03_route(self, name):
        """Stop at P02, follow fixed P01-frame interpolation, then turn to P01 yaw-90."""
        second = load_p03()
        print(
            'P01 -> P02 -> P03 -> right 90; P02-P03:',
            len(second.points),
            'points; endpoint=',
            (second.x, second.y),
            'heading: hold P01; final yaw: P01 - 90 deg',
            flush=True,
        )
        self.emit(
            'p03_route_plan', points=second.points, final_yaw=self.book.origin.yaw - math.pi / 2
        )
        if name == 'path_plan':
            return
        if any(label in self.book.data['points'] for label in ('P02', 'P03', 'P03_after_turn')):
            raise ValueError('Route already recorded; start a fresh initialized P01 session')
        self.path_command('path', 'P02')
        self.move(second)
        self.record('P03')
        actual = self.runner.pose()
        self.move(Motion('turn', angle=self.book.origin.yaw - math.pi / 2 - actual.yaw))
        self.record('P03_after_turn')
        print('P03 route completed; stopped', flush=True)

    def run(self):
        print(HELP, '\nmotion> ', end='', flush=True)
        while not self.quitting and not getattr(self.runner, 'terminate_requested', False):
            try:
                self.runner.pump()  # Track odom resets even while the prompt is idle.
                present, line = self.inputs.poll()
                if not present:
                    continue
                if line is None:
                    return
                commands = parse_batch(line)
                for command in commands:
                    self.execute(command)
                print('motion> ', end='', flush=True)
            except (RuntimeError, ValueError, OSError, KeyboardInterrupt) as error:
                self.stop_motion()
                self.runner.cancel_requested = False
                self.discard_pending()
                self.emit('stopped', reason=str(error) or 'Operator stop')
                print('STOPPED:', str(error) or 'Operator stop', '\nmotion> ', end='', flush=True)
                # A latched odom fault cannot be cleared by another movement command.
                if self.runner.invalid or getattr(self.runner, 'terminate_requested', False):
                    self.quitting = True


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--laser-log-only', action='store_true', required=True)
    parser.add_argument('--speed', type=float, default=0.04)
    parser.add_argument('--turn-speed', type=float, default=0.10)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.speed) or not 0.01 <= args.speed <= 0.12:
        parser.error('speed must be .01-.12 m/s')
    if not math.isfinite(args.turn_speed) or not 0.02 <= args.turn_speed <= 0.60:
        parser.error('turn-speed must be .02-.60 rad/s')
    return args


def main():
    args = arguments()
    path = (args.output or Path(f'motion_points_{time.time_ns()}.json')).resolve()
    if path.suffix != '.json':
        raise ValueError('Output must end with .json; audit uses the matching .jsonl')
    # Check output paths before connecting to ROS; exclusive creation also prevents overwrite.
    if path.exists() or path.with_suffix('.jsonl').exists():
        raise ValueError('Output already exists; choose a new session file')
    with open('/tmp/distance_controller_survey_return.lock', 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        r = WallRunner(args.speed)

        def request_stop(signum, _frame):
            r.terminate_requested = signum == signal.SIGTERM
            r.request_cancel()

        handlers = {
            sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)
        }
        try:
            r.exclusive()
            origin = r.pose()
            if r.frame != 'odom' or r.twist_frame != 'base_link':
                raise RuntimeError('Requires odom frame and base_link child frame')
            book = TeachingBook(path, origin, r.frame)
            with path.with_suffix('.jsonl').open('x') as audit:
                console = Console(r, book, ConsoleInput(), audit, args.turn_speed)
                console.record('P01')
                print('Current stopped pose is P01; no automatic initialization. File:', path)
                console.run()
        finally:
            r.stop_translation()
            r.close(False)
            for sig, handler in handlers.items():
                signal.signal(sig, handler)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, OSError, KeyboardInterrupt) as error:
        print('STOPPED:', error)
        raise SystemExit(2) from error
