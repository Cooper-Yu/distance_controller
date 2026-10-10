#!/usr/bin/env python3
"""Inspect or execute ONE supervised reverse-survey action; never auto-advance."""

import argparse
from dataclasses import dataclass, replace
import json
import math
from pathlib import Path
import signal
import subprocess
import time


@dataclass(frozen=True)
class Action:
    """A relative action: meters for translation, radians for turn."""

    place: str
    kind: str
    value: float
    provisional: bool = False


# Consolidated commanded distances, NOT a concatenation of reset odom coordinates.
OUTWARD = (
    Action('P01->P02', 'forward', 0.90),
    Action('P02->P03', 'forward', 0.82),
    Action('P03', 'turn', -math.pi / 2),
    Action('P03->P04', 'forward', 0.45),
    Action('P04->P05', 'right', 0.27),
    Action('P05->P06', 'forward', 0.52, True),
    Action('P06->P07', 'left', 0.40, True),
    Action('P07->P08', 'forward', 0.50, True),
    Action('P08->P09', 'right', 0.50, True),
    Action('P09->P10', 'forward', 0.40),
    Action('P10', 'turn', -math.pi / 2),
    Action('P10->P11', 'forward', 0.40),
    Action('P11', 'turn', -math.pi / 2),
    Action('P11->P12', 'forward', 0.80),
    Action('P12', 'turn', math.pi / 2),
    Action('P12->P13', 'forward', 0.55),
    Action('P13', 'turn', -math.pi / 2),
    Action('P13->P14', 'forward', 0.48),
    Action('P14', 'turn', math.pi / 2),
    Action('P14->P15', 'forward', 0.40),
    Action('P15', 'turn', math.pi),
)


def reverse_route(actions):
    """Invert each relative action and its order, including the final half-turn."""
    opposite = {'forward': 'backward', 'backward': 'forward', 'left': 'right', 'right': 'left'}
    result = []
    for action in reversed(actions):
        place = '->'.join(reversed(action.place.split('->')))
        kind = 'turn' if action.kind == 'turn' else opposite[action.kind]
        value = -action.value if kind == 'turn' else action.value
        result.append(replace(action, place=place, kind=kind, value=value))
    return result


def describe(index, action):
    """Return a reviewable command summary without ROS or motion."""
    unit = 'rad' if action.kind == 'turn' else 'm'
    warning = ' [MERGED CORRECTION: NOT REPLAY-VERIFIED]' if action.provisional else ''
    return f'{index:02d}: {action.place} {action.kind} {action.value:.6f} {unit}{warning}'


def parse_status(message):
    """Read complete status fields, avoiding substring matches such as pending=10."""
    fields = dict(item.split('=', 1) for item in message.split() if '=' in item)
    return fields


def idle(message, label=None):
    """Completion requires stopped WAITING, no queued/return work, and expected label."""
    fields = parse_status(message)
    return (
        fields.get('state') == 'WAITING'
        and fields.get('pending') == '0'
        and fields.get('return_remaining') == '0'
        and (label is None or fields.get('waypoint') == label)
    )


class Runner:
    """Own one controller process and stop it on failure; never restore old history."""

    def __init__(self, handle_signals=True):
        import rclpy
        from distance_controller.srv import ExecuteStep
        from geometry_msgs.msg import Twist

        self.ros = rclpy
        self.service = ExecuteStep
        self.twist = Twist
        from rclpy.signals import SignalHandlerOptions

        rclpy.init(signal_handler_options=None if handle_signals else SignalHandlerOptions.NO)
        self.node = rclpy.create_node('supervised_return')
        self.client = self.node.create_client(ExecuteStep, '/distance_controller/step')
        self.child = None
        self.stop_pub = None
        from survey_distances import DistanceObserver

        self.observer = DistanceObserver(self.node)
        self.observations = []

    def observe(self, phase):
        """Save and display a new four-direction scan, or its explicit unavailable reason."""
        from survey_distances import print_snapshot

        result = self.observer.snapshot(phase)
        self.observations.append(result)
        print_snapshot(result)

    def exclusive(self):
        """Reject existing command publishers/controllers before owning any process."""
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            self.ros.spin_once(self.node, timeout_sec=0.1)
        if self.node.get_publishers_info_by_topic('/cmd_vel') or self.client.service_is_ready():
            raise RuntimeError('Existing /cmd_vel publisher or step service: stop it first.')

    def start(self, args):
        """Start an owned process group, keeping its runtime log visible."""
        self.child = subprocess.Popen(args, start_new_session=True)

    def call(self, action, **values):
        """Send once; ambiguous acceptance raises rather than retrying a motion."""
        if not self.client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('Step service unavailable')
        request = self.service.Request()
        request.action = action
        for key, value in values.items():
            setattr(request, key, value)
        future = self.client.call_async(request)
        self.ros.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        if not future.done():
            raise RuntimeError('Acceptance unknown: stop and inspect before any retry')
        response = future.result()
        if not response.accepted:
            raise RuntimeError(response.message)
        return response.message

    def wait_idle(self, timeout, label=None):
        """Check endpoint completion, child exit and faults within a wall-time bound."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.child.poll() is not None:
                raise RuntimeError(f'Controller exited early: {self.child.returncode}')
            if self.client.service_is_ready():
                message = self.call('status')
                if parse_status(message).get('state') == 'FAULT':
                    raise RuntimeError(message)
                if idle(message, label):
                    return message
            self.ros.spin_once(self.node, timeout_sec=0.2)
        raise RuntimeError('Timed out waiting for a stopped endpoint')

    def move(self, action, index):
        """Adopt the current pose, perform one bounded translation, finish and exit."""
        self.start(
            [
                'ros2',
                'run',
                'distance_controller',
                'distance_controller',
                '2',
                '--ros-args',
                '-p',
                'manual_mode:=true',
                '-p',
                'adopt_current_pose:=true',
                '-p',
                'max_speed:=0.03',
            ]
        )
        self.wait_idle(15.0)
        label = f'return_{index:02d}'
        budget = action.value / 0.03 * 3 + 20
        print(
            self.call(
                action.kind,
                distance=action.value,
                speed=0.03,
                frame='heading',
                label=label,
                dwell=1.0,
                timeout=budget,
            ),
            flush=True,
        )
        result = self.wait_idle(budget + 5, label)
        self.call('finish')
        if self.child.wait(timeout=10) != 0:
            raise RuntimeError('Translation controller exited with failure')
        return result

    def turn(self, action):
        """Run one relative turn without preparation; require successful process exit."""
        self.start(
            [
                'ros2',
                'run',
                'turn_controller',
                'turn_controller',
                '2',
                '--skip-preparation',
                '--ros-args',
                '-p',
                f'turn_angles:=[{action.value}]',
                '-p',
                'max_angular_speed:=0.20',
            ]
        )
        if self.child.wait(timeout=60) != 0:
            raise RuntimeError('Turn controller exited with failure')
        return 'Turn controller reported successful exit; inspect pose and clearance.'

    def close(self, failed):
        """On interruption/failure, send zeros before shutting down our process group."""
        import os

        try:
            if failed and self.child is not None:
                self.stop_pub = self.node.create_publisher(self.twist, '/cmd_vel', 10)
                end = time.monotonic() + 1.0
                # Stop owned controller first so it cannot overwrite our zero commands.
                if self.child.poll() is None:
                    os.killpg(self.child.pid, signal.SIGINT)
                while time.monotonic() < end:
                    self.stop_pub.publish(self.twist())
                    self.ros.spin_once(self.node, timeout_sec=0.05)
                if self.child.poll() is None:
                    try:
                        self.child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(self.child.pid, signal.SIGKILL)
                        self.child.wait(timeout=5)
        finally:
            self.node.destroy_node()
            self.ros.shutdown()


def measure_only():
    """Read and persist one observation without starting a controller or publishing velocity."""
    runner = Runner()
    try:
        runner.observe('read_only')
        path = Path(f'laser_observation_{time.time_ns()}.json')
        path.write_text(json.dumps(runner.observations, indent=2) + '\n')
        print(f'Observation: {path.resolve()}')
    finally:
        runner.close(False)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--step', type=int, help='One-based action number; no automatic next action'
    )
    parser.add_argument(
        '--execute', action='store_true', help='Enable one action after typed confirmation'
    )
    parser.add_argument(
        '--distance', type=float, help='Override this translation distance in meters'
    )
    parser.add_argument(
        '--measure', action='store_true', help='Read four directions without motion'
    )
    args = parser.parse_args()
    if args.measure:
        if args.execute or args.step is not None or args.distance is not None:
            parser.error('--measure cannot be combined with motion options')
        return measure_only()
    route = reverse_route(OUTWARD)
    if args.step is None:
        if args.execute or args.distance is not None:
            parser.error('--step is required')
        for index, action in enumerate(route, 1):
            print(describe(index, action))
        return 0
    if not 1 <= args.step <= len(route):
        parser.error('Step out of range')
    action = route[args.step - 1]
    if args.distance is not None:
        if action.kind == 'turn' or not math.isfinite(args.distance) or not 0 < args.distance <= 1:
            parser.error('--distance requires a translation and 0 < meters <= 1')
        action = replace(action, value=args.distance)
    print(describe(args.step, action), flush=True)
    if not args.execute:
        return 0
    prompt = f'Confirm physical start pose and clear swept path. Type RUN {args.step}: '
    if input(prompt).strip() != f'RUN {args.step}':
        print('Cancelled; no controller started.')
        return 1
    # A separate process must not overlap a second instance of this helper.
    import fcntl

    with open('/tmp/distance_controller_survey_return.lock', 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        runner = Runner()
        failed = True
        try:
            runner.exclusive()
            runner.observe('before')
            result = (
                runner.turn(action) if action.kind == 'turn' else runner.move(action, args.step)
            )
            print(result, flush=True)
            runner.observe('after')
            failed = False
        finally:
            runner.close(failed)
            audit = {
                'step': args.step,
                'action': action.__dict__,
                'success': not failed,
                'laser_observations': runner.observations,
                'time': time.time(),
                'note': 'Operator must verify location before next step.',
            }
            path = Path(f'return_step_{args.step:02d}_{time.time_ns()}.json')
            path.write_text(json.dumps(audit, indent=2) + '\n')
            print(f'Audit: {path.resolve()}')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, subprocess.TimeoutExpired, KeyboardInterrupt, EOFError) as error:
        print(f'STOPPED: {error}')
        raise SystemExit(2) from error
