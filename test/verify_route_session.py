#!/usr/bin/env python3
"""Isolated installed frontend -> supervised backend -> synthetic sensor tests."""

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import yaml
from verify_task6_route_runner import Fixture, rclpy

os.environ['ROS_DOMAIN_ID'] = '192'
os.environ['ROS_LOCALHOST_ONLY'] = '1'


class SupervisedFixture(Fixture):
    def launch(self, source):
        from ament_index_python.packages import get_package_share_directory

        template = (
            Path(get_package_share_directory('pid_maze_solver')) / 'waypoints/waypoints_real.yaml'
        )
        route = yaml.safe_load(template.read_text())
        position, heading = [0.0, 0.0], 0.0
        for action in route['actions']:
            if action['kind'] == 'turn':
                heading = 0.15
                action['heading_rad'] = heading
            else:
                action['heading_rad'] = heading
                action['points'] = [position[:], [position[0] + 0.04, position[1]]]
                position = action['points'][-1][:]
        route['validated'], route['test_until'] = False, 'P12'
        scene = self.directory / 'scene.yaml'
        scene.write_text(yaml.safe_dump(route, sort_keys=False))
        self.sent, self.drop_scan = set(), False
        destination = 'P03_turn' if self.mode == 'turn_boundary' else 'P02'
        return subprocess.Popen(
            [
                'ros2',
                'run',
                'pid_maze_solver',
                'pid_maze_solver',
                '2',
                '--supervised',
                '--waypoints',
                str(scene),
                '--destination',
                destination,
                '--output',
                str(self.path),
            ],
            stdin=subprocess.PIPE,
            text=True,
            stdout=self.output,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    def send(self, key, command):
        if key not in self.sent:
            self.child.stdin.write(command + '\n')
            self.child.stdin.flush()
            self.sent.add(key)

    def inject(self):  # noqa: PLR0912 - explicit five-case fixture state transitions
        if self.child.poll() is not None:
            return
        events = self.events()
        names = [e['event'] for e in events]
        completed = [e for e in events if e['event'] == 'supervised_action_completed']
        if 'supervised_session_started' in names:
            self.send('next1', 'next')
        if self.mode == 'turn_boundary':
            if len(completed) in (1, 2):
                self.send('next' + str(len(completed) + 1), 'next')
            if len(completed) == 3:
                self.send('preview', 'back_plan')
            if 'supervised_command_rejected' in names:
                self.send('quit', 'quit')
            return
        if completed:
            self.send('preview', 'back_plan')
        plans = [e for e in events if e['event'] == 'supervised_return_plan']
        if plans:
            self.send('back', 'back ' + plans[-1]['plan']['token'])
        if 'supervised_return_started' in names and not self.injected:
            if self.mode == 'return_stale' and self.velocity[0] < -0.001:
                self.feedback, self.injected = False, True
            elif self.mode == 'return_scan_stale':
                self.drop_scan, self.injected = True, True
            elif self.mode == 'return_cancel' and self.velocity[0] < -0.001:
                os.killpg(self.child.pid, signal.SIGINT)
                self.injected = True
        if 'supervised_return_completed' in names:
            self.send('quit', 'quit')

    def publish_scan(self, stamp):
        if not self.drop_scan:
            super().publish_scan(stamp)

    def verify(self):
        self.wait(lambda: self.child.poll() is not None, 110)
        self.wait(self.released, 10)
        events = self.events()
        names = [e['event'] for e in events]
        assert 'route_completed' not in names
        assert self.commands and sum(abs(v) for v in self.commands[-1]) == 0
        self.wait(lambda: not self.node.get_publishers_info_by_topic('/cmd_vel'), 5)
        if self.mode == 'success':
            assert self.child.returncode == 0, self.log.read_text()[-5000:]
            returned = next(e for e in events if e['event'] == 'supervised_return_completed')
            p01 = json.loads(self.path.read_text())['origin']
            actual = returned['actual']
            assert math.hypot(actual['x'] - p01['x'], actual['y'] - p01['y']) <= 0.005
            assert events[-2]['cursor'] == 0
        elif self.mode == 'turn_boundary':
            assert self.child.returncode == 0, self.log.read_text()[-5000:]
            rejected = next(e for e in events if e['event'] == 'supervised_command_rejected')
            assert 'across a turn' in rejected['reason']
            assert 'supervised_return_started' not in names
            assert events[-2]['cursor'] == 3
        else:
            assert self.injected, self.mode
            assert self.child.returncode != 0, self.log.read_text()[-5000:]
            assert 'supervised_return_completed' not in names
            assert events[-1]['event'] == 'route_failed'
            assert 'supervised_session_failed' in names
        guard = [e for e in events if e['event'] == 'laser_guard']
        assert guard and all(e['enforced'] for e in guard)
        print(self.mode, 'PASS', 'exit', self.child.returncode, self.log, flush=True)


def main():
    output = Path('/tmp/route_session_ros') / str(time.time_ns())
    output.mkdir(parents=True)
    rclpy.init()
    try:
        for mode in os.getenv(
            'ROUTE_SESSION_CASES',
            'success,return_stale,return_scan_stale,return_cancel,turn_boundary',
        ).split(','):
            fixture = SupervisedFixture(output / mode, mode)
            try:
                fixture.verify()
            finally:
                fixture.close()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
