#!/usr/bin/env python3
"""Actual formal outbound process, then reverse runner in one continuous synthetic odom epoch."""

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from verify_task6_route_runner import Fixture, rclpy
from nav_msgs.msg import Odometry

os.environ['ROS_DOMAIN_ID'] = '193'
os.environ['ROS_LOCALHOST_ONLY'] = '1'


class ReturnFixture(Fixture):
    def __init__(self, directory, mode):
        self.phase = 'outward'
        super().__init__(directory, mode)
        self.sent = set()

    def start_return(self):
        self.wait(lambda: self.child.poll() is not None, 100)
        assert self.child.returncode == 0, self.log.read_text()[-4000:]
        self.wait(self.released, 10)
        self.wait(lambda: not self.node.get_publishers_info_by_topic('/cmd_vel'), 5)
        source = self.path
        data = json.loads(source.read_text())
        gid = bytes(
            self.node.get_publishers_info_by_topic('/odometry/filtered')[0].endpoint_gid
        ).hex()
        self.output.close()
        self.path = self.directory / 'return.json'
        self.log = self.directory / 'return.log'
        self.output = self.log.open('w')
        self.return_command = [
            'ros2',
            'run',
            'distance_controller',
            'route_return_runner',
            '--source',
            str(source),
            '--confirm-same-odom',
            data['session_id'],
            '--odom-publisher-gid',
            gid,
            '--output',
            str(self.path),
            '--laser-log-only',
        ]
        self.child = subprocess.Popen(
            self.return_command,
            stdin=subprocess.PIPE,
            text=True,
            stdout=self.output,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self.phase = 'return'

    def inject(self):
        if self.phase != 'return' or self.child.poll() is not None:
            return
        events = self.events()
        if not any(e['event'] == 'return_session_started' for e in events):
            return
        done = [e for e in events if e['event'] == 'return_action_completed']
        if len(done) < 3 and len(done) not in self.sent:
            self.child.stdin.write('next\n')
            self.child.stdin.flush()
            self.sent.add(len(done))
        started = [e for e in events if e['event'] == 'return_action_started']
        if not started or self.injected:
            return
        if self.mode == 'turn_drift' and started[-1]['kind'] == 'turn' and self.velocity[2] < -0.01:
            self.pose[0] += 0.045
        elif self.mode == 'stale' and math.hypot(*self.velocity[:2]) > 0.005:
            self.feedback = False
        elif self.mode == 'publisher_changed' and math.hypot(*self.velocity[:2]) > 0.005:
            self.node.destroy_publisher(self.odom)
            self.odom = self.node.create_publisher(Odometry, '/odometry/filtered', 10)
        elif self.mode == 'cancel' and math.hypot(*self.velocity[:2]) > 0.005:
            os.killpg(self.child.pid, signal.SIGINT)
        else:
            return
        self.injected = True

    def verify(self):
        self.start_return()
        self.wait(lambda: self.child.poll() is not None, 110)
        self.wait(self.released, 10)
        self.wait(lambda: not self.node.get_publishers_info_by_topic('/cmd_vel'), 5)
        events = self.events()
        assert self.commands and sum(abs(v) for v in self.commands[-1]) == 0
        assert not any(
            e['event'] in ('preparation_started', 'preparation_handoff', 'route_completed')
            for e in events
        )
        if self.mode == 'success':
            assert self.child.returncode == 0, self.log.read_text()[-5000:]
            assert events[-1]['event'] == 'return_completed', events[-1]
            completed = [e['name'] for e in events if e['event'] == 'return_action_completed']
            assert completed == ['P03', 'P02_after_turn', 'P02'], completed
            last = events[-1]
            assert (
                math.hypot(
                    last['actual']['x'] - last['original_p01']['x'],
                    last['actual']['y'] - last['original_p01']['y'],
                )
                <= 0.005
            )
        else:
            assert self.injected
            assert self.child.returncode != 0
            assert events[-1]['event'] == 'return_session_failed'
            assert not any(e['event'] == 'return_completed' for e in events)
            reason = {
                'turn_drift': '4 cm',
                'stale': 'Odom stale',
                'publisher_changed': 'publisher identity',
                'cancel': 'Cancelled',
            }[self.mode]
            assert reason in events[-1]['reason'], events[-1]
        retry = subprocess.run(self.return_command, capture_output=True, text=True, timeout=5)
        assert retry.returncode == 2 and 'restart/resume forbidden' in retry.stdout, retry.stdout
        print(self.mode, 'PASS', 'exit', self.child.returncode, self.log, flush=True)


def main():
    output = Path('/tmp/route_return_ros') / str(time.time_ns())
    output.mkdir(parents=True)
    rclpy.init()
    try:
        for mode in os.getenv(
            'ROUTE_RETURN_CASES', 'success,turn_drift,stale,publisher_changed,cancel'
        ).split(','):
            fixture = ReturnFixture(output / mode, mode)
            try:
                fixture.verify()
            finally:
                fixture.close()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
