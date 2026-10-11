#!/usr/bin/env python3
"""Isolated ROS synthetic-plant check; never run on a hardware ROS domain.

Run under ROS_LOCALHOST_ONLY=1 and a dedicated ROS_DOMAIN_ID. Starts the installed
console, feeds odometry, and checks command output, file data and fault stopping.
"""

import json
import fcntl
import math
import os
from pathlib import Path
import subprocess
import signal
import tempfile
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


class Fixture:
    def __init__(self, directory):
        self.node = rclpy.create_node('motion_console_fixture')
        self.pub = self.node.create_publisher(
            Odometry, '/odometry/filtered', qos_profile_sensor_data
        )
        self.sub = self.node.create_subscription(Twist, '/cmd_vel', self.receive, 10)
        self.x, self.y, self.yaw = 1.0, 2.0, math.pi / 2
        self.velocity, self.received, self.commands = (0.0, 0.0, 0.0), 0.0, []
        self.feedback_enabled, self.freeze, self.jump = True, False, False
        self.previous = time.monotonic()
        self.timer = self.node.create_timer(0.02, self.tick)
        self.path = Path(directory) / 'points.json'
        self.output = (Path(directory) / 'console.log').open('w+')
        self.child = subprocess.Popen(
            [
                'ros2',
                'run',
                'distance_controller',
                'motion_console',
                '--laser-log-only',
                '--speed',
                os.getenv('MOTION_TEST_SPEED', '0.04'),
                '--output',
                str(self.path),
            ],
            start_new_session=True,
            stdin=subprocess.PIPE,
            stdout=self.output,
            stderr=subprocess.STDOUT,
            text=True,
        )

    def receive(self, msg):
        self.velocity = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.received = time.monotonic()
        self.commands.append(self.velocity)

    def tick(self):
        now = time.monotonic()
        dt, self.previous = now - self.previous, now
        vx, vy, wz = self.velocity if now - self.received < 0.3 else (0, 0, 0)
        if not self.freeze:
            self.x += (math.cos(self.yaw) * vx - math.sin(self.yaw) * vy) * dt
            self.y += (math.sin(self.yaw) * vx + math.cos(self.yaw) * vy) * dt
            self.yaw += wz * dt
        if not self.feedback_enabled:
            return
        msg = Odometry()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
        msg.pose.pose.position.x = self.x + (1 if self.jump else 0)
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2)
        msg.twist.twist.linear.x, msg.twist.twist.linear.y = float(vx), float(vy)
        msg.twist.twist.angular.z = float(wz)
        self.pub.publish(msg)

    def text(self):
        self.output.flush()
        return self.path.with_name('console.log').read_text()

    def wait(self, condition, seconds=25):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if condition():
                return
        raise AssertionError('Timeout\n' + self.text()[-4000:])

    def send(self, line):
        self.child.stdin.write(line + '\n')
        self.child.stdin.flush()

    def close(self):
        if self.child.poll() is None:
            self.child.stdin.close()
            try:
                self.wait(lambda: self.child.poll() is not None, 5)
            except AssertionError:
                self.child.kill()
                self.child.wait(timeout=5)

        # ros2 run may exit before its child finishes the zero-command cleanup.
        def released():
            with open('/tmp/distance_controller_survey_return.lock', 'a') as lock:
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    return True
                except BlockingIOError:
                    return False

        self.wait(released, 5)
        self.output.close()
        self.node.destroy_node()


def run(directory):
    fixture = Fixture(directory)
    try:
        fixture.wait(lambda: 'motion> ' in fixture.text())
        fixture.send('move xy 0.10 0.03 --heading on --lateral on; record P02; delta P01 P02')
        fixture.wait(lambda: 'start_body' in fixture.text(), 30)
        data = json.loads(fixture.path.read_text())
        result = data['points']['P02']['relative_to_p01']
        assert abs(result['x_m'] - 0.1) < 0.006 and abs(result['y_m'] - 0.03) < 0.006, result
        assert data['points']['P02']['laser']['status'] == 'no_scan'
        count = len(fixture.commands)
        fixture.send('move forward .1; move left nan')
        fixture.wait(lambda: 'Numbers must be finite' in fixture.text())
        assert len(fixture.commands) == count, 'Malformed batch moved robot'
        fixture.send('turn right 15; record TURN')
        fixture.wait(lambda: 'Recorded: TURN' in fixture.text(), 25)
        result = json.loads(fixture.path.read_text())['points']['TURN']['relative_to_p01']
        assert abs(result['yaw_deg'] + 15) < 1, result
        fixture.send('move forward 0.03; move left 0.03; record P03')
        fixture.wait(lambda: 'Recorded: P03' in fixture.text(), 30)
        fixture.send('move forward .5; record MUST_NOT_EXIST')
        fixture.wait(lambda: abs(fixture.velocity[0]) > 0.015)
        fixture.send('stop')
        fixture.wait(lambda: 'Operator stop' in fixture.text())
        assert fixture.velocity == (0.0, 0.0, 0.0)
        assert 'MUST_NOT_EXIST' not in json.loads(fixture.path.read_text())['points']
        fixture.send('move left .5')
        fixture.wait(lambda: abs(fixture.velocity[1]) > 0.015)
        fixture.feedback_enabled = False
        fixture.wait(lambda: 'Odom stale' in fixture.text())
        assert fixture.velocity == (0.0, 0.0, 0.0)
        fixture.feedback_enabled = True
        fixture.send('move forward .5')
        fixture.wait(lambda: abs(fixture.velocity[0]) > 0.015)
        fixture.jump = True
        fixture.wait(lambda: fixture.child.poll() is not None)
        assert 'Odom position jumped' in fixture.text()
        assert fixture.velocity == (0.0, 0.0, 0.0)
        print(
            'PASS: installed CLI, diagonal/rotated frame, turn, sequential batch, record/delta, '
            'missing scan, malformed batch, stop cancellation, stale odom, odom jump'
        )
    finally:
        fixture.close()


def cancel_case(directory, mode):
    fixture = Fixture(directory)
    competitor = None
    try:
        fixture.wait(lambda: 'motion> ' in fixture.text())
        fixture.send('move forward .5; record MUST_NOT_EXIST')
        fixture.wait(lambda: abs(fixture.velocity[0]) > 0.015)
        if mode == 'eof':
            fixture.child.stdin.close()
        elif mode == 'competitor':
            competitor = fixture.node.create_publisher(Twist, '/cmd_vel', 10)
        else:
            os.killpg(fixture.child.pid, signal.SIGTERM)
        marker = 'Competing cmd_vel' if competitor else None
        fixture.wait(
            lambda: (
                marker in fixture.text()
                if marker
                else fixture.child.poll() is not None and fixture.velocity == (0.0, 0.0, 0.0)
            ),
            8,
        )
        assert fixture.velocity == (0.0, 0.0, 0.0), (mode, fixture.velocity)
        assert 'MUST_NOT_EXIST' not in json.loads(fixture.path.read_text())['points']
        print('PASS:', mode, 'stopped; queued record canceled')
    finally:
        if competitor:
            fixture.node.destroy_publisher(competitor)
        fixture.close()


if __name__ == '__main__':
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert int(os.environ.get('ROS_DOMAIN_ID', '0')) >= 190, 'Use an isolated test domain'
    rclpy.init()
    try:
        with tempfile.TemporaryDirectory() as directory:
            run(directory)
        for mode in ('eof', 'sigterm', 'competitor'):
            with tempfile.TemporaryDirectory() as directory:
                cancel_case(directory, mode)
    finally:
        rclpy.shutdown()
