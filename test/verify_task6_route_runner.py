#!/usr/bin/env python3
"""Localhost-only real-node fixture for preparation, route completion and fault stopping.

Run after sourcing Humble and the local workspace. Uses domain 189 only, a
raycast room, synthetic odometry, and the installed noninteractive executable.
"""

import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

os.environ['ROS_DOMAIN_ID'] = '189'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ.pop('CYCLONEDDS_URI', None)
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class Fixture:
    def __init__(self, directory, mode):
        self.directory, self.mode = directory, mode
        directory.mkdir()
        self.node = rclpy.create_node('task6_route_fixture')
        self.odom = self.node.create_publisher(Odometry, '/odometry/filtered', 10)
        self.scan = self.node.create_publisher(LaserScan, '/scan_filtered', 10)
        self.sub = self.node.create_subscription(Twist, '/cmd_vel', self.receive, 10)
        self.pose, self.velocity = [0.0, 0.0, 0.0], (0.0, 0.0, 0.0)
        self.received, self.last_scan, self.previous = 0.0, 0.0, time.monotonic()
        self.feedback, self.injected, self.commands = True, False, []
        self.broadcast = StaticTransformBroadcaster(self.node)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'base_link', 'laser'
        transform.transform.translation.x, transform.transform.translation.z = 0.02, 0.17345
        transform.transform.rotation.z, transform.transform.rotation.w = 1.0, 0.0
        self.broadcast.sendTransform(transform)
        self.timer = self.node.create_timer(0.02, self.tick)
        self.path = directory / 'points.json'
        self.log = directory / 'runtime.log'
        self.output = self.log.open('w')
        source = directory / 'route.json'
        source.write_text(
            json.dumps(
                dict(
                    schema=1,
                    scene=2,
                    frame='P01',
                    actions=[
                        dict(
                            name='P02',
                            kind='path',
                            heading_rad=0.0,
                            points=[[0.0, 0.0], [0.04, 0.0]],
                        ),
                        dict(name='P02_after_turn', kind='turn', heading_rad=0.15),
                        dict(
                            name='P03',
                            kind='path',
                            heading_rad=0.15,
                            points=[[0.04, 0.0], [0.045, 0.015], [0.04, 0.03]],
                        ),
                    ],
                )
            )
        )
        self.child = self.launch(source)

    def launch(self, source):
        """Launch the installed default backend; specialised fixtures may inject a test adapter."""
        return subprocess.Popen(
            [
                'ros2',
                'run',
                'distance_controller',
                'task6_route_runner',
                '--scene',
                '2',
                '--route-json',
                str(source),
                '--laser-log-only',
                '--output',
                str(self.path),
            ],
            stdin=subprocess.DEVNULL,
            stdout=self.output,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    def receive(self, msg):
        self.velocity = msg.linear.x, msg.linear.y, msg.angular.z
        self.received = time.monotonic()
        self.commands.append(self.velocity)

    def events(self):
        audit = self.path.with_suffix('.jsonl')
        if not audit.exists():
            return []
        return [json.loads(line) for line in audit.read_text().splitlines() if line]

    def inject(self):
        if self.injected:
            return
        events = self.events()
        started = [e for e in events if e['event'] == 'route_action_started']
        if self.mode == 'prepare_cancel' and any(
            e['event'] == 'preparation_started' for e in events
        ):
            os.killpg(self.child.pid, signal.SIGTERM)
        elif not started:
            return
        elif self.mode == 'stale' and self.velocity[0] > 0.01:
            self.feedback = False
        elif (
            self.mode == 'drift'
            and started[-1]['name'] == 'P02_after_turn'
            and self.velocity[2] > 0.01
        ):
            self.pose[0] += 0.045
        elif self.mode == 'cancel' and self.velocity[0] > 0.01:
            os.killpg(self.child.pid, signal.SIGINT)
        else:
            return
        self.injected = True

    def tick(self):
        now = time.monotonic()
        dt, self.previous = min(now - self.previous, 0.05), now
        vx, vy, wz = self.velocity if now - self.received < 0.3 else (0.0, 0.0, 0.0)
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        self.pose[0] += (c * vx - s * vy) * dt
        self.pose[1] += (s * vx + c * vy) * dt
        self.pose[2] += wz * dt
        self.inject()
        msg = Odometry()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.pose[:2]
        msg.pose.pose.orientation.z = math.sin(self.pose[2] / 2)
        msg.pose.pose.orientation.w = math.cos(self.pose[2] / 2)
        msg.twist.twist.linear.x, msg.twist.twist.linear.y = vx, vy
        msg.twist.twist.angular.z = wz
        if self.feedback:
            self.odom.publish(msg)
        if now - self.last_scan > 0.07:
            self.publish_scan(msg.header.stamp)
            self.last_scan = now

    def publish_scan(self, stamp):
        msg = LaserScan()
        msg.header.frame_id, msg.header.stamp = 'laser', stamp
        msg.angle_min, msg.angle_increment, msg.angle_max = -math.pi, math.pi / 360, math.pi
        msg.range_min, msg.range_max = 0.15, 40.0
        x = self.pose[0] + 0.02 * math.cos(self.pose[2])
        y = self.pose[1] + 0.02 * math.sin(self.pose[2])
        values = []
        for index in range(721):
            angle = msg.angle_min + index * msg.angle_increment + self.pose[2] + math.pi
            dx, dy = math.cos(angle), math.sin(angle)
            candidates = []
            if abs(dx) > 1e-8:
                candidates.append(((2.0 if dx > 0 else -0.28) - x) / dx)
            if abs(dy) > 1e-8:
                candidates.append(((0.6 if dy > 0 else -0.26) - y) / dy)
            distance = min(v for v in candidates if v > 0)
            values.append(distance if distance >= 0.15 else float('inf'))
        msg.ranges = values
        self.scan.publish(msg)

    def wait(self, condition, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if condition():
                return
        raise AssertionError('Timeout: ' + self.log.read_text()[-5000:])

    def released(self):
        with open('/tmp/distance_controller_survey_return.lock', 'a') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                return True
            except BlockingIOError:
                return False

    def verify(self):
        self.wait(lambda: self.child.poll() is not None, 100)
        self.wait(self.released, 10)
        events = self.events()
        if self.mode == 'success':
            assert self.child.returncode == 0, self.log.read_text()[-5000:]
            assert events[-1]['event'] == 'route_completed', events[-1]
            data = json.loads(self.path.read_text())
            assert list(data['points']) == ['P01', 'P02', 'P02_after_turn', 'P03']
            last = data['points']['P03']['relative_to_p01']
            assert math.hypot(last['x_m'] - 0.04, last['y_m'] - 0.03) < 0.005
            assert abs(math.radians(last['yaw_deg']) - 0.15) < 0.01
            assert any(e['event'] == 'preparation_handoff' for e in events)
            reconnect = next(e for e in events if e['event'] == 'post_turn_reconnect')
            assert reconnect['adjusted']['points'][1:] == reconnect['original']['points'][1:]
        else:
            assert self.injected, self.mode
            assert self.child.returncode != 0, self.mode
            assert events[-1]['event'] == 'route_failed', events[-1]
            assert not any(e['event'] == 'route_completed' for e in events)
            reasons = {
                'stale': 'Odom stale',
                'drift': '4 cm',
                'cancel': 'Cancelled',
                'prepare_cancel': 'Cancelled',
            }
            assert reasons[self.mode] in events[-1]['reason'], events[-1]
        if self.commands:
            assert sum(abs(v) for v in self.commands[-1]) == 0, self.commands[-1]
        self.wait(lambda: not self.node.get_publishers_info_by_topic('/cmd_vel'), 5)
        print(self.mode, 'PASS', 'exit', self.child.returncode, 'log', self.log, flush=True)

    def close(self):
        if self.child.poll() is None:
            os.killpg(self.child.pid, signal.SIGINT)
            self.wait(lambda: self.child.poll() is not None, 12)
        self.wait(self.released, 10)
        self.output.close()
        self.node.destroy_node()


def main():
    output = Path('/tmp/task6_route_test') / str(time.time_ns())
    output.mkdir(parents=True)
    rclpy.init()
    try:
        for mode in os.getenv(
            'TASK6_ROUTE_CASES', 'success,stale,drift,cancel,prepare_cancel'
        ).split(','):
            fixture = Fixture(output / mode, mode)
            try:
                fixture.verify()
            finally:
                fixture.close()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
