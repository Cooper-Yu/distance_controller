"""Isolated domain 168: manual continuation, feedback policies and bounded failure tests."""

import os

os.environ['ROS_DOMAIN_ID'] = '168'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import json
import math
import re
import signal
import subprocess
import time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from distance_controller.srv import ExecuteStep

ROOT = Path(os.environ.get('FIXTURE_OUTPUT_ROOT', '/tmp/distance_controller_fixture'))
OUT = (
    ROOT
    / 'runtime_logs'
    / (time.strftime('manual_steps_%Y%m%d_%H%M%S') + '_' + str(time.time_ns() % 1_000_000_000))
)
OUT.mkdir(parents=True)
EXE = (
    Path(
        subprocess.check_output(['ros2', 'pkg', 'prefix', 'distance_controller'], text=True).strip()
    )
    / 'lib/distance_controller/distance_controller'
)
rclpy.init()
RESULTS = []


class Plant:
    def __init__(self, case, extra=None):
        self.case = case
        self.node = rclpy.create_node('step_fixture_' + case)
        self.ns = '/step_fixture/' + case
        self.base, self.laser = case + '_base', case + '_laser'
        self.x, self.y, self.yaw = 0.0, -0.025, 0.10
        self.angle = 0.2
        self.cmd = [0.0, 0.0, 0.0]
        self.frozen = False
        self.scan_enabled = True
        self.odom_enabled = True
        self.odom_stamp_age = 0.0
        self.odom_frame_override = None
        self.feedback_speed_override = None
        self.front_bad = False
        self.side_bad = False
        self.prev = time.monotonic()
        self.next_scan = 0
        self.path = OUT / (case + '.log')
        self.log = self.path.open('w')
        self.sub = self.node.create_subscription(Twist, self.ns + '/cmd', self.receive, 10)
        self.odom_pub = self.node.create_publisher(Odometry, self.ns + '/odom', 10)
        self.scan_pub = self.node.create_publisher(
            LaserScan, self.ns + '/scan', qos_profile_sensor_data
        )
        self.tf = StaticTransformBroadcaster(self.node)
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = self.base, self.laser
        tf.transform.translation.x = 0.02
        tf.transform.translation.z = 0.173
        tf.transform.rotation.z = 1.0
        tf.transform.rotation.w = 0.0
        self.tf.sendTransform(tf)
        params = {
            'manual_mode': 'true',
            'route': '[AB]',
            'forward_distance': '0.08',
            'lateral_distance': '0.04',
            'heading_gain': '3.0',
            'max_yaw_rate': '0.5',
            'alignment_settle_duration': '0.1',
            'centering_gain': '2.0',
            'dwell_duration': '0.02',
            'front_body_extent': '0.20',
            'odom_topic': self.ns + '/odom',
            'cmd_vel_topic': self.ns + '/cmd',
            'scan_topic': self.ns + '/scan',
            'base_frame': self.base,
        }
        extra = dict(extra or {})
        params_file = extra.pop('_params_file', None)
        if params_file:
            params.pop('forward_distance')
            params.pop('lateral_distance')
        params.update(extra)
        args = [str(EXE), '2', '--ros-args', '-r', '__ns:=' + self.ns]
        if params_file:
            args.extend(['--params-file', str(params_file)])
        for key, value in params.items():
            args.extend(['-p', key + ':=' + value])
        self.proc = subprocess.Popen(args, stdout=self.log, stderr=subprocess.STDOUT)
        self.client = self.node.create_client(ExecuteStep, self.ns + '/distance_controller/step')

    def receive(self, msg):
        self.cmd = [msg.linear.x, msg.linear.y, msg.angular.z]
        assert all(math.isfinite(v) for v in self.cmd)
        assert math.hypot(*self.cmd[:2]) <= 0.100001 and abs(self.cmd[2]) <= 0.500001

    def text(self):
        return self.path.read_text()

    def pose(self):
        c, s = math.cos(self.angle), math.sin(self.angle)
        return c * self.x - s * self.y, s * self.x + c * self.y

    def tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.005)
        now = time.monotonic()
        dt = min(0.03, now - self.prev)
        self.prev = now
        vx, vy, wz = self.cmd if not self.frozen else (0.0, 0.0, 0.0)
        self.x += (math.cos(self.yaw) * vx - math.sin(self.yaw) * vy) * dt
        self.y += (math.sin(self.yaw) * vx + math.cos(self.yaw) * vy) * dt
        self.yaw += wz * dt
        if self.odom_enabled:
            odom = Odometry()
            odom.header.frame_id, odom.child_frame_id = 'odom', self.base
            stamp = self.node.get_clock().now().nanoseconds - int(self.odom_stamp_age * 1e9)
            odom.header.stamp.sec = stamp // 1_000_000_000
            odom.header.stamp.nanosec = stamp % 1_000_000_000
            if self.odom_frame_override:
                odom.header.frame_id = self.odom_frame_override
            odom.pose.pose.position.x, odom.pose.pose.position.y = self.pose()
            odom.pose.pose.orientation.z = math.sin((self.yaw + self.angle) / 2)
            odom.pose.pose.orientation.w = math.cos((self.yaw + self.angle) / 2)
            odom.twist.twist.linear.x, odom.twist.twist.linear.y = vx, vy
            odom.twist.twist.angular.z = wz
            if self.feedback_speed_override is not None:
                odom.twist.twist.linear.x = self.feedback_speed_override
            self.odom_pub.publish(odom)
        if self.scan_enabled and now >= self.next_scan:
            self.next_scan = now + 0.05
            self.publish_scan()

    def publish_scan(self):
        scan = LaserScan()
        scan.header.frame_id = self.laser
        scan.header.stamp = self.node.get_clock().now().to_msg()
        scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi, math.pi / 360
        scan.range_min, scan.range_max = 0.15, 40.0
        lx, ly = self.x + 0.02 * math.cos(self.yaw), self.y + 0.02 * math.sin(self.yaw)
        ranges = []
        for i in range(721):
            body = math.pi + scan.angle_min + i * scan.angle_increment
            dx, dy = math.cos(body + self.yaw), math.sin(body + self.yaw)
            hits = [40.0]
            if abs(dy) > 1e-8:
                hits.append(((0.30 if dy > 0 else -0.30) - ly) / dy)
            if dx < -1e-8:
                hits.append((-0.28 - lx) / dx)
            if dx > 1e-8:
                hits.append((0.65 - lx) / dx)
            distance = min(v for v in hits if v > 0)
            angle = math.atan2(math.sin(body), math.cos(body))
            if self.front_bad and abs(angle) < 0.10:
                distance = float('nan')
            if self.side_bad and abs(abs(angle) - math.pi / 2) < 0.20:
                distance = float('nan')
            ranges.append(distance if 0.15 <= distance <= 40 else float('inf'))
        scan.ranges = ranges
        self.scan_pub.publish(scan)

    def until(self, predicate, timeout=20):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.tick()
            if predicate():
                return
            if self.proc.poll() is not None:
                raise AssertionError((self.case, 'unexpected exit', self.text()[-1500:]))
        raise AssertionError((self.case, 'timeout', self.text()[-1500:]))

    def wait(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.tick()

    def call(self, action, accepted=True, **kwargs):
        self.until(self.client.service_is_ready, 5)
        req = ExecuteStep.Request()
        req.action, req.dwell = action, 0.02
        for key, value in kwargs.items():
            setattr(req, key, value)
        future = self.client.call_async(req)
        self.until(future.done, 5)
        response = future.result()
        assert response.accepted == accepted, (self.case, action, response.message)
        return response.message

    def cli_status(self, action='status', *values):
        command = subprocess.Popen(
            [
                'ros2',
                'run',
                'distance_controller',
                'step',
                action,
                *values,
                '--service',
                self.ns + '/distance_controller/step',
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        try:
            self.until(lambda: command.poll() is not None, 10)
            out, err = command.communicate()
            assert command.returncode == 0 and 'ACCEPTED:' in out, (out, err)
            if action == 'status':
                assert 'state=WAITING' in out
        finally:
            if command.poll() is None:
                command.kill()
                command.wait()

    def idle(self):
        self.until(lambda: 'Manual WAITING:' in self.text())
        self.wait(0.1)
        assert 'state=WAITING' in self.call('status')
        return float(re.search(r'heading_reference=([-\d.]+)', self.text())[1])

    def move(self, action, **kwargs):
        previous = self.text().count('Endpoint ')
        self.call(action, **kwargs)
        self.until(lambda: self.text().count('Endpoint ') > previous)
        self.wait(0.1)
        assert 'state=WAITING' in self.call('status')
        assert max(abs(v) for v in self.cmd) < 1e-8

    def fault(self, reason):
        self.until(lambda: reason in self.text(), 8)
        self.wait(0.2)
        assert max(abs(v) for v in self.cmd) < 1e-8
        self.call('forward', accepted=False, distance=0.1)

    def close(self):
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        self.log.close()
        self.node.destroy_node()


def run_case(name, fn, extra=None):
    if os.environ.get('STEP_CASE') and name != os.environ['STEP_CASE']:
        return
    plant = Plant(name, extra)
    try:
        fn(plant)
        RESULTS.append({'case': name, 'passed': True})
    finally:
        plant.close()
        (OUT / 'results.json').write_text(json.dumps(RESULTS, indent=2))


def planned_heading_mismatch(p):
    """A large planned yaw mismatch must latch a stop, never turn during adoption."""
    initial = (p.x, p.y, p.yaw)
    p.fault('PLANNED_HEADING_MISMATCH')
    assert max(abs(a - b) for a, b in zip(initial, (p.x, p.y, p.yaw))) < 1e-6


def adopt_pose(p):
    p.scan_enabled = False
    p.yaw = -1.7
    initial = p.pose()
    p.feedback_speed_override = 0.03
    p.wait(0.5)
    assert 'adopted current pose' not in p.text()
    assert max(abs(v) for v in p.cmd) < 1e-8
    p.feedback_speed_override = None
    reference = p.idle()
    assert abs(reference - (p.yaw + p.angle)) < 0.01
    assert math.hypot(p.pose()[0] - initial[0], p.pose()[1] - initial[1]) < 0.001
    assert 'pending=0' in p.call('status')
    assert 'Centered A recorded' not in p.text()
    p.move('forward', distance=0.06, speed=0.03, frame='heading', label='P04_trial')
    moved = (p.pose()[0] - initial[0], p.pose()[1] - initial[1])
    along = moved[0] * math.cos(reference) + moved[1] * math.sin(reference)
    cross = -moved[0] * math.sin(reference) + moved[1] * math.cos(reference)
    assert 0.045 < along < 0.075 and abs(cross) < 0.01, (along, cross)
    p.call('finish')
    p.until(lambda: p.proc.poll() is not None, 3)
    assert p.proc.returncode == 0


def sequence(p):
    reference = p.idle()
    p.cli_status()
    p.call('turn', accepted=False, distance=0.2)
    p.call('left', accepted=False, distance=0.05, side_centering=True)
    p.call('forward', accepted=False, distance=float('nan'))
    p.call('forward', accepted=False, distance=0.05, frame='body_live')
    p.yaw += 0.09
    p.wait(0.2)
    assert abs(p.cmd[0]) < 1e-10 and abs(p.cmd[1]) < 1e-10 and p.cmd[2] < 0
    p.until(lambda: abs(p.yaw + p.angle - reference) <= 0.01)
    p.wait(0.2)
    for action, dx, dy in [
        ('forward', 0.06, 0),
        ('backward', -0.04, 0),
        ('left', 0, 0.04),
        ('right', 0, -0.04),
    ]:
        before = p.pose()
        p.move(action, distance=abs(dx + dy), frame='heading')
        after = p.pose()
        expected = (
            math.cos(reference) * dx - math.sin(reference) * dy,
            math.sin(reference) * dx + math.cos(reference) * dy,
        )
        assert (
            math.hypot(after[0] - before[0] - expected[0], after[1] - before[1] - expected[1])
            < 0.011
        )
        assert abs(p.yaw + p.angle - reference) <= 0.0101
    x, y = p.pose()
    p.move('target', x=x - 0.03, y=y + 0.02, label='D')
    assert math.hypot(p.pose()[0] - (x - 0.03), p.pose()[1] - (y + 0.02)) < 0.011
    p.move('relative', x=0.03, y=-0.02)
    p.call('forward', distance=0.04)
    p.call('right', accepted=False, distance=0.03)
    p.until(lambda: p.text().count('Endpoint ') == 8)
    p.wait(0.15)
    p.call('finish')
    end = time.monotonic() + 3
    while p.proc.poll() is None and time.monotonic() < end:
        p.tick()
    assert p.proc.poll() == 0


def laser_policies(p):
    reference = p.idle()
    p.call('forward', distance=0.05, side_centering=True, label='D')
    p.y = 0.035
    p.until(lambda: p.text().count('Endpoint ') == 2)
    p.wait(0.1)
    assert abs(p.y) < 0.011
    p.scan_enabled = False
    p.wait(0.7)
    p.move('right', distance=0.04)
    assert abs(p.yaw + p.angle - reference) <= 0.0101


def front_arrival(p):
    p.idle()
    p.move('front_wall', distance=0.15, max_travel=0.5, timeout=20.0, label='D')
    assert 0.14 <= 0.65 - p.x - 0.20 <= 0.17
    assert 'Endpoint D recorded=' in p.text()


def front_failure(p):
    p.idle()
    p.call('front_wall', distance=0.15)
    p.wait(0.3)
    p.front_bad = True
    p.fault('REQUIRED_LASER_UNAVAILABLE')
    assert p.text().count('Endpoint ') == 1


def front_too_close(p):
    p.idle()
    p.call('front_wall', distance=0.60)
    p.fault('FRONT_CLEARANCE_TOO_SMALL')


def timeout_case(p):
    p.idle()
    p.frozen = True
    p.call('forward', distance=0.1, timeout=0.4)
    p.fault('TIME_OR_TRAVEL_LIMIT')


def travel_case(p):
    p.idle()
    p.call('forward', distance=0.1, max_travel=0.015)
    p.fault('TIME_OR_TRAVEL_LIMIT')


def odom_loss(p):
    p.idle()
    p.odom_enabled = False
    p.wait(0.8)
    assert 'state=RECOVERING' in p.call('status')
    p.wait(2.0)
    assert 'state=FAULT' in p.call('status')
    assert max(abs(v) for v in p.cmd) < 1e-8
    p.call('forward', accepted=False, distance=0.1)


def paused_route(p):
    p.until(lambda: 'Centered A recorded=' in p.text())
    p.wait(0.1)
    assert 'state=WAITING' in p.call('status') and 'Segment 1/' not in p.text()
    p.call('forward', accepted=False, distance=0.02)
    p.move('resume')
    p.call('right', accepted=False, distance=0.02)
    p.move('resume')
    p.call('resume', accepted=False)
    assert p.text().count('Endpoint ') == 2
    assert 'Endpoint D recorded=' in p.text()


def front_stale(p):
    p.idle()
    p.call('front_wall', distance=0.15)
    p.wait(0.2)
    p.scan_enabled = False
    p.fault('REQUIRED_LASER_UNAVAILABLE')
    assert p.text().count('Endpoint ') == 1


def cancel_step(p):
    p.idle()
    p.call('forward', distance=0.2)
    p.wait(0.2)
    p.call('cancel')
    p.fault('MANUAL_CANCEL')
    p.call('forward', accepted=False, distance=0.1)
    assert p.text().count('Endpoint ') == 1


def wait_return(p, legs, **kwargs):
    previous = p.text().count('Endpoint ')
    p.call(**kwargs)
    p.until(lambda: p.text().count('Endpoint ') == previous + legs, 35)
    p.wait(0.15)
    assert 'state=WAITING' in p.call('status')
    assert max(abs(v) for v in p.cmd) < 1e-8


def return_chain(p):
    reference = p.idle()
    b = p.pose()
    p.move('right', distance=0.06, label='C')
    c = p.pose()
    p.move('forward', distance=0.06, label='D')
    assert 'completed=3 outward=3' in p.call('history')
    p.call('return_to', accepted=False, label='Z')
    p.call('backtrack', accepted=False, steps=0)
    p.call('backtrack', accepted=False, steps=99)
    wait_return(p, 1, action='backtrack', steps=1)
    assert math.hypot(p.pose()[0] - c[0], p.pose()[1] - c[1]) < 0.012
    assert 'completed=4 outward=2' in p.call('history')
    wait_return(p, 1, action='return_to', label='B')
    assert math.hypot(p.pose()[0] - b[0], p.pose()[1] - b[1]) < 0.012
    p.move('left', distance=0.04, label='E')
    wait_return(p, 2, action='return_to', label='A')
    assert 'outward=0' in p.call('history')
    assert abs(p.yaw + p.angle - reference) < 0.021
    p.call('backtrack', accepted=False, steps=1)


def repeated_visits(p):
    p.idle()
    p.move('right', distance=0.04, label='C')
    p.move('left', distance=0.04, label='B')
    p.call('return_to', accepted=False, label='B')
    wait_return(p, 2, action='return_to', use_visit_id=True, visit_id=1)
    assert 'current_visit=1' in p.call('history')
    p.call('return_to', accepted=False, use_visit_id=True, visit_id=3)


def waiting_displacement(p):
    p.idle()
    p.x += 0.04
    p.fault('WAIT_POSITION_CHANGED')
    assert 'completed=1 outward=1' in p.call('history')
    p.call('backtrack', accepted=False, steps=1)


def return_failure(p):
    p.idle()
    p.move('right', distance=0.06, label='D')
    p.call('return_to', label='A')
    p.wait(0.2)
    p.call('cancel')
    p.fault('MANUAL_CANCEL')
    assert 'completed=2 outward=2' in p.call('history')


def return_laser_conversion(p):
    p.idle()
    b = p.pose()
    p.move('front_wall', distance=0.15, max_travel=0.5, timeout=20.0, label='D')
    p.scan_enabled = False
    p.wait(0.7)
    wait_return(p, 1, action='return_to', label='B')
    assert math.hypot(p.pose()[0] - b[0], p.pose()[1] - b[1]) < 0.012
    history = p.call('history')
    assert 'completion=front_wall' in history and 'reverse_of=2' in history
    reverse = next(line for line in history.splitlines() if line.startswith('edge=3 '))
    assert 'speed=0.03' in reverse


def return_replaces_pending(p):
    p.idle()
    assert 'pending=1' in p.call('status')
    wait_return(p, 1, action='return_to', label='A')
    assert 'pending=0' in p.call('status')
    p.call('resume', accepted=False)
    assert 'Endpoint D recorded=' not in p.text()


def cli_returns(p):
    p.idle()
    p.cli_status('history')
    p.move('right', distance=0.04, label='D')
    p.cli_status('backtrack', '1')
    p.until(lambda: p.text().count('Endpoint ') == 3)
    p.wait(0.15)
    p.cli_status('return_to', '--visit-id', '0')
    p.until(lambda: p.text().count('Endpoint ') == 4)
    p.wait(0.15)
    assert 'outward=0' in p.call('history')


def automatic_independent(p):
    p.until(lambda: 'Route completed.' in p.text(), 30)
    assert p.text().count('Endpoint ') == 2
    assert 'Endpoint D recorded=' in p.text()
    assert max(abs(v) for v in p.cmd) < 1e-8
    p.proc.wait(timeout=3)
    assert p.proc.returncode == 0


def waypoint_logs(p):
    import re

    p.until(lambda: 'Route completed.' in p.text(), 40)
    text = p.text()
    points = re.findall(r'Waypoint (\d+) (\w+): x=([-\d.]+) y=([-\d.]+) yaw=([-\d.]+)', text)
    goals = re.findall(r'Segment \d+/4 target=\(([-\d.]+), ([-\d.]+)\), yaw=([-\d.]+)', text)
    reached = re.findall(r'Reached (\w+) via', text)
    assert 'Initialization progress: stage=' in text
    assert 'center_error=' in text and 'rear_target=' in text
    assert 'heading_basis=held_odom_heading' in text
    assert 'required_hold=' in text
    assert text.index('Initialization progress:') < text.index('Centered A recorded=')
    assert [point[1] for point in points] == ['A', 'B', 'C', 'B', 'A'], points
    assert reached == ['B', 'C', 'B', 'A'], reached
    assert [point[2:] for point in points[1:]] == goals, (points, goals)
    assert points[0][2:] == points[-1][2:], points
    assert 'Target: x=' in text and 'heading_control=enabled' in text
    assert max(abs(v) for v in p.cmd) < 1e-8
    p.proc.wait(timeout=3)
    assert p.proc.returncode == 0


def start_outage(p):
    p.idle()
    p.call('forward', distance=0.15)
    p.wait(0.2)
    p.odom_enabled = False
    p.until(lambda: 'ODOM_RECOVERING:' in p.text(), 2)
    p.wait(0.1)
    assert max(abs(v) for v in p.cmd) < 1e-8
    assert p.text().count('Endpoint ') == 1
    p.call('forward', accepted=False, distance=0.1)


def odom_recover(p):
    start_outage(p)
    p.odom_enabled = True
    p.wait(0.15)
    assert 'ODOM_RECOVERED:' not in p.text()
    assert max(abs(v) for v in p.cmd) < 1e-8
    p.until(lambda: 'ODOM_RECOVERED:' in p.text(), 2)
    p.until(lambda: p.text().count('Endpoint ') == 2, 8)
    assert p.text().count('Segment 2/2 target=') == 1
    assert 'state=WAITING' in p.call('status')


def odom_exit(p):
    start_outage(p)
    p.until(lambda: p.proc.poll() is not None, 4)
    assert p.proc.returncode == 2
    assert 'ODOM_RECOVERY_TIMEOUT' in p.text()
    assert max(abs(v) for v in p.cmd) < 1e-8
    assert p.text().count('Endpoint ') == 1


def odom_stale(p):
    start_outage(p)
    p.odom_stamp_age = 1.0
    p.odom_enabled = True
    p.fault('ODOM_RECOVERY_TIMEOUT')
    assert 'ODOM_RECOVERED:' not in p.text()


def odom_future(p):
    start_outage(p)
    p.odom_stamp_age = -1.0
    p.odom_enabled = True
    p.fault('ODOM_RECOVERY_TIMEOUT')
    assert 'ODOM_RECOVERED:' not in p.text()


def odom_not_stopped(p):
    start_outage(p)
    p.feedback_speed_override = 0.03
    p.odom_enabled = True
    p.fault('ODOM_RECOVERY_TIMEOUT')
    assert 'ODOM_RECOVERED:' not in p.text()


def odom_jump(p):
    start_outage(p)
    p.x += 0.30
    p.odom_enabled = True
    p.fault('ODOM_RECOVERY_POSE_OR_FRAME_JUMP')
    assert 'ODOM_RECOVERED:' not in p.text()


def odom_frame_jump(p):
    start_outage(p)
    p.odom_frame_override = 'changed_odom'
    p.odom_enabled = True
    p.fault('ODOM_RECOVERY_POSE_OR_FRAME_JUMP')


def odom_single(p):
    start_outage(p)
    p.odom_enabled = True
    p.tick()
    p.odom_enabled = False
    p.fault('ODOM_RECOVERY_TIMEOUT')
    assert 'ODOM_RECOVERED:' not in p.text()


def odom_dwell(p):
    p.until(lambda: 'Entering DONE state | segment=0' in p.text(), 20)
    p.odom_enabled = False
    p.until(lambda: 'ODOM_RECOVERING:' in p.text(), 2)
    assert 'Endpoint ' not in p.text()
    p.odom_enabled = True
    p.until(lambda: 'ODOM_RECOVERED:' in p.text(), 2)
    p.wait(0.4)
    assert 'Endpoint ' not in p.text()
    p.until(lambda: p.text().count('Endpoint ') == 1, 5)
    assert p.text().count('Entering DONE state | segment=0') >= 2


def odom_budget(p):
    p.idle()
    p.call('forward', distance=0.5, timeout=1.0)
    p.wait(0.15)
    p.odom_enabled = False
    p.until(lambda: 'ODOM_RECOVERING:' in p.text(), 2)
    p.wait(0.4)
    p.odom_enabled = True
    p.fault('TIME_OR_TRAVEL_LIMIT')
    assert p.text().count('Endpoint ') == 1


def missing_extent(p):
    p.idle()
    p.call('front_wall', accepted=False, distance=0.15)
    assert 'state=WAITING' in p.call('status')


try:
    run_case('odom_recover', odom_recover)
    run_case('odom_exit', odom_exit)
    run_case('odom_stale', odom_stale, {'base_command_watchdog_verified': 'false'})
    run_case('odom_future', odom_future, {'base_command_watchdog_verified': 'false'})
    run_case('odom_not_stopped', odom_not_stopped, {'base_command_watchdog_verified': 'false'})
    run_case('odom_jump', odom_jump, {'base_command_watchdog_verified': 'false'})
    run_case('odom_frame_jump', odom_frame_jump, {'base_command_watchdog_verified': 'false'})
    run_case('odom_single', odom_single, {'base_command_watchdog_verified': 'false'})
    run_case('odom_dwell', odom_dwell, {'dwell_duration': '1.0'})
    run_case('odom_budget', odom_budget)
    run_case(
        'waypoint_logs',
        waypoint_logs,
        {
            '_params_file': Path(__file__).resolve().parents[1] / 'config/segments.yaml',
            'manual_mode': 'false',
            'route': '[AB, BC, CB, BA]',
            'segments.AB.dx': '.06',
            'segments.BC.dy': '-.05',
            'segments.CB.dy': '.05',
            'segments.BA.dx': '-.06',
            'segments.AB.dwell': '.02',
            'segments.BC.dwell': '.02',
            'segments.CB.dwell': '.02',
            'segments.BA.dwell': '.02',
        },
    )
    run_case(
        'automatic_independent',
        automatic_independent,
        {
            '_params_file': Path(__file__).resolve().parents[1] / 'config/segments.yaml',
            'manual_mode': 'false',
            'route': '[AB, BD]',
            'segments.AB.dx': '.06',
            'segments.AB.dwell': '.02',
            'segments.BD.dx': '.04',
            'segments.BD.speed': '.1',
            'segments.BD.dwell': '.02',
        },
    )
    run_case('cli_returns', cli_returns)
    run_case('return_chain', return_chain)
    run_case('repeated_visits', repeated_visits)
    run_case('waiting_displacement', waiting_displacement)
    run_case('return_failure', return_failure)
    run_case('return_laser_conversion', return_laser_conversion)
    run_case(
        'return_replaces_pending',
        return_replaces_pending,
        {'route': '[AB, BD]', 'segments.BD.dx': '.04', 'segments.BD.dy': '0.0'},
    )
    run_case(
        'independent_yaml',
        paused_route,
        {
            '_params_file': Path(__file__).resolve().parents[1] / 'config/segments.yaml',
            'route': '[AB, BD]',
            'start_paused': 'true',
            'segments.AB.dx': '.06',
            'segments.AB.dwell': '.02',
            'segments.BD.dx': '.04',
            'segments.BD.speed': '.1',
            'segments.BD.dwell': '.02',
        },
    )
    run_case(
        'planned_heading_mismatch',
        planned_heading_mismatch,
        {'adopt_current_pose': 'true', 'adopt_planned_heading': 'true', 'planned_heading': '1.2'},
    )
    run_case('adopt_pose', adopt_pose, {'adopt_current_pose': 'true'})
    run_case('sequence', sequence)
    run_case('laser_policies', laser_policies)
    run_case('front_arrival', front_arrival)
    run_case('front_invalid', front_failure)
    run_case('front_too_close', front_too_close)
    run_case('step_timeout', timeout_case)
    run_case('step_travel', travel_case)
    run_case('idle_odom_loss', odom_loss, {'base_command_watchdog_verified': 'false'})
    run_case(
        'paused_custom_route',
        paused_route,
        {
            'start_paused': 'true',
            'route': '[AB, BD]',
            'segments.BD.dx': '0.04',
            'segments.BD.dy': '0.0',
            'segments.AB.speed': '0.06',
        },
    )
    run_case('front_stale', front_stale)
    run_case('cancel_step', cancel_step)
    run_case('missing_extent', missing_extent, {'front_body_extent': '-1.0'})
finally:
    rclpy.shutdown()
    (OUT / 'results.json').write_text(json.dumps(RESULTS, indent=2))
assert RESULTS, 'No matching STEP_CASE was executed'
print(json.dumps({'out': str(OUT), 'results': RESULTS}))
