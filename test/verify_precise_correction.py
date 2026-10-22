"""Isolated real-node initialization regression; generated room, no robot connection."""

import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

os.environ['ROS_DOMAIN_ID'] = '194'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ.pop('CYCLONEDDS_URI', None)
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

ROOT = Path(__file__).resolve().parents[1]


class Room:
    def __init__(self, mode, output):
        self.mode, self.output = mode, output
        self.node = rclpy.create_node('precise_correction_fixture')
        self.pose, self.velocity = [0.0] * 3, [0.0] * 3
        self.odom = self.node.create_publisher(Odometry, '/odometry/filtered', 10)
        self.scan = self.node.create_publisher(LaserScan, '/scan_filtered', 10)
        self.subscription = self.node.create_subscription(Twist, '/cmd_vel', self.receive, 10)
        self.broadcaster = StaticTransformBroadcaster(self.node)
        mount = TransformStamped()
        mount.header.frame_id, mount.child_frame_id = 'base_link', 'laser'
        mount.transform.translation.x, mount.transform.translation.z = 0.02, 0.173
        mount.transform.rotation.z = 1.0
        mount.transform.rotation.w = 0.0
        self.broadcaster.sendTransform(mount)
        self.last_tick, self.last_scan = time.monotonic(), 0.0
        self.verify_start = self.correction_start = self.stop_start = None
        self.eligible = 0
        self.active = False
        self.injected = False
        self.samples = []
        self.saved_stamp = None

    def receive(self, message):
        self.velocity[:] = [message.linear.x, message.linear.y, message.angular.z]
        self.samples.append(dict(t=time.monotonic(), pose=self.pose[:], command=self.velocity[:]))
        if self.active:
            assert math.hypot(*self.velocity[:2]) <= 0.008000001, self.velocity
            assert abs(self.velocity[2]) <= 0.020000001, self.velocity

    def log_state(self, now):
        text = (self.output / 'runtime.log').read_text()
        if self.verify_start is None and 'START_VERIFY: stopped hold' in text:
            self.verify_start = now
        if self.correction_start is None and 'START_CORRECTION:' in text:
            self.correction_start = now
        if self.stop_start is None and 'START_CORRECTION_STOP:' in text:
            self.stop_start = now
        self.active = text.rfind('START_CORRECTION:') > max(
            text.rfind('START_CORRECTION_STOP:'), text.rfind('START_CORRECTION_CANCELLED:')
        )
        return text

    def feedback_pose(self, now):
        dt = min(now - self.last_tick, 0.03)
        self.last_tick = now
        if self.mode != 'freeze' or self.correction_start is None:
            c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
            self.pose[0] += (c * self.velocity[0] - s * self.velocity[1]) * dt
            self.pose[1] += (s * self.velocity[0] + c * self.velocity[1]) * dt
            disturbance = 0.0015 if self.mode == 'heading_hold_positive' else -0.0015
            if not self.active or not self.mode.startswith('heading_hold_'):
                disturbance = 0.0
            self.pose[2] += (self.velocity[2] + disturbance) * dt
        if self.correction_start is not None and not self.injected:
            if self.mode.startswith('heading_hold_'):
                self.pose[2] += 0.003 if self.mode.endswith('positive') else -0.003
            elif self.mode == 'heading':
                self.pose[2] += 0.006
            elif self.mode == 'cross_track':
                self.pose[1] += 0.0035
            elif self.mode == 'travel':
                self.pose[0] -= 0.013
            self.injected = True

    def publish_odom(self, now):
        if self.mode == 'odom_gap' and self.correction_start is not None:
            if 0.1 < now - self.correction_start < 0.95:
                return
        message = Odometry()
        message.header.frame_id, message.child_frame_id = 'odom', 'base_link'
        message.header.stamp = self.node.get_clock().now().to_msg()
        if self.correction_start is not None:
            if self.mode == 'odom_frame':
                message.header.frame_id = 'new_odom'
            elif self.mode == 'odom_stamp':
                message.header.stamp.sec -= 1
        message.pose.pose.position.x, message.pose.pose.position.y = self.pose[:2]
        message.pose.pose.orientation.z = math.sin(self.pose[2] / 2)
        message.pose.pose.orientation.w = math.cos(self.pose[2] / 2)
        (
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
            message.twist.twist.angular.z,
        ) = self.velocity
        if self.verify_start is not None and self.mode == 'stop_velocity':
            message.twist.twist.linear.x = 0.004
        if self.stop_start is not None and self.mode == 'post_stop' and now - self.stop_start < 0.4:
            message.twist.twist.linear.x = 0.004
        self.odom.publish(message)

    def distances(self, now):
        right, rear = 0.261149, 0.284
        if self.verify_start is not None:
            rear = 0.27107325982 if self.mode == 'reverse' else 0.28892674018
            if self.mode in ('scan_gap', 'odom_gap'):
                rear = 0.294  # Residual stays outside tolerance after the interrupted first step.
            if self.mode == 'dual':
                right = 0.26892674018
            if self.mode == 'oversized':
                rear = 0.305
            if self.mode == 'attempt_limit':
                rear -= self.pose[0]  # Keep a stable bias after each successful odom move.
            if self.correction_start is None and now - self.verify_start >= 0.52:
                self.eligible += 1
                if self.mode == 'unstable':
                    right += 0.008 if self.eligible % 2 else -0.008
                elif self.eligible <= 8:
                    right += 0.012  # First disjoint batch fails; rolling window later stabilizes.
                if (
                    self.mode not in ('reverse', 'oversized', 'scan_gap', 'odom_gap')
                    and self.eligible % 5 == 0
                ):
                    rear = 0.2845  # An occasional single frame is already within final tolerance.
            if self.active and self.mode == 'single_frame' and not hasattr(self, 'perfect_frame'):
                self.perfect_frame = True
                return 0.26 - self.pose[1], 0.28 - self.pose[0]
        return right, rear

    def publish_scan(self, now):
        if now - self.last_scan < 0.07:
            return
        if self.mode == 'scan_gap' and self.correction_start is not None:
            if 0.1 < now - self.correction_start < 0.95:
                return
        self.last_scan = now
        right, rear = self.distances(now)
        message = LaserScan()
        message.header.frame_id = 'laser'
        message.header.stamp = self.node.get_clock().now().to_msg()
        if self.stop_start is not None and self.mode == 'post_stop' and now - self.stop_start < 0.4:
            message.header.stamp = self.saved_stamp
        else:
            self.saved_stamp = message.header.stamp
        message.angle_min, message.angle_increment, message.angle_max = (
            -math.pi,
            math.pi / 360,
            math.pi,
        )
        message.range_min, message.range_max = 0.15, 40.0
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        x, y = self.pose[0] + 0.02 * c, self.pose[1] + 0.02 * s
        ranges = []
        for index in range(721):
            angle = message.angle_min + index * message.angle_increment + self.pose[2] + math.pi
            dx, dy = math.cos(angle), math.sin(angle)
            candidates = []
            if abs(dx) > 1e-8:
                candidates.append(((0.6 if dx > 0 else -rear) - x) / dx)
            if abs(dy) > 1e-8:
                candidates.append(((0.4 if dy > 0 else -right) - y) / dy)
            distance = min(v for v in candidates if v > 0)
            ranges.append(distance if distance >= 0.15 else math.inf)
        message.ranges = ranges
        self.scan.publish(message)

    def tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.008)
        now = time.monotonic()
        self.log_state(now)
        self.feedback_pose(now)
        self.publish_odom(now)
        self.publish_scan(now)


def verify(room, code, text):
    faults = dict(
        freeze='START_CORRECTION_TIMEOUT',
        heading='START_CORRECTION_HEADING',
        cross_track='START_CORRECTION_CROSS_TRACK',
        travel='START_CORRECTION_TRAVEL_LIMIT',
        unstable='START_VERIFY_TIMEOUT',
        stop_velocity='START_VERIFY_TIMEOUT',
        oversized='START_CORRECTION_RANGE',
        odom_frame='START_CORRECTION_ODOM_FRAME',
        odom_stamp='START_CORRECTION_ODOM_STAMP',
        attempt_limit='START_REFINEMENT_LIMIT',
        stage_deadline='POSITIONING_TIMEOUT',
        total_deadline='PREPARATION_TOTAL_TIMEOUT',
    )
    if room.mode in faults:
        assert code == 2 and faults[room.mode] in text, (code, text[-5000:])
        assert 'START_VERIFIED:' not in text
        if room.mode in ('unstable', 'stop_velocity', 'oversized'):
            assert 'START_CORRECTION:' not in text
        if room.mode == 'attempt_limit':
            attempts = re.findall(r'START_CORRECTION: attempt=(\d+)', text)
            assert attempts == ['1', '2', '3'], attempts
        if room.mode in ('stage_deadline', 'total_deadline'):
            assert 'START_CORRECTION:' in text
            assert f'START_CORRECTION_CANCELLED: reason={faults[room.mode]}' in text
    else:
        assert code == 0 and 'START_VERIFIED:' in text, (code, text[-5000:])
        assert 'START_CORRECTION:' in text and 'START_CORRECTION_STOP:' in text
        assert text.index('START_CORRECTION_STOP:') < text.index('START_VERIFIED:')
        assert text.count('START_VERIFY: stopped hold') >= 2
        steps = re.findall(
            r'START_CORRECTION: attempt=(\d+) body_dx=([-\d.]+) body_dy=([-\d.]+)', text
        )
        assert steps and max(int(a) for a, _, _ in steps) <= 3
        for _, x, y in steps:
            assert math.hypot(float(x), float(y)) <= 0.010001
            assert float(x) > 0 if room.mode == 'reverse' else float(x) < 0
        if room.mode in ('scan_gap', 'odom_gap'):
            token = 'scan_recovery' if room.mode == 'scan_gap' else 'odom_recovery'
            assert f'START_CORRECTION_CANCELLED: reason={token}' in text
            suffix = text.split(f'START_CORRECTION_CANCELLED: reason={token}', 1)[1]
            assert suffix.index('START_VERIFY: stopped hold') < suffix.index('START_VERIFIED:')
            assert len(steps) == 2 and [int(a) for a, _, _ in steps] == [1, 2]
            assert abs(room.pose[0] + 0.014) < 0.0006  # New median goal, not old -0.010 target.
        if room.mode.startswith('heading_hold_'):
            start = room.correction_start
            stop = room.stop_start
            active = [s for s in room.samples if start <= s['t'] < stop]
            assert active and max(abs(s['pose'][2]) for s in active) <= 0.005
            assert max(abs(s['command'][2]) for s in active) > 0.002
            sign = 1 if room.mode.endswith('positive') else -1
            assert any(sign * s['command'][2] < -0.002 for s in active)
            assert abs(active[-1]['pose'][2]) < 0.0012, active[-1]
            assert 'START_CORRECTION_HEADING' not in text
        if room.mode == 'post_stop':
            assert time.monotonic() - room.stop_start > 1.8
    assert room.samples and sum(abs(v) for v in room.velocity) < 1e-9
    assert not room.node.get_publishers_info_by_topic('/cmd_vel')


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'single_frame'
    if mode == 'deadline_child':
        return deadline_child(sys.argv[2])
    if mode == 'node_fault_child':
        return node_fault_child()
    output = Path('/tmp/precise_correction_ros') / f'{time.time_ns()}_{mode}'
    output.mkdir(parents=True)
    rclpy.init()
    room = Room(mode, output)
    command = [
        'python3',
        str(ROOT / 'tools/odom_trial.py'),
        '--start',
        'prepare',
        '--alignment-wall',
        'right',
        '--positioning-wall',
        'right',
        '--right-target-distance',
        '0.26',
        '--precise-start',
        '--laser-log-only',
    ]
    if mode in ('stage_deadline', 'total_deadline'):
        command = ['python3', str(Path(__file__).resolve()), 'deadline_child', mode]
    if mode in ('odom_frame', 'odom_stamp'):
        command = ['python3', str(Path(__file__).resolve()), 'node_fault_child']
    proc = None
    try:
        with (output / 'runtime.log').open('w') as log:
            proc = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=log,
                stderr=subprocess.STDOUT,
                text=True,
                cwd=output,
                start_new_session=True,
            )
            proc.stdin.write('quit\n')
            proc.stdin.close()
            deadline = time.monotonic() + 45
            while proc.poll() is None and time.monotonic() < deadline:
                room.tick()
            assert proc.poll() is not None, 'Fixture deadline'
            for _ in range(80):
                room.tick()
            text = (output / 'runtime.log').read_text()
            verify(room, proc.returncode, text)
            (output / 'commands.json').write_text(json.dumps(room.samples))
            (output / 'result.json').write_text(
                json.dumps(dict(mode=mode, exit_code=proc.returncode, last_zero=True, publishers=0))
            )
            print('PASS', mode, output, flush=True)
    finally:
        if proc is not None and proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=10)
        room.node.destroy_node()
        rclpy.shutdown()


def node_fault_child():
    """Observe the C++ fault directly, before a higher adapter rejects the same input."""
    import fcntl

    sys.path.insert(0, str(ROOT / 'tools'))
    from survey_return import Runner

    with open('/tmp/distance_controller_survey_return.lock', 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        runner = Runner()
        try:
            runner.exclusive()
            runner.start(
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
                    'start_paused:=true',
                    '-p',
                    'alignment_wall:=right',
                    '-p',
                    'robust_wall_heading:=true',
                    '-p',
                    'positioning_wall:=right',
                    '-p',
                    'right_target_distance:=0.26',
                    '-p',
                    'precise_start:=true',
                ]
            )
            runner.wait_idle(30)
        except (RuntimeError, ValueError, OSError, KeyboardInterrupt) as error:
            print('STOPPED:', error, flush=True)
            return 2
        finally:
            runner.close(True)
    return 0


def deadline_child(mode):
    """Use the real preparation adapter with a shorter test-only outer deadline."""
    import fcntl

    sys.path.insert(0, str(ROOT / 'tools'))
    from wall_runtime import WallRunner

    with open('/tmp/distance_controller_survey_return.lock', 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        runner = WallRunner()
        runner.prepare_options += [
            '-p',
            'positioning_wall:=right',
            '-p',
            'right_target_distance:=0.26',
            '-p',
            'precise_start:=true',
            '-p',
            'positioning_timeout:=3.0' if mode == 'stage_deadline' else 'preparation_timeout:=4.8',
        ]
        try:
            runner.exclusive()
            runner.prepare_start()
        except (RuntimeError, ValueError, OSError, KeyboardInterrupt) as error:
            print('STOPPED:', error, flush=True)
            return 2
        finally:
            runner.stop_owned()
            runner.close(False)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
