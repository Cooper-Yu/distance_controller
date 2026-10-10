"""ROS backend for one-origin plans; reuse validated C++ motion controllers."""

import math
import os
import signal
import subprocess
import time

from action_plan import Pose, errors
from survey_return import Runner, idle, parse_status


class PlannedRunner(Runner):
    """Track one continuous odom epoch and execute absolute targets in that epoch."""

    def __init__(self):
        super().__init__(handle_signals=False)
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data

        self.cancel_requested = False
        self.latest = None
        self.stamp = None
        self.frame = None
        self.invalid = None
        self.last_raw = None
        self.continuous_yaw = 0.0
        self.odom_sub = self.node.create_subscription(
            Odometry, '/odometry/filtered', self.receive_pose, qos_profile_sensor_data
        )

    def receive_pose(self, msg):
        """Read odom into a continuous-yaw pose; latch discontinuities, never rebase the route."""
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        values = [p.x, p.y, q.x, q.y, q.z, q.w, v.linear.x, v.linear.y, v.angular.z]
        if not all(math.isfinite(x) for x in values):
            self.invalid = 'Nonfinite odom'
            return
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if abs(norm - 1) > 0.01:
            self.invalid = 'Invalid odom orientation'
            return
        raw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if self.stamp is not None and stamp < self.stamp:
            self.invalid = 'Odom timestamp moved backward'
        if self.frame is not None and self.frame != msg.header.frame_id:
            self.invalid = 'Odom frame changed'
        if self.latest and math.hypot(p.x - self.latest[0].x, p.y - self.latest[0].y) > 0.20:
            self.invalid = 'Odom position jumped'
        delta = math.atan2(
            math.sin(raw - (self.last_raw or 0)), math.cos(raw - (self.last_raw or 0))
        )
        if self.last_raw is not None and abs(delta) > 0.25:
            self.invalid = 'Odom yaw jumped'
        self.continuous_yaw = raw if self.last_raw is None else self.continuous_yaw + delta
        self.last_raw, self.stamp, self.frame = raw, stamp, msg.header.frame_id
        self.latest = (
            Pose(p.x, p.y, self.continuous_yaw),
            time.monotonic(),
            math.hypot(v.linear.x, v.linear.y),
            abs(v.angular.z),
        )

    def request_cancel(self):
        """Signal-safe Python handler: request stop without raising inside rclpy C callbacks."""
        self.cancel_requested = True
        if self.child is not None:
            try:
                os.killpg(self.child.pid, signal.SIGINT)
            except ProcessLookupError:
                pass

    def pump(self):
        self.ros.spin_once(self.node, timeout_sec=0.05)
        if self.cancel_requested:
            raise KeyboardInterrupt
        if self.invalid:
            raise RuntimeError(self.invalid + '; stop and establish a new session after inspection')

    def pose(self):
        """Return fresh stopped pose; receipt freshness alone cannot validate stale message stamps."""
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            self.pump()
            if self.latest is None:
                continue
            pose, receipt, speed, wz = self.latest
            age = (self.node.get_clock().now().nanoseconds - self.stamp) / 1e9
            if (
                time.monotonic() - receipt <= 0.5
                and -0.1 <= age <= 0.5
                and speed < 0.01
                and wz < 0.02
            ):
                return pose
        raise RuntimeError('Fresh stopped odom unavailable')

    def wait_idle(self, timeout, label=None, preparation=False):
        """Preparation may retain its paused default route; completed actions require an empty queue."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump()
            if self.child.poll() is not None:
                raise RuntimeError(f'Controller exited early: {self.child.returncode}')
            if not self.client.service_is_ready():
                continue
            message = self.call('status')
            if parse_status(message).get('state') == 'FAULT':
                raise RuntimeError(message)
            fields = parse_status(message)
            prepared = (
                preparation
                and fields.get('state') == 'WAITING'
                and fields.get('waypoint') == 'A'
                and fields.get('return_remaining') == '0'
            )
            if prepared or idle(message, label):
                return message
        raise RuntimeError('Timed out waiting for endpoint')

    def stopped_heading(self, target):
        """Wait through small planned-heading holding corrections before sending a translation."""
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            actual = self.pose()
            if abs(errors(actual, target)[1]) < 0.01:
                return
        raise RuntimeError('Planned heading did not settle')

    def translate_target(self, target):
        """Command absolute odom x/y and planned heading; never anchor targets at actual stop errors."""
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
                'adopt_planned_heading:=true',
                '-p',
                f'planned_heading:={target.yaw}',
                '-p',
                'max_speed:=0.03',
            ]
        )
        self.wait_idle(15)
        self.stopped_heading(target)
        actual = self.pose()
        distance = math.hypot(actual.x - target.x, actual.y - target.y)
        budget = distance / 0.03 * 3 + 20
        print(
            self.call(
                'target',
                x=target.x,
                y=target.y,
                speed=0.03,
                dwell=0.5,
                timeout=budget,
                label='session_target',
            ),
            flush=True,
        )
        print(self.wait_idle(budget + 5, 'session_target'), flush=True)
        self.call('finish')
        self.wait_exit(10)

    def wait_exit(self, timeout):
        deadline = time.monotonic() + timeout
        while self.child.poll() is None and time.monotonic() < deadline:
            self.pump()
        if self.child.poll() is None:
            raise RuntimeError('Controller exit timeout')
        if self.child.returncode != 0:
            raise RuntimeError(f'Controller failure: {self.child.returncode}')

    def rotate_target(self, target):
        """Convert continuous target yaw to a relative turn from the observed start, preserving sign."""
        actual = self.pose()
        if math.hypot(actual.x - target.x, actual.y - target.y) > 0.03:
            raise RuntimeError('Turn position displaced; inspect before rotating')
        delta = target.yaw - actual.yaw
        if abs(delta) < 0.005:
            return
        if abs(delta) > 2 * math.pi + 0.05:
            raise RuntimeError('Turn delta outside this session range')
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
                f'turn_angles:=[{delta}]',
                '-p',
                'max_angular_speed:=0.20',
                '-p',
                'segment_timeout:=60.0',
            ]
        )
        self.wait_exit(65)

    def stop_owned(self):
        """On failure/interrupt stop owned children and publish zeros; retain this ROS observer."""
        if self.child is None:
            return
        if self.child.poll() is None:
            os.killpg(self.child.pid, signal.SIGINT)
        self.stop_pub = self.node.create_publisher(self.twist, '/cmd_vel', 10)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            self.stop_pub.publish(self.twist())
            self.ros.spin_once(self.node, timeout_sec=0.05)
        try:
            self.child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(self.child.pid, signal.SIGKILL)
            self.child.wait(timeout=5)
        self.node.destroy_publisher(self.stop_pub)
        self.stop_pub = None

    def prepare_start(self):
        """Run existing A-wall preparation once, finish it, then let Session capture its origin."""
        try:
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
                    'start_paused:=true',
                ]
            )
            print(self.wait_idle(90, preparation=True), flush=True)
            self.call('finish')
            self.wait_exit(10)
        except (Exception, KeyboardInterrupt):
            self.stop_owned()
            raise
        finally:
            self.child = None
            self.cancel_requested = False

    def execute(self, step, target):
        """Exclusive one-action lifetime; observations and exceptions propagate to Session history."""
        self.child = None
        try:
            self.exclusive()
            self.pose()
            self.observe('before')
            if step.kind == 'turn':
                self.rotate_target(target)
            else:
                self.translate_target(target)
            self.pose()
            self.observe('after')
        except (Exception, KeyboardInterrupt):
            self.stop_owned()
            raise
        finally:
            self.child = None
            self.cancel_requested = False
