"""ROS backend for one-origin plans; reuse validated C++ motion controllers."""

import math
import os
import signal
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

    def start(self, args):
        """Bind fault evidence to this child lifetime, never a prior controller."""
        self.owned_fault_observed = False
        self.owned_fault_child = None
        super().start(args)

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
        self.body_velocity = (v.linear.x, v.linear.y, v.angular.z)
        self.twist_frame = msg.child_frame_id
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
                self.owned_fault_observed = True
                self.owned_fault_child = self.child
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
                f'max_angular_speed:={getattr(self, "turn_speed", 0.20)}',
                '-p',
                'segment_timeout:=60.0',
            ]
        )
        self.wait_exit(65)

    def request_owned_zero(self):
        """Ask the existing owner to latch zero; never introduce a competing publisher."""
        if (
            getattr(self, 'owned_fault_observed', False)
            and getattr(self, 'owned_fault_child', None) is self.child
        ):
            return True  # C++ FAULT paths publish zero before reporting FAULT.
        try:
            if self.child.poll() is not None or not self.client.service_is_ready():
                return False
            request = self.service.Request()
            request.action = 'cancel'
            future = self.client.call_async(request)
            deadline = time.monotonic() + 0.10
            while not future.done() and time.monotonic() < deadline:
                self.ros.spin_once(
                    self.node, timeout_sec=max(0.0, min(0.01, deadline - time.monotonic()))
                )
            return future.done() and future.result() is not None and future.result().accepted
        except (Exception, KeyboardInterrupt) as error:
            print('Owned zero request unconfirmed:', error, flush=True)
            return False

    @staticmethod
    def owned_group_alive(child):
        """The Popen child was started in its own session; include surviving descendants."""
        try:
            os.killpg(child.pid, 0)
        except ProcessLookupError:
            return False
        return True

    def terminate_owned(self, child):
        """Short bounded escalation; process exit is not proof of physical braking."""
        for sig, duration in (
            (signal.SIGINT, 0.15),
            (signal.SIGTERM, 0.10),
            (signal.SIGKILL, 0.25),
        ):
            if child.poll() is not None and not self.owned_group_alive(child):
                return
            try:
                os.killpg(child.pid, sig)
            except ProcessLookupError:
                pass
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                if child.poll() is not None and not self.owned_group_alive(child):
                    return
                # Do not call pump(): a latched cancellation must not abort cleanup.
                self.ros.spin_once(
                    self.node, timeout_sec=max(0.0, min(0.01, deadline - time.monotonic()))
                )
        if child.poll() is None or self.owned_group_alive(child):
            raise RuntimeError(
                'Owned controller did not exit after bounded escalation; stop unverified'
            )

    def wait_command_release(self):
        """Require a continuous empty graph before creating our stop publisher."""
        deadline, empty_since = time.monotonic() + 2.0, None
        while time.monotonic() < deadline:
            self.ros.spin_once(self.node, timeout_sec=0.01)
            now = time.monotonic()
            if self.node.get_publishers_info_by_topic('/cmd_vel'):
                empty_since = None
            elif empty_since is None:
                empty_since = now
            elif now - empty_since >= 0.10:
                return
        raise RuntimeError('Command publisher did not release; no competing zero publisher created')

    def stop_owned(self):
        """Reap the owned controller, then hand zero output to exactly one parent publisher.

        Unresponsive children are killed within the bounded escalation; this does not
        certify physical stopping without the base watchdog and fresh feedback checks.
        """
        if self.child is None:
            return
        self.owned_zero_acknowledged = self.request_owned_zero()
        self.terminate_owned(self.child)
        self.child = None
        self.wait_command_release()
        self.stop_pub = self.node.create_publisher(self.twist, '/cmd_vel', 10)
        try:
            deadline, next_due = time.monotonic() + 1.0, 0.0
            while (now := time.monotonic()) < deadline:
                if now >= next_due:
                    self.stop_pub.publish(self.twist())
                    next_due = time.monotonic() + 0.05
                self.ros.spin_once(
                    self.node, timeout_sec=max(0.0, min(next_due, deadline) - time.monotonic())
                )
        finally:
            self.node.destroy_publisher(self.stop_pub)
            self.stop_pub = None

    def cleanup_preserving_error(self, original):
        """A secondary cleanup failure must not erase the first controller fault."""
        print(
            'Controller operation failed before cleanup:', str(original) or 'Cancelled', flush=True
        )
        try:
            self.stop_owned()
        except (Exception, KeyboardInterrupt) as error:
            original.cleanup_error = str(error) or type(error).__name__
            print('Secondary owned cleanup failure:', original.cleanup_error, flush=True)

    def prepare_start(self):
        """Run existing A-wall preparation once, finish it, then let Session capture its origin."""
        print(
            'Preparing start only; the action-session route will be shown after handoff.',
            flush=True,
        )
        self.owned_fault_observed = False
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
                + getattr(self, 'prepare_options', [])
            )
            print(self.wait_idle(90, preparation=True), flush=True)
            self.call('finish')
            self.wait_exit(10)
            self.child = None
        except (Exception, KeyboardInterrupt) as error:
            self.cleanup_preserving_error(error)
            raise

    def execute(self, step, target):
        """Exclusive one-action lifetime; observations and exceptions propagate to Session history."""
        self.child = None
        self.owned_fault_observed = False
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
            self.child = None
        except (Exception, KeyboardInterrupt) as error:
            self.cleanup_preserving_error(error)
            raise
