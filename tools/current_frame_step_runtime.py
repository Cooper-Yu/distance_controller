"""New-epoch, bounded single-step feedback checks; never restore a historical P01."""

import math
import time

from action_plan import Pose
from route_laser_runtime import RouteLaserGuard, ShadowRouteLaserGuard
from route_prefix_return_plan import ODOM, odom_record, stopped
from route_session_state import near


class CurrentEpochGuard:
    """Bind current publishers; explicit manual mode observes laser without certifying free space."""

    def __init__(self, runner, gids, distance, speed, emit, *, supervised_manual=False):
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data

        self.runner, self.gids, self.distance, self.speed = runner, gids, distance, speed
        self.raw, self.raw_invalid, self.origin, self.raw_origin = None, None, None, None
        self.filtered = None
        self.motion_started = False
        self.heading_bound_rad = 0.035 if supervised_manual else 0.01
        self.path_m = 0.0
        self.path_pose = None
        self.laser = (
            ShadowRouteLaserGuard(runner, emit)
            if supervised_manual
            else RouteLaserGuard(runner, emit)
        )
        self.subscription = runner.node.create_subscription(
            Odometry, ODOM[1], self.receive_raw, qos_profile_sensor_data
        )

    def close(self):
        self.runner.node.destroy_subscription(self.subscription)

    def receive_raw(self, message):
        try:
            current = odom_record(time.monotonic(), message)
            actual = Pose(**current['pose'])
            if not all(math.isfinite(v) for v in vars(actual).values()):
                raise ValueError('Nonfinite raw odometry pose')
            stopped(current['velocity'])
            if current['frames'] != ('odom', 'base_link') or current['stamp_ns'] <= 0:
                raise ValueError('Raw odometry frame/stamp invalid')
            if self.raw is not None:
                previous = self.raw
                if current['stamp_ns'] < previous['stamp_ns']:
                    raise ValueError('Raw odometry timestamp moved backward')
                if current['stamp_ns'] == previous['stamp_ns']:
                    if any(current[k] != previous[k] for k in ('pose', 'frames', 'velocity')):
                        raise ValueError('Raw odometry same-stamp content changed')
                    current['receipt'] = previous['receipt']
                if not near(actual, Pose(**previous['pose']), 0.20, 0.25):
                    raise ValueError('Raw odometry jumped')
            self.raw = current
        except (ValueError, TypeError, KeyError) as error:
            self.raw_invalid = str(error)

    def check_filtered(self):
        runner = self.runner
        current = runner.moving_pose()
        data = (current, tuple(runner.body_velocity), runner.frame, runner.twist_frame)
        if self.filtered is not None:
            stamp, previous, receipt = self.filtered
            if runner.stamp < stamp or (runner.stamp == stamp and data != previous):
                raise RuntimeError('Filtered odometry chronology/content changed')
        else:
            receipt = runner.latest[1]
        if self.filtered is None or runner.stamp != self.filtered[0]:
            receipt = runner.latest[1]
        self.filtered = runner.stamp, data, receipt
        if not 0 <= time.monotonic() - receipt <= 0.5:
            raise RuntimeError('Filtered odometry receipt stale')
        if (runner.frame, runner.twist_frame) != ('odom', 'base_link'):
            raise RuntimeError('Filtered odometry frame changed')
        return current

    def check(self):
        for topic in ODOM:
            actual = [
                bytes(i.endpoint_gid).hex()
                for i in self.runner.node.get_publishers_info_by_topic(topic)
            ]
            if actual != [self.gids[topic]]:
                raise RuntimeError('Current odometry publisher identity/count changed: ' + topic)
        if self.raw_invalid or self.raw is None:
            raise RuntimeError('Raw odometry unavailable/invalid: ' + str(self.raw_invalid))
        source_age = (self.runner.node.get_clock().now().nanoseconds - self.raw['stamp_ns']) / 1e9
        if not -0.1 <= source_age <= 0.5 or not 0 <= time.monotonic() - self.raw['receipt'] <= 0.5:
            raise RuntimeError('Raw odometry stale')
        actual = self.check_filtered()
        if self.origin is not None:
            self.check_bound(actual)
        return actual

    def check_bound(self, actual):
        if not self.motion_started:
            if (
                not stopped(self.raw['velocity'])
                or not stopped(self.runner.body_velocity)
                or not near(actual, self.origin, 0.005, 0.01)
                or not near(Pose(**self.raw['pose']), self.raw_origin, 0.005, 0.01)
            ):
                raise RuntimeError('Pose/velocity changed before first small-step command')
        if self.path_pose is not None:
            self.path_m += math.hypot(actual.x - self.path_pose.x, actual.y - self.path_pose.y)
        self.path_pose = actual
        if self.path_m > self.distance + 0.02:
            raise RuntimeError('Small-step total travel exceeded distance + 2 cm')
        if not near(actual, self.origin, self.distance + 0.01, self.heading_bound_rad):
            raise RuntimeError('Small-step displacement/heading bound exceeded')

    def bind_stopped(self, cancellation):
        deadline, held, anchor, raw_anchor = time.monotonic() + 3, None, None, None
        while time.monotonic() < deadline:
            self.runner.pump()
            cancellation.check()
            if self.raw is None or self.runner.latest is None:
                continue
            actual = self.check()
            quiet = stopped(self.raw['velocity']) and stopped(self.runner.body_velocity)
            raw_pose = Pose(**self.raw['pose'])
            if not quiet or (
                anchor is not None
                and (
                    not near(actual, anchor, 0.003, 0.005)
                    or not near(raw_pose, raw_anchor, 0.003, 0.005)
                )
            ):
                held, anchor, raw_anchor = None, None, None
                continue
            anchor, raw_anchor, held = (
                anchor or actual,
                raw_anchor or raw_pose,
                held or time.monotonic(),
            )
            if time.monotonic() - held >= 0.5:
                self.origin, self.raw_origin = actual, Pose(**self.raw['pose'])
                self.path_pose = actual
                return actual
        raise RuntimeError('Fresh stopped dual odometry hold unavailable')

    def __call__(self, command):
        self.check()
        if (
            len(command) != 3
            or not all(math.isfinite(v) for v in command)
            or abs(command[2]) > 0.100000001
        ):
            raise RuntimeError('Single-step command exceeds bounded velocity')
        scale = min(1.0, self.speed / max(math.hypot(*command[:2]), 1e-30))
        command = (command[0] * scale, command[1] * scale, command[2])
        # RouteLaserGuard snapshots current pose and reprojects scan-time points on every
        # call; its scan/TF pairing cache never freezes the current-base transformation.
        # Manual supervision never waives scan timestamp, TF, or pairing validity.
        # Shadow evaluation may report unknown coverage, but stale input still faults.
        self.runner.geometry()
        self.runner.scan_pose()
        applied = self.laser(command)
        if not self.motion_started and any(abs(v) > 0 for v in applied):
            self.check()
            self.motion_started = True
        return applied
