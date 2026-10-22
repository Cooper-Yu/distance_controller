"""Dual-epoch and immutable-evidence guard for boundary-only prefix recovery."""

from pathlib import Path
import time

from route_prefix_return_plan import ODOM, digest, odom_record, stopped
from route_return_plan import pose
from route_return_runner import Continuity
from route_session_state import near


def fingerprint(path):
    value = path.stat()
    return value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns, value.st_ctime_ns


class PrefixContinuity(Continuity):
    """Retain both original odometry identities; no target adoption or epoch recovery."""

    def __init__(self, runner, source, metadata):
        super().__init__(runner, metadata['gids'][ODOM[0]][0])
        self.source, self.metadata = source, metadata
        self.raw = None
        self.raw_invalid = None
        self.files = {Path(p): sha for p, sha in metadata['binding_files'].items()}
        self.stats = {p: fingerprint(p) for p in self.files}
        self.verify_bytes()
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data

        self.subscription = runner.node.create_subscription(
            Odometry, ODOM[1], self.receive_raw, qos_profile_sensor_data
        )

    def close(self):
        self.runner.node.destroy_subscription(self.subscription)

    def verify_bytes(self):
        if any(digest(p) != sha for p, sha in self.files.items()):
            raise RuntimeError('Original prefix evidence changed')
        self.check_files()

    def check_files(self):
        if any(fingerprint(p) != value for p, value in self.stats.items()):
            raise RuntimeError('Original prefix evidence metadata changed')

    def receive_raw(self, message):
        try:
            current = odom_record(time.monotonic(), message)
            actual = pose(current['pose'])
            stopped(current['velocity'])  # finite validation also while moving
            if current['frames'] != ('odom', 'base_link') or current['stamp_ns'] <= 0:
                raise ValueError('Raw odom frame/stamp invalid')
            if self.raw is not None:
                self.compare_raw(current, actual)
            self.raw = current
        except (ValueError, TypeError, KeyError) as error:
            self.raw_invalid = str(error)

    def compare_raw(self, current, actual):
        if current['stamp_ns'] < self.raw['stamp_ns']:
            raise ValueError('Raw odom timestamp moved backward')
        if current['stamp_ns'] == self.raw['stamp_ns']:
            if any(current[k] != self.raw[k] for k in ('pose', 'velocity', 'frames')):
                raise ValueError('Raw odom same-stamp content changed')
            current['receipt'] = self.raw['receipt']
        if not near(actual, pose(self.raw['pose']), 0.20, 0.25):
            raise ValueError('Raw odom jumped')

    def check(self):
        super().check()
        self.check_files()
        infos = self.runner.node.get_publishers_info_by_topic(ODOM[1])
        if [bytes(i.endpoint_gid).hex() for i in infos] != self.metadata['gids'][ODOM[1]]:
            raise RuntimeError('Raw odom publisher identity/count changed; prefix return refused')
        self.check_raw_fresh()
        self.runner.moving_pose()  # filtered source and receipt freshness throughout WAITING
        if not self.motion_started:
            self.check_initial_stop()

    def check_raw_fresh(self):
        if self.raw_invalid or self.raw is None:
            raise RuntimeError('Raw odometry unavailable/invalid: ' + str(self.raw_invalid))
        age = (self.runner.node.get_clock().now().nanoseconds - self.raw['stamp_ns']) / 1e9
        if not -0.1 <= age <= 0.5 or not 0 <= time.monotonic() - self.raw['receipt'] <= 0.5:
            raise RuntimeError('Raw odometry stale')
        if self.raw['stamp_ns'] < self.metadata['raw_stamp_ns']:
            raise RuntimeError('Raw odometry predates original closed boundary')

    def check_initial_stop(self):
        if not stopped(self.raw['velocity']) or not stopped(self.runner.body_velocity):
            raise RuntimeError('Prefix attach requires stopped dual odometry')
        if not near(pose(self.raw['pose']), pose(self.metadata['raw_last_stop']), 0.005, 0.01):
            raise RuntimeError('Raw pose moved from original stopped prefix')
        if not near(self.runner.moving_pose(), self.source.last_stop, 0.005, 0.01):
            raise RuntimeError('Filtered pose moved from original stopped prefix')

    def __call__(self, command):
        if not self.motion_started and any(abs(v) > 0 for v in command):
            self.verify_bytes()
        return super().__call__(command)
