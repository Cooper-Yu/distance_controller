"""Bounded stopped feedback recovery for the prepared observation-only Task6 route."""

from collections import deque
import math
import time


class OdomEpoch:
    """Validate one immutable frame/clock/pose epoch, including the raw source."""

    def __init__(self):
        self.sample = None
        self.error = None
        self.history = deque(maxlen=100)

    def receive(self, msg, now):
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        values = (
            p.x,
            p.y,
            p.z,
            q.x,
            q.y,
            q.z,
            q.w,
            v.linear.x,
            v.linear.y,
            v.linear.z,
            v.angular.x,
            v.angular.y,
            v.angular.z,
        )
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp <= 0:
            self.error = 'Invalid odometry timestamp'
            return
        if not all(math.isfinite(x) for x in values):
            self.error = 'Nonfinite odometry'
            return
        if abs(sum(x * x for x in (q.x, q.y, q.z, q.w)) ** 0.5 - 1) > 0.01:
            self.error = 'Invalid odometry quaternion'
            return
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        frame = (msg.header.frame_id, msg.child_frame_id)
        if frame != ('odom', 'base_link'):
            self.error = 'Odometry frame changed'
        if self.sample is not None:
            old = self.sample
            if stamp == old['stamp'] and values != old['values']:
                self.error = 'Odometry changed at an unchanged timestamp'
                return
            if stamp < old['stamp']:
                self.error = 'Odometry timestamp moved backward'
            if math.hypot(p.x - old['x'], p.y - old['y']) > 0.20:
                self.error = 'Odometry position jumped'
            if abs(math.remainder(yaw - old['yaw'], 2 * math.pi)) > 0.25:
                self.error = 'Odometry yaw jumped'
        sample = dict(
            stamp=stamp,
            receipt=now,
            x=p.x,
            y=p.y,
            yaw=yaw,
            values=values,
            stopped=math.hypot(v.linear.x, v.linear.y) < 0.01 and abs(v.angular.z) < 0.02,
        )
        if self.sample is None or stamp != self.sample['stamp']:
            self.history.append(sample)
        if self.sample is not None and stamp == self.sample['stamp']:
            sample['receipt'] = self.sample['receipt']
        self.sample = sample

    def fresh(self, now, ros_ns, limit=0.35):
        return (
            self.sample is not None
            and now - self.sample['receipt'] <= limit
            and (-0.1 <= (ros_ns - self.sample['stamp']) / 1e9 <= limit)
        )

    def recovered(self, since, now, ros_ns, since_ros_ns=None):
        if not self.fresh(now, ros_ns):
            return False
        samples = []
        for sample in self.history:
            if sample['receipt'] < since:
                continue
            if since_ros_ns is not None and sample['stamp'] < since_ros_ns:
                continue
            if not sample['stopped'] or (
                samples
                and (
                    sample['receipt'] - samples[-1]['receipt'] > 0.35
                    or (sample['stamp'] - samples[-1]['stamp']) / 1e9 > 0.35
                )
            ):
                samples = []
            if sample['stopped']:
                samples.append(sample)
        return len(samples) >= 3 and samples[-1]['receipt'] - samples[0]['receipt'] >= 0.3


class ScanWindow:
    """Count distinct valid scans, never repeated loop checks or queued pre-pause scans."""

    def __init__(self):
        self.history = deque(maxlen=100)

    def observe(self, stamp, receipt, valid):
        if not self.history or stamp != self.history[-1]['stamp']:
            self.history.append(dict(stamp=stamp, receipt=receipt, valid=valid))

    def recovered(self, since, since_ros_ns, now, ros_ns):
        samples = []
        for sample in self.history:
            if sample['receipt'] < since or sample['stamp'] < since_ros_ns:
                continue
            if not sample['valid'] or (
                samples
                and (
                    sample['receipt'] - samples[-1]['receipt'] > 0.35
                    or (sample['stamp'] - samples[-1]['stamp']) / 1e9 > 0.35
                )
            ):
                samples = []
            if sample['valid']:
                samples.append(sample)
        return (
            len(samples) >= 3
            and samples[-1]['receipt'] - samples[0]['receipt'] >= 0.3
            and (
                now - samples[-1]['receipt'] <= 0.35
                and -0.1 <= (ros_ns - samples[-1]['stamp']) / 1e9 <= 0.35
            )
        )


class RouteFeedbackPause:
    """No epoch rebase, target change, command replay, or motion while feedback is old."""

    def __init__(self, runner, emit, cancel):
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data

        self.runner, self.emit, self.cancel = runner, emit, cancel
        self.sources = {
            topic: OdomEpoch()
            for topic in ('/rosbot_xl_base_controller/odom', '/odometry/filtered')
        }
        self.identities = {}
        self.used, self.pause_id, self.generation = 0.0, 0, 0
        self.active = False
        self.scan_stamp = None
        self.scan_frame = None
        self.scan_window = ScanWindow()
        self.command_owner = None
        self.command_identity = None
        self.subscriptions = [
            runner.node.create_subscription(
                Odometry,
                topic,
                lambda msg, source=source: source.receive(msg, time.monotonic()),
                qos_profile_sensor_data,
            )
            for topic, source in self.sources.items()
        ]
        # Baseline is strict: recovery is enabled only after both live sources were bound.
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            runner.pump()
            self.cancel()
            self.check_faults(bind=True)
            if all(s.fresh(time.monotonic(), self.ros_ns()) for s in self.sources.values()):
                if self.scan_ready():
                    break
        else:
            raise RuntimeError('Route feedback baseline unavailable')
        self.original_pump = runner.pump
        runner.pump = self.pump
        runner.exclusive = self.exclusive

    def exclusive(self):
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            self.pump()
        self.runner.assert_exclusive()

    def ros_ns(self):
        return self.runner.node.get_clock().now().nanoseconds

    def check_faults(self, bind=False):
        r = self.runner
        if r.invalid:
            raise RuntimeError(r.invalid)
        if r.child is not None and r.child.poll() is None:
            raise RuntimeError('Feedback pause cannot share a child controller')
        if r.client.service_is_ready():
            raise RuntimeError('Competing step controller')
        for topic, source in self.sources.items():
            if source.error:
                raise RuntimeError(f'{topic}: {source.error}')
            if source.sample is not None and (self.ros_ns() - source.sample['stamp']) / 1e9 < -0.1:
                raise RuntimeError(f'{topic}: future odometry timestamp')
            pubs = r.node.get_publishers_info_by_topic(topic)
            if len(pubs) != 1:
                raise RuntimeError(f'{topic}: requires exactly one publisher')
            identity = bytes(pubs[0].endpoint_gid)
            if topic not in self.identities and bind:
                self.identities[topic] = identity
            if self.identities.get(topic) != identity:
                raise RuntimeError(f'{topic}: publisher identity changed')
        self.check_command_owner()

    def check_command_owner(self):
        r = self.runner
        pubs = r.node.get_publishers_info_by_topic('/cmd_vel')
        if len(pubs) != (1 if r.velocity_pub is not None else 0):
            raise RuntimeError('Competing cmd_vel publisher')
        if pubs:
            pub = pubs[0]
            if pub.node_name != r.node.get_name() or pub.node_namespace != r.node.get_namespace():
                raise RuntimeError('cmd_vel publisher is not the route owner')
            identity = bytes(pub.endpoint_gid)
            if self.command_owner is not r.velocity_pub:
                self.command_owner, self.command_identity = r.velocity_pub, identity
            if identity != self.command_identity:
                raise RuntimeError('cmd_vel publisher identity changed')

    def scan_ready(self):
        r = self.runner
        scan = r.observer.scan
        if scan is None:
            return False
        stamp = scan.header.stamp.sec * 1_000_000_000 + scan.header.stamp.nanosec
        if stamp <= 0 or (self.ros_ns() - stamp) / 1e9 < -0.1:
            raise RuntimeError('Invalid/future scan timestamp')
        if self.scan_stamp is not None and stamp < self.scan_stamp:
            raise RuntimeError('Scan timestamp moved backward')
        if self.scan_frame is not None and scan.header.frame_id != self.scan_frame:
            raise RuntimeError('Scan frame changed')
        self.scan_stamp, self.scan_frame = stamp, scan.header.frame_id
        if not all(math.isfinite(v) for v in (scan.range_min, scan.range_max)) or not (
            0 < scan.range_min < scan.range_max
        ):
            raise RuntimeError('Invalid scan range bounds')
        enough_ranges = (
            sum(math.isfinite(v) and scan.range_min <= v <= scan.range_max for v in scan.ranges)
            >= 20
        )
        if not enough_ranges:
            self.scan_window.observe(stamp, r.observer.received, False)
            return False
        if not -0.1 <= (self.ros_ns() - stamp) / 1e9 <= 0.8 or (
            time.monotonic() - r.observer.received > 0.8
        ):
            return False
        try:
            r.geometry()  # Stamped TF and scan geometry; never infer clearance from absence.
        except RuntimeError as error:
            if isinstance(error.__cause__, ValueError):
                raise RuntimeError(f'Invalid scan geometry/TF: {error.__cause__}') from error
            if 'geometry/TF unavailable' in str(error):
                self.scan_window.observe(stamp, r.observer.received, False)
                return False
            raise
        fresh_recovery = -0.1 <= (self.ros_ns() - stamp) / 1e9 <= 0.35 and (
            time.monotonic() - r.observer.received <= 0.35
        )
        self.scan_window.observe(stamp, r.observer.received, fresh_recovery)
        return True

    def scan_recovered(self, since, since_ros_ns, now, ros_ns):
        return self.scan_window.recovered(since, since_ros_ns, now, ros_ns)

    def ready(self):
        now, ros_ns = time.monotonic(), self.ros_ns()
        scan_ready = self.scan_ready()
        return all(s.fresh(now, ros_ns) for s in self.sources.values()) and scan_ready

    def pump(self):
        self.original_pump()
        if not self.active:
            self.check()

    def check(self):
        self.cancel()
        self.check_faults()
        if self.ready():
            return False
        return self.recover()

    def recover(self):
        r = self.runner
        started = time.monotonic()
        started_ros_ns = self.ros_ns()
        budget = min(5.0, 10.0 - self.used)
        if budget <= 0:
            raise RuntimeError('Feedback cumulative pause budget exhausted')
        owned = r.velocity_pub is not None
        anchors = {
            topic: dict(source.sample) if source.sample is not None else None
            for topic, source in self.sources.items()
        }
        if owned:
            r.send((0.0, 0.0, 0.0))
        elif r.guarded_command is None or any(r.guarded_command[0]):
            raise RuntimeError('Feedback pause requires confirmed prior zero command')
        self.pause_id += 1
        self.emit(
            'feedback_pause_started',
            pause_id=self.pause_id,
            budget_s=budget,
            owns_publisher=owned,
            reason='required source freshness/TF expired',
        )
        self.active = True
        next_zero = started + 0.05
        try:
            while time.monotonic() - started < budget:
                if owned and time.monotonic() >= next_zero:
                    r.send((0.0, 0.0, 0.0))
                    next_zero = time.monotonic() + 0.05
                self.original_pump()
                self.cancel()
                self.check_faults()
                for topic, source in self.sources.items():
                    anchor, sample = anchors[topic], source.sample
                    if (
                        anchor is not None
                        and sample is not None
                        and math.hypot(sample['x'] - anchor['x'], sample['y'] - anchor['y']) > 0.04
                    ):
                        raise RuntimeError(f'{topic}: feedback pause drift exceeded 4 cm')
                now, ros_ns = time.monotonic(), self.ros_ns()
                if (
                    self.ready()
                    and self.scan_recovered(started, started_ros_ns, now, ros_ns)
                    and all(
                        s.recovered(started, now, ros_ns, started_ros_ns)
                        for s in self.sources.values()
                    )
                ):
                    self.used += now - started
                    self.generation += 1
                    self.emit(
                        'feedback_pause_completed',
                        pause_id=self.pause_id,
                        elapsed_s=now - started,
                        cumulative_s=self.used,
                    )
                    return True
            raise RuntimeError('Feedback pause timeout; route canceled')
        finally:
            self.active = False
            if owned:
                r.send((0.0, 0.0, 0.0))

    def observation(self, phase):
        """Use the valid current scan; avoid the separate two-second new-scan waiter."""
        from rclpy.time import Time
        from survey_distances import summarize_scan

        self.check()
        r, scan = self.runner, self.runner.observer.scan
        tf = r.observer.buffer.lookup_transform(
            'base_link', scan.header.frame_id, Time.from_msg(scan.header.stamp)
        )
        q, t = tf.transform.rotation, tf.transform.translation
        return dict(
            phase=phase,
            status='ok',
            topic='/scan_filtered',
            direction_frame='base_link',
            range_origin='laser sensor; NOT body clearance',
            scan_frame=scan.header.frame_id,
            stamp_ns=self.scan_stamp,
            distances=summarize_scan(scan, (q.x, q.y, q.z, q.w)),
            sensor_translation_m=[t.x, t.y, t.z],
            sensor_rotation_xyzw=[q.x, q.y, q.z, q.w],
        )
