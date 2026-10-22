#!/usr/bin/env python3
"""Bounded read-only odometry/reset timeline; temporal association is not causation."""

import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import re
import signal
import time
import shutil
from common import singleton, write_json


class Watch:
    def __init__(self, output, limit):
        self.output, self.limit = output, limit
        self.stream = output.open('x')
        self.begin = time.monotonic()
        self.bytes = 0
        self.full = False
        self.counts = Counter()
        self.latest, self.identities, self.max_step = {}, {}, {}
        self.interval_bounds = {}
        self.last_nonzero = None
        self.last_command = None
        self.last_moving = {}
        self.rosout_window = (self.begin, 0)

    def idle(self):
        now = time.monotonic()
        return dict(
            observed_idle_s=now - (self.last_nonzero or self.begin),
            idle_is_lower_bound=self.last_nonzero is None,
            last_nonzero_elapsed_s=None
            if self.last_nonzero is None
            else self.last_nonzero - self.begin,
            last_command=self.last_command,
        )

    def emit(self, event, **fields):
        self.counts[event] += 1
        row = dict(
            event=event,
            wall_s=time.time(),
            elapsed_s=time.monotonic() - self.begin,
            **fields,
        )
        line = json.dumps(row, ensure_ascii=True, allow_nan=False, separators=(',', ':')) + '\n'
        size = len(line.encode())
        if self.bytes + size > self.limit:
            self.full = True
            return
        self.stream.write(line)
        self.stream.flush()
        self.bytes += size

    def odom(self, topic, msg):
        now = time.monotonic()
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        numbers = [p.x, p.y, q.x, q.y, q.z, q.w, v.linear.x, v.linear.y, v.angular.z]
        if not all(math.isfinite(n) for n in numbers):
            self.emit('nonfinite_odom', topic=topic)
            return
        if abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) > 0.01:
            self.emit('invalid_odom', topic=topic, reason='quaternion_norm')
            return
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        # Calibration is a real-robot wall-clock contract, never simulation time.
        source_age = time.time() - stamp / 1e9
        if stamp <= 0 or not -0.1 <= source_age <= 0.5:
            self.emit(
                'invalid_odom', topic=topic, reason='source_timestamp', source_age_s=source_age
            )
        state = dict(
            stamp_ns=stamp,
            frame=msg.header.frame_id,
            child=msg.child_frame_id,
            pose=[p.x, p.y, yaw],
            velocity=[v.linear.x, v.linear.y, v.angular.z],
            receipt_elapsed_s=now - self.begin,
            receipt_wall_s=time.time(),
        )
        bounds = self.interval_bounds.setdefault(
            topic, {'min': state['pose'][:], 'max': state['pose'][:]}
        )
        bounds['min'] = [min(a, b) for a, b in zip(bounds['min'], state['pose'])]
        bounds['max'] = [max(a, b) for a, b in zip(bounds['max'], state['pose'])]
        previous = self.latest.get(topic)
        self.counts['odom:' + topic] += 1
        if math.hypot(v.linear.x, v.linear.y) > 0.005 or abs(v.angular.z) > 0.01:
            self.last_moving[topic] = now
        if previous:
            dx = math.dist(state['pose'][:2], previous['pose'][:2])
            dyaw = math.remainder(yaw - previous['pose'][2], 2 * math.pi)
            dt = (stamp - previous['stamp_ns']) / 1e9
            self.max_step[topic] = max(self.max_step.get(topic, 0), dx)
            if dt > 0.5 or now - self.begin - previous['receipt_elapsed_s'] > 0.5:
                self.emit('feedback_gap', topic=topic, stamp_delta_s=dt)
            reasons = []
            if dx > 0.2 or abs(dyaw) > 0.25:
                reasons.append('pose_jump')
            if dt <= 0 or dt > 5:
                reasons.append('stamp_jump')
            if (state['frame'], state['child']) != (
                previous['frame'],
                previous['child'],
            ):
                reasons.append('frame_change')
            if reasons:
                self.emit(
                    'odom_change',
                    topic=topic,
                    reasons=reasons,
                    before=previous,
                    after=state,
                    step_m=dx,
                    step_yaw=dyaw,
                    stamp_delta_s=dt,
                    receipt_delta_s=now - self.begin - previous['receipt_elapsed_s'],
                    command_idle=self.idle(),
                )
        self.latest[topic] = state

    def command(self, msg):
        values = [
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        ]
        if not all(math.isfinite(n) for n in values):
            self.emit('nonfinite_cmd')
            return
        active = any(abs(n) > 1e-6 for n in values)
        before = self.last_command
        self.last_command = dict(
            values=values, elapsed_s=time.monotonic() - self.begin, nonzero=active
        )
        self.counts['cmd_vel'] += 1
        if active:
            self.last_nonzero = time.monotonic()
        if before is None or before['nonzero'] != active:
            self.emit('command_state', command=self.last_command)

    def rosout(self, msg):
        if not re.search(
            r'ekf|filter|controller|reconnect|disconnect|reset|restart|time.jump|clock|odom',
            msg.name + ' ' + msg.msg,
            re.I,
        ):
            return
        now = time.monotonic()
        start, count = self.rosout_window
        if now - start >= 2:
            start, count = now, 0
        self.rosout_window = (start, count + 1)
        if count >= 10:
            self.counts['rosout_suppressed'] += 1
            return
        self.emit(
            'rosout',
            name=msg.name,
            level=msg.level,
            message=msg.msg[:512],
            stamp_ns=msg.stamp.sec * 1_000_000_000 + msg.stamp.nanosec,
        )

    def snapshot(self, node, topics):
        for topic in topics:
            identities = sorted(
                (bytes(i.endpoint_gid).hex(), i.node_namespace, i.node_name)
                for i in node.get_publishers_info_by_topic(topic)
            )
            if topic not in self.identities or identities != self.identities[topic]:
                self.emit(
                    'publisher_graph',
                    topic=topic,
                    previous=self.identities.get(topic),
                    current=identities,
                    command_idle=self.idle(),
                )
                self.identities[topic] = identities
        now = time.monotonic()
        self.emit(
            'sample',
            latest=self.latest,
            interval_bounds=self.interval_bounds,
            publisher_gids=self.identities,
            receipt_age_s={
                k: now - self.begin - v['receipt_elapsed_s'] for k, v in self.latest.items()
            },
            odom_idle_s={k: now - self.last_moving.get(k, self.begin) for k in self.latest},
            odom_idle_lower_bound={k: k not in self.last_moving for k in self.latest},
            counts=dict(self.counts),
            command_idle=self.idle(),
        )

        self.interval_bounds = {}

    def close(self, reason):
        self.emit('finished', reason=reason, command_idle=self.idle())
        self.stream.close()
        result = dict(
            reason=reason,
            duration_s=time.monotonic() - self.begin,
            bytes=self.bytes,
            limit_bytes=self.limit,
            counts=dict(self.counts),
            latest=self.latest,
            max_step_m=self.max_step,
            identities=self.identities,
            command_idle=self.idle(),
            causal_conclusion='not established; observations only',
        )
        write_json(self.output.with_suffix('.summary.json'), result)
        return result


def main():  # noqa: PLR0915 - bounded ROS lifecycle
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=1800)
    parser.add_argument('--max-bytes', type=int, default=4_000_000)
    parser.add_argument('--base-topic', default='/odom')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 1800:
        parser.error('seconds must be 1..1800')
    if not 65536 <= args.max_bytes <= 8_000_000:
        parser.error('max-bytes must be 65536..8000000')
    if args.output.suffix != '.jsonl' or args.output.with_suffix('.summary.json').exists():
        parser.error('use a new .jsonl and summary path')
    owner_lock = singleton('odom_reset_watch')
    if shutil.disk_usage(args.output.parent).free < 300 * 1024 * 1024:
        parser.error('less than 300 MiB free')
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rcl_interfaces.msg import Log
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('read_only_odom_epoch_watch')
    watch = Watch(args.output, args.max_bytes)
    stopped = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopped.append(True))
    topics = ['/odometry/filtered', args.base_topic, '/cmd_vel']
    for topic in topics[:2]:
        node.create_subscription(
            Odometry,
            topic,
            lambda msg, t=topic: watch.odom(t, msg),
            qos_profile_sensor_data,
        )
    node.create_subscription(Twist, '/cmd_vel', watch.command, qos_profile_sensor_data)
    node.create_subscription(Log, '/rosout', watch.rosout, qos_profile_sensor_data)
    reason = 'exception'
    try:
        watch.emit(
            'started',
            topics=topics + ['/rosout'],
            pid=os.getpid(),
            process_start_ticks=Path('/proc/self/stat').read_text().rsplit(')', 1)[1].split()[19],
            schema=2,
            seconds=args.seconds,
            max_bytes=args.max_bytes,
            note='read only; no speed publisher; causality unproven',
        )
        tick = watch.begin
        while not stopped and not watch.full and time.monotonic() - watch.begin < args.seconds:
            if shutil.disk_usage(args.output.parent).free < 300 * 1024 * 1024:
                reason = 'disk_floor'
                break
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() >= tick:
                watch.snapshot(node, topics)
                tick = time.monotonic() + 2
        if reason != 'disk_floor':
            reason = 'file_limit' if watch.full else 'signal' if stopped else 'duration'
    finally:
        result = watch.close(reason)
        node.destroy_node()
        rclpy.shutdown()
        owner_lock.close()
        print('STOP', reason, 'SECONDS', round(result['duration_s'], 1))
        print('ODOM_EVENTS', result['counts'].get('odom_change', 0))
        print('GID_EVENTS', result['counts'].get('publisher_graph', 0))
        print('BYTES', result['bytes'])
        print('JSONL', args.output)
        print('SUMMARY', args.output.with_suffix('.summary.json'))


if __name__ == '__main__':
    main()
