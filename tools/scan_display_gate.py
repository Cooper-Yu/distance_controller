#!/usr/bin/env python3
"""Delay visualization scans until complete scan-time TF is available.

Display-only output. Never use this queue as a motion-control sensor input.
Original stamps and all scan fields are preserved. Missing TF expires scans.
"""

import math
import time
from collections import deque

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class ScanDisplayGate(Node):
    """Bounded queue with a short grace period for other TF subscribers."""

    def __init__(self):
        super().__init__('scan_display_gate')
        self.buffer = Buffer(cache_time=Duration(seconds=10))
        self.listener = TransformListener(self.buffer, self)
        self.pending = deque()
        self.sent = 0
        self.dropped = 0
        self.publisher = self.create_publisher(LaserScan, '/scan_rviz', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/scan_filtered', self.receive, qos_profile_sensor_data
        )
        self.timer = self.create_timer(0.02, self.flush)
        self.report_timer = self.create_timer(10.0, self.report)
        self.get_logger().info(
            'Display only: /scan_filtered -> /scan_rviz; waiting for map/odom TF'
        )

    def receive(self, scan):
        if not scan.ranges or not math.isfinite(scan.time_increment) or scan.time_increment < 0:
            self.dropped += 1
            return
        if len(self.pending) >= 30:
            self.pending.popleft()
            self.dropped += 1
        self.pending.append([time.monotonic(), scan, None])

    def ready(self, scan):
        start = Time.from_msg(scan.header.stamp)
        end = start + Duration(seconds=(len(scan.ranges) - 1) * scan.time_increment)
        return all(
            self.buffer.can_transform(frame, scan.header.frame_id, stamp)
            for frame in ('odom', 'map')
            for stamp in (start, end)
        )

    def flush(self):
        now = time.monotonic()
        for item in self.pending:
            if self.ready(item[1]):
                if item[2] is None:
                    item[2] = now
            else:
                item[2] = None
        while self.pending:
            received, scan, ready_at = self.pending[0]
            if now - received > 1.0:
                self.pending.popleft()
                self.dropped += 1
                continue
            if ready_at is None:
                break
            if now - ready_at < 0.10:
                break
            self.publisher.publish(scan)
            self.sent += 1
            self.pending.popleft()

    def report(self):
        self.get_logger().info(
            f'sent={self.sent} expired_or_invalid={self.dropped} pending={len(self.pending)}'
        )


def main():
    rclpy.init()
    node = ScanDisplayGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
