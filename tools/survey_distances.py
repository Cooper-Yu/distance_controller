"""Read-only four-direction LaserScan observations in base_link directions.

Reported ranges are sensor-origin ray lengths, not vehicle-edge clearance.
TF supplies direction mapping; stale scans and unavailable TF produce no distances.
"""

import math
import statistics
import time

DIRECTIONS = {'front': 0.0, 'left': math.pi / 2, 'right': -math.pi / 2, 'rear': math.pi}


def summarize_scan(scan, quaternion, half_window=math.radians(5)):
    """Read scan rays and sensor rotation; return per-direction finite range statistics.

    Input scan is a LaserScan-like object. Quaternion rotates laser directions into
    base_link. Output ranges remain distances from the sensor, in meters. No body
    dimensions or origin-offset subtraction are applied. Poor coverage is explicit.
    """
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid TF quaternion')
    x, y, z, w = (v / norm for v in quaternion)
    if abs(1 - 2 * (x * x + y * y)) < math.cos(math.radians(5)):
        raise ValueError('Scan plane is not approximately horizontal')
    if (
        not math.isfinite(scan.angle_min)
        or not math.isfinite(scan.angle_increment)
        or scan.angle_increment == 0
        or not math.isfinite(scan.range_min)
        or not math.isfinite(scan.range_max)
        or scan.range_max <= scan.range_min
    ):
        raise ValueError('Invalid scan geometry/range bounds')
    buckets = {name: [] for name in DIRECTIONS}
    counts = dict.fromkeys(DIRECTIONS, 0)
    for i, distance in enumerate(scan.ranges):
        angle = scan.angle_min + i * scan.angle_increment
        c, s = math.cos(angle), math.sin(angle)
        bx = (1 - 2 * (y * y + z * z)) * c + 2 * (x * y - z * w) * s
        by = 2 * (x * y + z * w) * c + (1 - 2 * (x * x + z * z)) * s
        body_angle = math.atan2(by, bx)
        for name, center in DIRECTIONS.items():
            delta = math.atan2(math.sin(body_angle - center), math.cos(body_angle - center))
            if abs(delta) <= half_window + 1e-9:
                counts[name] += 1
                if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
                    buckets[name].append(float(distance))
    result = {}
    for name, values in buckets.items():
        total, valid = counts[name], len(values)
        usable = valid >= 3 and valid >= total / 2
        result[name] = {
            'status': 'ok' if usable else 'insufficient_returns',
            'valid': valid,
            'total': total,
            'min_m': min(values) if usable else None,
            'median_m': statistics.median(values) if usable else None,
            'max_m': max(values) if usable else None,
        }
    return result


class DistanceObserver:
    """Consume /scan_filtered and TF through the caller's ROS node/executor."""

    def __init__(self, node):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        from tf2_ros import Buffer, TransformListener

        self.node = node
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self.scan = None
        self.received = None
        self.sub = node.create_subscription(
            LaserScan, '/scan_filtered', self.receive, qos_profile_sensor_data
        )

    def receive(self, msg):
        """Cache the newest delivered scan and its monotonic receipt time."""
        self.scan, self.received = msg, time.monotonic()

    def snapshot(self, phase, timeout=2.0):
        """Wait boundedly for a new stamped scan; missing observations never imply clear space."""
        import rclpy
        from rclpy.time import Time
        from tf2_ros import TransformException

        start = self.node.get_clock().now().nanoseconds
        deadline = time.monotonic() + timeout
        result = {
            'phase': phase,
            'topic': '/scan_filtered',
            'direction_frame': 'base_link',
            'range_origin': 'laser sensor; NOT body clearance',
            'half_window_deg': 5.0,
            'status': 'no_scan',
            'distances': None,
        }
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            scan = self.scan
            if scan is None:
                continue
            stamp = scan.header.stamp.sec * 1_000_000_000 + scan.header.stamp.nanosec
            age = (self.node.get_clock().now().nanoseconds - stamp) / 1e9
            result.update(
                scan_frame=scan.header.frame_id,
                stamp_ns=stamp,
                stamp_age_s=age,
                receipt_age_s=time.monotonic() - self.received,
            )
            if stamp < start or not -0.1 <= age <= 0.5 or result['receipt_age_s'] > 0.5:
                result['status'] = 'stale_or_not_new_scan'
                continue
            try:
                tf = self.buffer.lookup_transform(
                    'base_link', scan.header.frame_id, Time.from_msg(scan.header.stamp)
                )
                q, t = tf.transform.rotation, tf.transform.translation
                distances = summarize_scan(scan, (q.x, q.y, q.z, q.w))
                result.update(
                    status='ok',
                    distances=distances,
                    sensor_translation_m=[t.x, t.y, t.z],
                    sensor_rotation_xyzw=[q.x, q.y, q.z, q.w],
                )
                result.pop('reason', None)
                return result
            except TransformException as error:
                result.update(status='tf_unavailable', reason=str(error))
            except ValueError as error:
                result.update(status='invalid_geometry', reason=str(error))
                return result
        return result


def print_snapshot(result):
    """Display exactly the values also saved in the per-step JSON audit."""
    print(f'LASER {result["phase"]}: {result["status"]} (sensor range, meters)', flush=True)
    for name, item in (result['distances'] or {}).items():
        prefix = f'  {name.upper()}: valid={item["valid"]}/{item["total"]}'
        if item['status'] == 'ok':
            print(
                f'{prefix} min={item["min_m"]:.3f} median={item["median_m"]:.3f} '
                f'max={item["max_m"]:.3f}',
                flush=True,
            )
        else:
            print(f'{prefix} unavailable ({item["status"]})', flush=True)
