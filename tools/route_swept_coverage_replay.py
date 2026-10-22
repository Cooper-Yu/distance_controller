#!/usr/bin/env python3
"""Closed sqlite bag -> shadow swept-demand evidence; never initialize or publish ROS.

The bag reader provides no per-message publisher identity/receipt-monotonic audit.
Therefore history is disabled per frame rather than assuming an unchanged epoch.
"""

import argparse
from collections import Counter
import json
import math
from pathlib import Path

from route_laser_shadow_replay import check_closed, load_bag, paired_velocity
from route_swept_coverage import CoverageHistory, CurrentState, LaserMount, Pose2, ScanEvidence


def mounting(tf):
    q = tf.rotation
    components = (q.x, q.y, q.z, q.w)
    norm = math.sqrt(sum(v * v for v in components))
    if not math.isfinite(norm) or abs(norm - 1) > 0.01:
        raise ValueError('Invalid recorded mounting quaternion')
    x, y, z, w = (v / norm for v in components)
    return LaserMount(
        tf.translation.x,
        tf.translation.y,
        tf.translation.z,
        math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
        math.asin(max(-1, min(1, 2 * (w * y - z * x)))),
        math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
    )


def odom_pose(message):
    p, q = message.pose.pose.position, message.pose.pose.orientation
    values = (q.x, q.y, q.z, q.w)
    norm = math.sqrt(sum(v * v for v in values))
    if not math.isfinite(norm) or abs(norm - 1) > 0.01:
        raise ValueError('Invalid recorded odometry quaternion')
    x, y, z, w = (v / norm for v in values)
    return Pose2(p.x, p.y, math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def frame_report(timestamp, scan, buffer, odom, times, candidate):
    from rclpy.time import Time

    measured, odom_ns = paired_velocity(timestamp, odom, times)
    transform = buffer.lookup_transform('base_link', scan.header.frame_id, Time()).transform
    pose = odom_pose(odom[odom_ns])
    epoch = 'offline_single_frame_' + str(timestamp)
    rays = tuple(
        (
            scan.angle_min + i * scan.angle_increment,
            float(r) if math.isfinite(r) and scan.range_min <= r <= scan.range_max else None,
        )
        for i, r in enumerate(scan.ranges)
    )
    evidence = ScanEvidence(
        timestamp,
        timestamp / 1e9,
        odom_ns,
        pose,
        mounting(transform),
        epoch,
        'odom',
        'base_link',
        scan.header.frame_id,
        scan.range_min,
        scan.range_max,
        scan.angle_increment,
        rays,
    )
    now_ns = max(timestamp, odom_ns)
    current = CurrentState(
        pose, odom_ns, now_ns / 1e9, now_ns, now_ns / 1e9, epoch, 'odom', 'base_link'
    )
    history = CoverageHistory()
    history.add(evidence)
    result = history.assess(current, candidate, measured)
    result.update(
        scan_stamp_ns=timestamp,
        paired_odom_stamp_ns=odom_ns,
        history_enabled=False,
        epoch_verified=False,
        evaluation='Offline counterfactual at paired odom; not original command replay',
        timing_evidence='Header-time surrogate; no live freshness or receipt evidence',
        unknown_ray_count=sum(r is None for _, r in rays),
        sensor_mount=vars(evidence.mount),
    )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--confirm-closed', action='store_true', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--candidate', type=float, nargs=3, required=True)
    parser.add_argument('--max-frames', type=int, default=0)
    args = parser.parse_args()
    closure = check_closed(args.bag, args.confirm_closed)
    buffer, scans, _, odom = load_bag(args.bag)
    counts = Counter()
    with args.output.open('x') as output:
        for index, (timestamp, scan) in enumerate(sorted(scans.items())):
            if args.max_frames and index >= args.max_frames:
                break
            try:
                report = frame_report(
                    timestamp, scan, buffer, odom, sorted(odom), tuple(args.candidate)
                )
            except Exception as error:
                report = dict(
                    state='unavailable',
                    reason=str(error),
                    scan_stamp_ns=timestamp,
                    enforced=False,
                    executable=False,
                    certified_free=False,
                )
            counts[report['state']] += 1
            output.write(json.dumps(report, allow_nan=False) + '\n')
    print(
        json.dumps(
            dict(
                closure=closure,
                states=dict(counts),
                output=str(args.output),
                history_enabled=False,
                enforced=False,
                executable=False,
            )
        )
    )
    return 0 if counts and counts['unavailable'] == 0 else 2


if __name__ == '__main__':
    raise SystemExit(main())
