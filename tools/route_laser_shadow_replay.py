#!/usr/bin/env python3
"""Read-only sqlite rosbag scan coverage audit; never initializes ROS or publishes.

The supplied candidate is counterfactual at each scan timestamp, not a replayed
command. Static lidar mounting and nearest bounded odometry are required.
"""

import argparse
from bisect import bisect_left
from collections import Counter
import json
from pathlib import Path
import sqlite3

from route_laser_coverage import diagnose, paired_kinds
from route_laser_guard import ScanSnapshot
from route_laser_runtime import scheduled_rays
from wall_geometry import scan_points


def check_closed(path, confirmed=False):
    """Session closure evidence, never a cross-host PID-liveness guess."""
    if not confirmed:
        raise ValueError('Explicit --confirm-closed required')
    if not path.is_dir() or not (path / 'metadata.yaml').is_file():
        raise ValueError('Closed bag directory with metadata.yaml required')
    if any(path.glob('*-wal')):
        raise ValueError('SQLite WAL present; require a closed recording')
    result_file = path.parent / 'result.json'
    verified = result_file.is_file()
    if verified and json.loads(result_file.read_text()).get('closed_metadata') is not True:
        raise ValueError('Recorder result does not confirm closed metadata')
    return dict(
        explicit_closed_confirmation=True,
        recorder_closed_metadata_verified=verified,
        closure_basis='session_result_and_metadata' if verified else 'user_confirmation_only',
        live_process_identity_verified=False,
    )


def stamp(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def load_bag(path):
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer

    buffer = Buffer()
    scans, raw, odom = {}, {}, {}
    files = sorted(path.glob('*.db3')) if path.is_dir() else [path]
    if not files:
        raise ValueError('No sqlite rosbag files')
    for file in files:
        with sqlite3.connect('file:' + str(file.resolve()) + '?mode=ro', uri=True) as db:
            topics = dict(db.execute('select id, name from topics'))
            for topic, data in db.execute('select topic_id, data from messages order by timestamp'):
                name = topics[topic]
                if name in ('/scan', '/scan_filtered'):
                    message = deserialize_message(data, LaserScan)
                    (raw if name == '/scan' else scans)[stamp(message)] = message
                elif name == '/odometry/filtered':
                    message = deserialize_message(data, Odometry)
                    odom[stamp(message)] = message
                elif name == '/tf_static':
                    for tf in deserialize_message(data, TFMessage).transforms:
                        buffer.set_transform_static(tf, 'recorded_bag')
    return buffer, scans, raw, odom


def paired_velocity(timestamp, odom, times):
    index = bisect_left(times, timestamp)
    candidates = times[max(0, index - 1) : index + 1]
    if not candidates:
        raise ValueError('No recorded odom')
    nearest = min(candidates, key=lambda value: abs(value - timestamp))
    if abs(nearest - timestamp) > 150_000_000:
        raise ValueError('Recorded odom outside 150 ms pairing limit')
    message = odom[nearest]
    if message.header.frame_id != 'odom' or message.child_frame_id != 'base_link':
        raise ValueError('Recorded odom/twist frame mismatch')
    twist = message.twist.twist
    return (twist.linear.x, twist.linear.y, twist.angular.z), nearest


def audit_frame(timestamp, scan, raw, buffer, odom, times, candidate):
    from rclpy.time import Time

    transform = buffer.lookup_transform('base_link', scan.header.frame_id, Time()).transform
    points, _ = scan_points(scan, transform)
    directions = scheduled_rays(scan, transform, 0.0)
    kinds, matched = paired_kinds(scan, raw)
    rays = tuple(
        (bearing, float(distance) if valid else None, kind)
        for (bearing, valid), distance, kind in zip(directions, scan.ranges, kinds)
    )
    velocity, odom_time = paired_velocity(timestamp, odom, times)
    snapshot = ScanSnapshot(tuple((x, y) for x, y, _ in points), directions)
    report = diagnose(
        snapshot, rays, candidate, velocity, (transform.translation.x, transform.translation.y)
    )
    # Keep the raw finite points as a separate evidence set: no assumed self-point deletion.
    if matched:
        raw_points, _ = scan_points(raw, transform)
        raw_snapshot = ScanSnapshot(tuple((x, y) for x, y, _ in raw_points), directions)
        raw_report = diagnose(
            raw_snapshot,
            rays,
            candidate,
            velocity,
            (transform.translation.x, transform.translation.y),
        )
        for field in (
            'observed_point_constraint',
            'nearest_observed_clearance_m',
            'nearest_observed_point',
        ):
            report['raw_' + field] = raw_report[field]
    report.update(
        scan_stamp_ns=timestamp,
        odom_stamp_ns=odom_time,
        raw_pair_matched=matched,
        evaluation='offline scan-time counterfactual; no ROS publishers',
    )
    return report


def bounds(values):
    return [min(values), max(values)] if values else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--confirm-closed', action='store_true', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument(
        '--candidate', nargs=3, type=float, required=True, metavar=('VX', 'VY', 'WZ')
    )
    args = parser.parse_args()
    closure = check_closed(args.bag, args.confirm_closed)
    buffer, scans, raw, odom = load_bag(args.bag)
    times = sorted(odom)
    states, raw_states = Counter(), Counter()
    fractions, widths, apertures = [], [], []
    paired = 0
    with args.output.open('x') as output:
        for timestamp, scan in sorted(scans.items()):
            try:
                report = audit_frame(
                    timestamp, scan, raw.get(timestamp), buffer, odom, times, tuple(args.candidate)
                )
                observed = report['observed_point_constraint']
                states[observed['state'] if observed else 'no_points'] += 1
                raw_observed = report.get('raw_observed_point_constraint')
                if raw_observed:
                    raw_states[raw_observed['state']] += 1
                coverage = report['coverage']
                fractions.append(coverage['finite_fraction'])
                widths.append(coverage['largest_finite_spacing_deg'])
                apertures.append(coverage['largest_aperture_at_audit_radius_m'])
                paired += report['raw_pair_matched']
            except Exception as error:
                report = dict(
                    scan_stamp_ns=timestamp,
                    state='unavailable',
                    reason=str(error),
                    enforced=False,
                    executable=False,
                    certified_free=False,
                )
                states['unavailable'] += 1
            output.write(json.dumps(report, allow_nan=False) + '\n')
    print(
        json.dumps(
            dict(
                closure=closure,
                frames=len(scans),
                observed_point_constraints=dict(states),
                raw_observed_point_constraints=dict(raw_states),
                raw_pairs=paired,
                finite_fraction_bounds=bounds(fractions),
                largest_gap_deg_bounds=bounds(widths),
                largest_aperture_m_bounds=bounds(apertures),
                output=str(args.output),
                enforced=False,
                certified_free=False,
            )
        )
    )
    return 0 if scans and states['unavailable'] == 0 else 2


if __name__ == '__main__':
    raise SystemExit(main())
