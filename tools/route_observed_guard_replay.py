#!/usr/bin/env python3
"""Closed-bag numerical observed-guard audit; no ROS initialization or publishers.

Bag receipt time is a replay clock, NOT monotonic callback receipt evidence.
Every output is non-executable, even when supplied provenance files exist.
"""

import argparse
from bisect import bisect_right
from collections import Counter
from dataclasses import asdict, fields
import hashlib
import json
import math
from pathlib import Path
import sqlite3

from action_plan import Pose
from route_laser_guard import GuardPolicy
from route_laser_runtime import scheduled_rays
from route_laser_shadow_replay import check_closed, load_bag, paired_velocity, stamp
from route_observed_guard import ObservedFeedback, ObservedGuard, ObservedPolicy, ObservedScan
from route_scan_binding import scan_identity
from wall_geometry import scan_points


TOPICS = ('/scan', '/scan_filtered', '/odometry/filtered', '/cmd_vel')
EPOCH = 'offline_numerical_epoch_unverified'


def file_evidence(path):
    """Hash only actual supplied bytes; absence is explicit, never synthesized."""
    if path is None:
        return None
    return {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def read_policy(path):
    value = json.loads(path.read_text())
    if set(value) != {f.name for f in fields(ObservedPolicy)}:
        raise ValueError('Every observed policy field must be explicit')
    if set(value['obstacle']) != {f.name for f in fields(GuardPolicy)}:
        raise ValueError('Every obstacle policy field must be explicit')
    policy = ObservedPolicy(**{**value, 'obstacle': GuardPolicy(**value['obstacle'])})
    policy.validate()
    return policy


def unique_records(records):
    """Validate original serialized repeats before the existing reader's dictionary use.

    Input tuples are (topic, bag_receipt_ns, source_stamp_ns, payload_digest, message).
    First receipt is retained; any backward stamp or changed duplicate rejects the bag.
    Commands have no source stamp and remain an ordered receipt stream.
    """
    streams = {topic: [] for topic in TOPICS}
    seen, last, repeats = {}, {}, Counter()
    for topic, receipt, source, digest, message in sorted(records, key=lambda row: row[1]):
        if topic != '/cmd_vel':
            if source < last.get(topic, source):
                raise ValueError(f'{topic}: source stamp moved backward')
            last[topic] = source
            key = topic, source
            if key in seen:
                if seen[key] != digest:
                    raise ValueError(f'{topic}: changed payload at duplicate stamp')
                repeats[topic] += 1
                continue
            seen[key] = digest
        streams[topic].append((receipt, source, message))
    return streams, dict(repeats)


def check_static_transform(known, transform):
    """Reject a changing recorded static mount rather than applying the last to all scans."""
    t, q = transform.transform.translation, transform.transform.rotation
    value = (transform.header.frame_id, t.x, t.y, t.z, q.x, q.y, q.z, q.w)
    key = transform.child_frame_id
    if key in known and known[key] != value:
        raise ValueError('Recorded static TF changed for ' + key)
    known[key] = value


def recorded_streams(path):
    """Retain receipts and duplicate checks missing from the shared geometry reader."""
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage

    types = dict(zip(TOPICS, (LaserScan, LaserScan, Odometry, Twist)))
    records, static = [], {}
    for file in sorted(path.glob('*.db3')):
        with sqlite3.connect(file.resolve().as_uri() + '?mode=ro', uri=True) as db:
            topics = dict(db.execute('select id, name from topics'))
            for topic, receipt, data in db.execute(
                'select topic_id, timestamp, data from messages order by timestamp, id'
            ):
                name = topics[topic]
                if name == '/tf_static':
                    for tf in deserialize_message(data, TFMessage).transforms:
                        check_static_transform(static, tf)
                if name not in types:
                    continue
                message = deserialize_message(data, types[name])
                source = None if name == '/cmd_vel' else stamp(message)
                records.append((name, receipt, source, hashlib.sha256(data).digest(), message))
    return unique_records(records)


def odom_pose(message):
    p, q = message.pose.pose.position, message.pose.pose.orientation
    values = (p.x, p.y, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(x) for x in values):
        raise ValueError('Nonfinite recorded odom')
    if abs(math.sqrt(sum(v * v for v in values[2:])) - 1) > 0.01:
        raise ValueError('Invalid recorded odom quaternion')
    return Pose(p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2)))


def latest_before(rows, receipt):
    index = bisect_right([row[0] for row in rows], receipt) - 1
    if index < 0:
        raise ValueError('No recorded message before evaluation receipt')
    return rows[index]


def candidate_at(commands, receipt, candidate, policy):
    if candidate is not None:
        return tuple(candidate), None
    cmd_time, _, message = latest_before(commands, receipt)
    if (receipt - cmd_time) / 1e9 > policy.receipt_age_s:
        raise ValueError('Recorded command older than explicit receipt age limit')
    return (message.linear.x, message.linear.y, message.angular.z), cmd_time


def paired_input(filtered, raw, odoms, transform, policy):
    """One immutable numerical snapshot, retaining every raw and filtered valid point."""
    if raw is None:
        raise ValueError('Raw scan with matching source stamp unavailable')
    receipt, source, scan = filtered
    if scan_identity(scan) != scan_identity(raw[2]):
        raise ValueError('Raw/filtered identity mismatch')
    now = max(receipt, raw[0])
    current_row = latest_before(odoms, now)
    available = {row[1]: row[2] for row in odoms if row[0] <= now}
    _, paired_stamp = paired_velocity(source, available, sorted(available))
    if abs(source - paired_stamp) / 1e9 > policy.pair_age_s:
        raise ValueError('Odom pairing exceeds explicit policy')
    current = current_row[2]
    twist = current.twist.twist
    measured = (twist.linear.x, twist.linear.y, twist.angular.z)
    points = tuple(
        (x, y) for message in (raw[2], scan) for x, y, _ in scan_points(message, transform)[0]
    )
    snapshot = ObservedScan(
        points,
        scheduled_rays(scan, transform, 0),
        abs(scan.angle_increment),
        odom_pose(available[paired_stamp]),
        source,
        paired_stamp,
        receipt / 1e9,
        EPOCH,
    )
    feedback = ObservedFeedback(
        odom_pose(current),
        current_row[1],
        current_row[0] / 1e9,
        now,
        now / 1e9,
        EPOCH,
        (current.header.frame_id, current.child_frame_id),
        True,
    )
    return snapshot, feedback, measured, now


def evaluate_streams(streams, transform_for, policy, candidate=None):
    """Once per new filtered stamp, never once per repeated/zero command."""
    guard = ObservedGuard(policy, EPOCH)
    raw = {row[1]: row for row in streams['/scan']}
    for row in streams['/scan_filtered']:
        detail = dict(scan_stamp_ns=row[1], filtered_bag_receipt_ns=row[0])
        try:
            snapshot, feedback, measured, now = paired_input(
                row, raw.get(row[1]), streams['/odometry/filtered'], transform_for(row[2]), policy
            )
            command, cmd_time = candidate_at(streams['/cmd_vel'], now, candidate, policy)
            result = guard.decide(snapshot, feedback, command, measured)
            detail.update(
                decision=asdict(result),
                candidate=command,
                measured=measured,
                recorded_cmd_bag_receipt_ns=cmd_time,
                evaluation_bag_receipt_ns=now,
                raw_bag_receipt_ns=raw[row[1]][0],
                odom_stamp_ns=feedback.stamp_ns,
                paired_odom_stamp_ns=snapshot.paired_odom_stamp_ns,
                point_count=len(snapshot.points),
            )
        except (ValueError, RuntimeError) as error:
            detail.update(decision=dict(state='unavailable', reason=str(error), command=(0, 0, 0)))
        detail.update(enforced=False, executable=False, certified_free=False, numerical_only=True)
        yield detail


def report(args):
    from rclpy.time import Time

    closure = check_closed(args.bag, args.confirm_closed)
    policy = read_policy(args.policy)
    streams, duplicates = recorded_streams(args.bag)
    # Shared reader supplies existing static TF-chain geometry. Duplicate source data
    # has already been checked; its dictionaries are deliberately not used here.
    buffer, _, _, _ = load_bag(args.bag)

    def transform_for(scan):
        try:
            return buffer.lookup_transform('base_link', scan.header.frame_id, Time()).transform
        except Exception as error:
            raise ValueError('Recorded static TF unavailable: ' + str(error)) from error

    states = Counter()
    with args.output.open('x') as output:
        for row in evaluate_streams(streams, transform_for, policy, args.candidate):
            states[row['decision']['state']] += 1
            output.write(json.dumps(row, allow_nan=False) + '\n')
    return dict(
        closure=closure,
        mode='observed_guard_closed_bag_numerical_only',
        policy=asdict(policy),
        policy_file=file_evidence(args.policy),
        filter_file=file_evidence(args.filter_params),
        source_file=file_evidence(args.source_manifest),
        metadata=file_evidence(args.bag / 'metadata.yaml'),
        frames=sum(states.values()),
        decisions=dict(states),
        duplicate_messages=duplicates,
        candidate=args.candidate,
        output=str(args.output),
        enforced=False,
        executable=False,
        active_filter_verified=False,
        source_epoch_verified=False,
        live_freshness_verified=False,
        certified_free=False,
        limitations=[
            'Bag receipt is replay time, not monotonic callback evidence',
            'Static TF only; mounting changes and source GIDs are not verified',
            'Optional file hashes identify supplied bytes, not active configuration',
            'Quality uses filtered ray directions; all raw and filtered points are retained',
            'Recorded cmd is the last at/before pair receipt, not a reconstructed controller input',
        ],
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--policy', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--filter-params', type=Path)
    parser.add_argument('--source-manifest', type=Path)
    parser.add_argument('--confirm-closed', action='store_true')
    choice = parser.add_mutually_exclusive_group(required=True)
    choice.add_argument('--candidate', type=float, nargs=3, metavar=('VX', 'VY', 'WZ'))
    choice.add_argument('--recorded-cmd', action='store_true')
    args = parser.parse_args()
    try:
        result = report(args)
    except (ValueError, OSError) as error:
        print(json.dumps({'error': str(error), 'executable': False}))
        return 2
    print(json.dumps(result, allow_nan=False))
    return 0 if result['frames'] and not result['decisions'].get('unavailable') else 2


if __name__ == '__main__':
    raise SystemExit(main())
