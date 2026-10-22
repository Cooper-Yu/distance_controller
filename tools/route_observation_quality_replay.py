#!/usr/bin/env python3
"""Closed-bag observation-quality survey, never a motion or live freshness approval."""

import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path

from route_laser_coverage import paired_kinds
from route_laser_guard import GuardPolicy, select_command
from route_laser_runtime import scheduled_rays
from route_laser_shadow_replay import load_bag, paired_velocity
from route_observation_quality import QualityPolicy, QualityTracker
from route_laser_shadow_replay import check_closed
from wall_geometry import scan_points

CANDIDATES = {
    'forward': (0.04, 0.0, 0.0),
    'backward': (-0.04, 0.0, 0.0),
    'left': (0.0, 0.04, 0.0),
    'right': (0.0, -0.04, 0.0),
    'turn_left': (0.0, 0.0, 0.1),
    'turn_right': (0.0, 0.0, -0.1),
    'forward_right': (0.03, -0.03, 0.0),
}


def main():
    from rclpy.time import Time

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--confirm-closed', required=True, action='store_true')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--minimum-fraction', type=float, default=0.75)
    parser.add_argument('--maximum-gap-deg', type=float, default=10)
    parser.add_argument('--persistence-s', type=float, default=0.3)
    args = parser.parse_args()
    policy = QualityPolicy(
        minimum_fraction=args.minimum_fraction,
        maximum_gap_deg=args.maximum_gap_deg,
        persistence_s=args.persistence_s,
    )
    policy.validate()
    closure = check_closed(args.bag, args.confirm_closed)
    buffer, scans, raw, odom = load_bag(args.bag)
    trackers = {name: QualityTracker(policy) for name in CANDIDATES}
    summary = {
        name: dict(
            states=Counter(),
            finite_constraint_states=Counter(),
            fractions=[],
            gaps=[],
            maximum_bad_duration_s=0.0,
        )
        for name in CANDIDATES
    }
    failed = 0
    with args.output.open('x') as output:
        for timestamp, scan in sorted(scans.items()):
            try:
                tf = buffer.lookup_transform('base_link', scan.header.frame_id, Time()).transform
                points, _ = scan_points(scan, tf)
                directions = scheduled_rays(scan, tf, 0)
                kinds, matched = paired_kinds(scan, raw.get(timestamp))
                rays = tuple(
                    (a, float(r) if valid else None, k)
                    for (a, valid), r, k in zip(directions, scan.ranges, kinds)
                )
                measured, _ = paired_velocity(timestamp, odom, sorted(odom))
                for name, candidate in CANDIDATES.items():
                    report = trackers[name].observe(
                        timestamp / 1e9,
                        rays,
                        scan.angle_increment,
                        candidate,
                        measured,
                        'offline_unverified_epoch',
                    )
                    constraint = select_command(
                        tuple((x, y) for x, y, _ in points), candidate, measured, GuardPolicy()
                    )
                    report['finite_point_constraint'] = asdict(constraint)
                    report.update(
                        scan_stamp_ns=timestamp,
                        candidate_name=name,
                        raw_pair_matched=matched,
                        epoch_verified=False,
                        timing='header-only persistence; no live receipt/freshness proof',
                    )
                    output.write(json.dumps(report, allow_nan=False) + '\n')
                    s = summary[name]
                    s['states'][report['state']] += 1
                    s['finite_constraint_states'][constraint.state] += 1
                    s['fractions'].append(report['angular_finite_fraction'])
                    s['gaps'].append(report['maximum_unknown_deg'])
                    s['maximum_bad_duration_s'] = max(
                        s['maximum_bad_duration_s'], report['bad_duration_s']
                    )
            except Exception as error:
                failed += 1
                # Do not bridge a frame whose pairing or geometry was unavailable.
                trackers = {name: QualityTracker(policy) for name in CANDIDATES}
                output.write(
                    json.dumps(dict(scan_stamp_ns=timestamp, error=str(error), executable=False))
                    + '\n'
                )
    for s in summary.values():
        for key in ('fractions', 'gaps'):
            values = s.pop(key)
            s[key + '_min_max'] = [min(values), max(values)] if values else None
    report = dict(
        closure=closure,
        frames=len(scans),
        failed_frames=failed,
        candidates=summary,
        calibrated=False,
        executable=False,
        policy=asdict(policy),
    )
    target = args.output.with_suffix('.summary.json')
    with target.open('x') as output:
        json.dump(report, output, indent=2, allow_nan=False)
    print(json.dumps(report, allow_nan=False))
    return 0 if scans and not failed else 2


if __name__ == '__main__':
    raise SystemExit(main())
