"""Offline full-turn footprint diagnostic; never publishes or grants motion permission."""

import argparse
import json
import math
from pathlib import Path

from route_laser_guard import RADIUS
from wall_geometry import point_clearance


def inspect_turn(points, angle_deg, margin_m=0.02):
    """Bound clearance to supplied finite points throughout a fixed-center rotation.

    The same chassis/wheel footprint is used as the live point guard. A 1 mm
    between-sample bound covers footprint motion, not sensor/model uncertainty.
    Invalid or missing returns, unseen heights and center drift remain unobserved.
    """
    if not math.isfinite(angle_deg) or not 0 < abs(angle_deg) <= 180:
        raise ValueError('Turn angle must be nonzero and within 180 degrees')
    if not math.isfinite(margin_m) or margin_m < 0.02:
        raise ValueError('Diagnostic margin must be at least 2 cm')
    if not points or any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in points):
        raise ValueError('Require all finite XY points from the supplied snapshot')
    angle = math.radians(angle_deg)
    count = max(1, math.ceil(RADIUS * abs(angle) / 0.002))
    allowance = RADIUS * abs(angle) / count / 2
    minimum, limiting, at_angle, first = math.inf, None, None, None
    start_gap = min(point_clearance(*p) for p in points)
    for i in range(count + 1):
        yaw = angle * i / count
        c, s = math.cos(yaw), math.sin(yaw)
        for x, y in points:
            gap = point_clearance(c * x + s * y, -s * x + c * y) - allowance
            if gap < minimum:
                minimum, limiting, at_angle = gap, (x, y), math.degrees(yaw)
            if gap < margin_m and first is None:
                first = math.degrees(yaw)
    return dict(
        diagnostic_only=True,
        motion_authorized=False,
        certified_free=False,
        center_motion_assumed_m=0.0,
        supplied_points=len(points),
        angle_deg=angle_deg,
        margin_m=margin_m,
        sample_count=count + 1,
        discretization_allowance_m=allowance,
        initial_point_gap_m=start_gap,
        minimum_point_gap_lower_bound_m=minimum,
        limiting_point_base_xy=limiting,
        minimum_at_angle_deg=at_angle,
        first_sample_below_margin_deg=first,
        finite_points_preserve_margin=minimum >= margin_m,
        unknown_returns='not assessed; cannot infer free space',
    )


def load_event(path, line_number):
    """Select an explicit audit line containing unsampled guard points; never ICP clouds."""
    if line_number < 1:
        raise ValueError('Audit line numbers start at 1')
    with Path(path).open() as stream:
        for number, line in enumerate(stream, 1):
            if number == line_number:
                event = json.loads(line)
                if 'points_base_xy' not in event:
                    raise ValueError('Selected event has no full guard point snapshot')
                return event['points_base_xy']
    raise ValueError('Audit line does not exist')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit', type=Path, required=True)
    parser.add_argument('--line', type=int, required=True)
    parser.add_argument(
        '--angle-deg', type=float, required=True, help='left positive, right negative'
    )
    args = parser.parse_args()
    result = inspect_turn(load_event(args.audit, args.line), args.angle_deg)
    result['source'] = dict(path=str(args.audit.resolve()), line=args.line)
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
