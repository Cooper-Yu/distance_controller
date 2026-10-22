"""Shadow-only coverage evidence. Missing returns never become synthetic free points.

Finite-point sweep constraints and unobserved angular apertures are deliberately
separate outputs. Apertures are geometric diagnostics, not permission thresholds.
"""

from collections import Counter
from dataclasses import asdict
import math
from statistics import median

from route_laser_guard import GuardPolicy, RADIUS, required_gap, select_command
from wall_geometry import point_clearance

TAU = 2 * math.pi


def range_kind(value, minimum, maximum):
    """Keep distinct sensor encodings without assigning an undocumented free range."""
    if math.isnan(value):
        return 'nan'
    if value == math.inf:
        return 'positive_inf'
    if value == -math.inf:
        return 'negative_inf'
    if value < minimum:
        return 'below_min'
    if value > maximum:
        return 'above_max'
    return 'finite'


def paired_kinds(scan, raw=None):
    """Label exact matching rays only; filter-created gaps are not assumed self returns."""
    matched = (
        raw is not None
        and all(
            getattr(scan, field) == getattr(raw, field)
            for field in ('header', 'angle_min', 'angle_increment', 'range_min', 'range_max')
        )
        and len(scan.ranges) == len(raw.ranges)
        and all(
            getattr(scan, f, None) == getattr(raw, f, None)
            for f in ('angle_max', 'time_increment', 'scan_time')
        )
    )
    kinds = []
    for i, value in enumerate(scan.ranges):
        kind = range_kind(value, scan.range_min, scan.range_max)
        if matched and kind != 'finite':
            original = range_kind(raw.ranges[i], raw.range_min, raw.range_max)
            kind = 'filtered_finite' if original == 'finite' else 'raw_' + original
        kinds.append(kind)
    return tuple(kinds), matched


def aperture(radius, angle):
    """Maximum transverse width at/below radius; wide sectors saturate at diameter."""
    return 2 * radius * math.sin(min(angle, math.pi) / 2)


def angular_evidence(rays, scenarios, audit_radius):
    """Report holes between finite returns, including missing scheduled sectors.

    Rays are (bearing, range-or-None, kind) from one compensated laser origin.
    A finite endpoint range is an observation, not a bound on a hidden obstacle.
    """
    if not rays:
        return dict(state='no_rays', certified_free=False, gaps=[])
    ordered = sorted((a % TAU, distance, kind) for a, distance, kind in rays)
    angles = [a for a, _, _ in ordered]
    steps = [b - a for a, b in zip(angles, (*angles[1:], angles[0] + TAU))]
    typical = median(step for step in steps if step > 1e-12) if any(steps) else TAU
    finite = [i for i, (_, distance, kind) in enumerate(ordered) if kind == 'finite']
    gaps = []
    maximum = 0.0
    for index, left in enumerate(finite):
        right = finite[(index + 1) % len(finite)]
        end_index = right if right > left else right + len(ordered)
        start = angles[left]
        end = angles[right] + (TAU if right <= left else 0)
        width = end - start
        maximum = max(maximum, width)
        hidden = [ordered[i % len(ordered)][2] for i in range(left + 1, end_index)]
        absent = max(steps[i % len(steps)] for i in range(left, end_index)) > 1.5 * typical + 1e-9
        if not hidden and not absent:
            continue
        gaps.append(
            dict(
                start_rad=start,
                end_rad=end,
                width_deg=math.degrees(width),
                aperture_at_audit_radius_m=aperture(audit_radius, width),
                invalid_count=len(hidden),
                kinds=dict(Counter(hidden)),
                missing_scheduled_sector=absent,
                motion_relevant=any(required_gap(start, end, v) for v in scenarios),
                flank_ranges_m=[ordered[left][1], ordered[right][1]],
                hidden_obstacle_distance_lower_bound_m=None,
            )
        )
    if not finite:
        maximum = TAU
        gaps.append(
            dict(
                start_rad=0.0,
                end_rad=TAU,
                width_deg=360.0,
                aperture_at_audit_radius_m=2 * audit_radius,
                invalid_count=len(ordered),
                kinds=dict(Counter(k for _, _, k in ordered)),
                missing_scheduled_sector=max(steps) > 1.5 * typical + 1e-9,
                motion_relevant=any(any(v) for v in scenarios),
                flank_ranges_m=[],
                hidden_obstacle_distance_lower_bound_m=None,
            )
        )
    return dict(
        state='gaps_present' if gaps else 'finite_sampled',
        certified_free=False,
        scheduled_count=len(ordered),
        finite_count=len(finite),
        finite_fraction=len(finite) / len(ordered),
        kinds=dict(Counter(k for _, _, k in ordered)),
        typical_step_deg=math.degrees(typical),
        largest_finite_spacing_deg=math.degrees(maximum),
        largest_aperture_at_audit_radius_m=aperture(audit_radius, maximum),
        finite_rays_reaching_audit_radius=sum(
            kind == 'finite' and distance >= audit_radius for _, distance, kind in ordered
        ),
        gaps=gaps,
    )


def diagnose(snapshot, rays, candidate, measured, sensor_origin=(0.0, 0.0), policy=GuardPolicy()):
    """Finite obstacles constrain motion even when coverage has a small unknown hole.

    This result is NEVER an executable guard decision. No aperture acceptance
    threshold or minimum obstacle size has been qualified on the physical robot.
    """
    policy.validate()
    if any(len(v) != 3 or not all(math.isfinite(x) for x in v) for v in (candidate, measured)):
        raise ValueError('Invalid diagnostic velocity')
    if not all(math.isfinite(v) for v in sensor_origin):
        raise ValueError('Invalid diagnostic sensor origin')
    if any(not math.isfinite(a) or (d is not None and not math.isfinite(d)) for a, d, _ in rays):
        raise ValueError('Invalid diagnostic ray geometry')
    if any(
        (kind == 'finite') != (distance is not None and distance > 0) for _, distance, kind in rays
    ):
        raise ValueError('Inconsistent diagnostic ray range and kind')
    translation = policy.horizon_s * max(math.hypot(*v[:2]) for v in (candidate, measured))
    # Every rotated box fits within RADIUS. Constant-twist translation is bounded
    # by |v_xy| * horizon; origin offset is measured in the current base frame.
    radius = RADIUS + translation + policy.hard_m + math.hypot(*sensor_origin)
    observed, nearest, nearest_gap = None, None, None
    if snapshot.points:
        if any(len(p) != 2 or not all(math.isfinite(x) for x in p) for p in snapshot.points):
            raise ValueError('Invalid diagnostic obstacle geometry')
        observed = asdict(select_command(snapshot.points, candidate, measured, policy))
        nearest = min(snapshot.points, key=lambda p: point_clearance(*p))
        nearest_gap = point_clearance(*nearest)
    return dict(
        mode='coverage_shadow',
        enforced=False,
        executable=False,
        certified_free=False,
        observed_point_constraint=observed,
        observed_point_count=len(snapshot.points),
        nearest_observed_point=nearest,
        nearest_observed_clearance_m=nearest_gap,
        audit_radius_m=radius,
        sensor_origin_base_xy=sensor_origin,
        coverage=angular_evidence(rays, (candidate, measured), radius),
        proposed_command=candidate,
        applied_command=candidate,
        measured_velocity=measured,
        policy=asdict(policy),
        limitation='No hidden-obstacle distance bound; no aperture acceptance threshold',
    )
