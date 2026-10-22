"""Pure per-ray raw/filtered provenance diagnostic; no ROS initialization or control.

A supplied box mask predicts geometric membership only. It does not identify
self returns, certify the active filter configuration, or establish free space.
"""

from collections import Counter
import hashlib
import json
import math

from wall_geometry import point_clearance

GEOMETRY = (
    'angle_min',
    'angle_max',
    'angle_increment',
    'time_increment',
    'scan_time',
    'range_min',
    'range_max',
)


def stamp_ns(stamp):
    if type(stamp.sec) is not int or type(stamp.nanosec) is not int:
        raise ValueError('Timestamp requires integer sec/nanosec')
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError('Invalid timestamp')
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def scan_identity(scan):
    stamp = stamp_ns(scan.header.stamp)
    values = {name: getattr(scan, name) for name in GEOMETRY}
    if not stamp or not scan.header.frame_id or not 1 <= len(scan.ranges) <= 4000:
        raise ValueError('Missing scan timestamp/frame/rays')
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values.values()):
        raise ValueError('Nonfinite scan geometry')
    if not (0 < scan.range_min < scan.range_max and scan.angle_increment != 0):
        raise ValueError('Invalid scan range/angle increment')
    if scan.time_increment < 0 or scan.scan_time < 0:
        raise ValueError('Negative scan timing')
    end = scan.angle_min + (len(scan.ranges) - 1) * scan.angle_increment
    # Float32 angle fields can differ slightly; one whole missing beam is rejected.
    if abs(end - scan.angle_max) > abs(scan.angle_increment) * 0.01 + 1e-6:
        raise ValueError('Ray count and angle bounds are inconsistent')
    return dict(stamp_ns=stamp, frame=scan.header.frame_id, count=len(scan.ranges), **values)


def validate_mask(manifest):
    if manifest.get('schema') != 1 or manifest.get('type') != 'laser_filters/LaserScanBoxFilter':
        raise ValueError('Explicit schema1 LaserScanBoxFilter manifest required')
    params = manifest.get('params', {})
    if params.get('box_frame') != 'base_link' or type(params.get('invert')) is not bool:
        raise ValueError('Explicit base_link mask and boolean invert required')
    for axis in 'xyz':
        lo, hi = params.get('min_' + axis), params.get('max_' + axis)
        if not all(type(v) in (int, float) and math.isfinite(v) for v in (lo, hi)) or lo >= hi:
            raise ValueError('Finite ordered mask bounds required')
    try:
        encoded = json.dumps(manifest, sort_keys=True, separators=(',', ':'), allow_nan=False)
    except (TypeError, ValueError) as error:
        raise ValueError('Filter manifest must be finite JSON') from error
    return params, hashlib.sha256(encoded.encode()).hexdigest()


def transform_binding(scan_tf, identity):
    stamp = stamp_ns(scan_tf.header.stamp)
    if (scan_tf.header.frame_id, scan_tf.child_frame_id) != ('base_link', identity['frame']):
        raise ValueError('TF must explicitly map scan frame into base_link')
    if stamp not in (0, identity['stamp_ns']):
        raise ValueError('Require static zero-stamp TF or exact scan-time TF')
    t, q = scan_tf.transform.translation, scan_tf.transform.rotation
    translation, quaternion = (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in (*translation, *quaternion)):
        raise ValueError('Nonfinite TF')
    norm = math.sqrt(sum(v * v for v in quaternion))
    if abs(norm - 1) > 0.01:
        raise ValueError('Invalid TF quaternion norm')
    normalized = tuple(v / norm for v in quaternion)
    return dict(
        stamp_ns=stamp,
        parent='base_link',
        child=identity['frame'],
        translation_xyz=translation,
        quaternion_xyzw=quaternion,
        normalized_quaternion_xyzw=normalized,
        timing='supplied_static' if stamp == 0 else 'scan_stamp_matched',
    )


def range_record(value, lower, upper):
    if type(value) not in (int, float):
        raise ValueError('Ranges must be numeric, including explicit Inf/NaN')
    if math.isnan(value):
        return dict(kind='nan', value='NaN')
    if math.isinf(value):
        return dict(
            kind='positive_inf' if value > 0 else 'negative_inf',
            value='+Inf' if value > 0 else '-Inf',
        )
    kind = 'below_min' if value < lower else 'above_max' if value > upper else 'finite_in_range'
    return dict(kind=kind, value=value)


def point_in_base(angle, distance, mount):
    x, y, z, w = mount['normalized_quaternion_xyzw']
    lx, ly = distance * math.cos(angle), distance * math.sin(angle)
    tx, ty, tz = mount['translation_xyz']
    return (
        tx + (1 - 2 * (y * y + z * z)) * lx + 2 * (x * y - z * w) * ly,
        ty + 2 * (x * y + z * w) * lx + (1 - 2 * (x * x + z * z)) * ly,
        tz + 2 * (x * z - y * w) * lx + 2 * (y * z + x * w) * ly,
    )


def bind_ray(index, raw, filtered, identity, mount, mask):
    angle = identity['angle_min'] + index * identity['angle_increment']
    rr = range_record(raw, identity['range_min'], identity['range_max'])
    fr = range_record(filtered, identity['range_min'], identity['range_max'])
    raw_valid, filtered_valid = rr['kind'] == 'finite_in_range', fr['kind'] == 'finite_in_range'
    category = (
        ('finite_retained' if raw == filtered else 'finite_changed')
        if (raw_valid and filtered_valid)
        else 'raw_finite_filtered_missing'
        if raw_valid
        else ('filtered_finite_raw_missing' if filtered_valid else 'both_missing')
    )
    point = point_in_base(angle, raw, mount) if raw_valid else None
    inside = (
        all(mask['min_' + a] < v < mask['max_' + a] for a, v in zip('xyz', point))
        if (point is not None)
        else None
    )
    return dict(
        index=index,
        laser_angle_rad=angle,
        raw=rr,
        filtered=fr,
        category=category,
        raw_point_base_xyz=point,
        filtered_point_base_xyz=point_in_base(angle, filtered, mount) if filtered_valid else None,
        raw_point_inside_filter_box=inside,
        supplied_box_would_remove=(inside != mask['invert']) if inside is not None else None,
        raw_planar_model_gap_m=point_clearance(*point[:2]) if point is not None else None,
        self_return_confirmed=False,
        unknown_beyond_observed_return=True,
    )


def bind_scan_pair(raw, filtered, scan_tf, filter_manifest, epoch):
    """Bind exact scan identities and preserve every ray; mismatches raise ValueError.

    Inputs are ROS-message-shaped objects plus a JSON mask manifest and explicit
    epoch identifier. The caller owns current GID/receipt/TF/config verification.
    No live freshness, obstacle classification or movement decision is made here.
    """
    if not isinstance(epoch, str) or not epoch.strip():
        raise ValueError('Explicit nonempty epoch identifier required')
    identity = scan_identity(raw)
    if identity != scan_identity(filtered):
        raise ValueError('Raw/filtered stamp, frame, count or geometry mismatch')
    mask, manifest_hash = validate_mask(filter_manifest)
    mount = transform_binding(scan_tf, identity)
    rays = [
        bind_ray(i, a, b, identity, mount, mask)
        for i, (a, b) in enumerate(zip(raw.ranges, filtered.ranges))
    ]
    return dict(
        schema=1,
        mode='scan_binding_shadow',
        enforced=False,
        executable=False,
        certified_free=False,
        epoch=epoch,
        identity=identity,
        transform=mount,
        filter_manifest_sha256=manifest_hash,
        filter_manifest=json.loads(json.dumps(filter_manifest)),
        active_filter_verified=False,
        live_freshness_verified=False,
        counts=dict(Counter(r['category'] for r in rays)),
        rays=rays,
        limitations=[
            'Mask membership is not self-return identity',
            'Inf/NaN never supplies free-space evidence',
            'All raw finite points retained, including model overlap',
            'Planar model gap does not establish physical collision',
        ],
    )
