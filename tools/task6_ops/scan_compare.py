#!/usr/bin/env python3
"""Read-only closed-bag/current laser pose comparison. Never rebase or publish motion."""

import argparse
import json
import math
import time
import sqlite3
from pathlib import Path
import numpy as np
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from route_return_plan import load_source
from route_return_calibration import file_sha256, source_binding, validate_reference, watch_binding


def json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError('Unsupported JSON value: ' + type(value).__name__)


def save_report(path, report):
    # Encode before opening the target, then atomically replace it.
    text = json.dumps(report, indent=2, default=json_value)
    temp = path.with_suffix('.json.tmp')
    temp.write_text(text)
    temp.replace(path)


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def rot(a):
    return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])


def nearest(a, b):
    d = ((a[:, None, :] - b[None, :, :]) ** 2).sum(axis=2)
    j = d.argmin(axis=1)
    return np.sqrt(d[np.arange(len(a)), j]), j


def points(scan, tf):
    r = np.asarray(scan.ranges)
    a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
    valid = np.isfinite(r) & (r > max(0.08, scan.range_min)) & (r < min(6, scan.range_max))
    p = np.column_stack((r[valid] * np.cos(a[valid]), r[valid] * np.sin(a[valid])))
    p = p @ tf[:2, :2].T + tf[:2, 3]
    return p[:: max(1, int(np.ceil(len(p) / 240)))]


def registration(source, target, seed=(0, 0, 0)):
    a = seed[2]
    t = np.array(seed[:2], dtype=float)
    for _ in range(35):
        moved = source @ rot(a).T + t
        d, j = nearest(moved, target)
        keep = d < min(0.18, max(0.035, float(np.quantile(d, 0.75))))
        if keep.sum() < 25:
            break
        x = moved[keep]
        y = target[j[keep]]
        xc = x.mean(0)
        yc = y.mean(0)
        u, _, vt = np.linalg.svd((x - xc).T @ (y - yc))
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0:
            vt[-1] *= -1
            rotation = vt.T @ u.T
        da = math.atan2(rotation[1, 0], rotation[0, 0])
        dt = yc - rotation @ xc
        a = wrap(a + da)
        t = rotation @ t + dt
        if np.linalg.norm(dt) < 0.0001 and abs(da) < 0.0001:
            break
    moved = source @ rot(a).T + t
    ds, _ = nearest(moved, target)
    dt, _ = nearest(target, moved)
    all_d = np.r_[ds, dt]
    inside = all_d < 0.035
    residual = float(np.sqrt(np.mean(all_d[inside] ** 2))) if inside.any() else 1.0
    overlap = float(min(np.mean(ds < 0.035), np.mean(dt < 0.035)))
    return {
        'x': float(t[0]),
        'y': float(t[1]),
        'yaw': float(a),
        'rmse': residual,
        'overlap': overlap,
        'score': residual + 0.12 * (1 - overlap),
    }


def medoid(clouds):
    score = []
    for a in clouds:
        score.append(sum(float(np.median(nearest(a, b)[0])) for b in clouds))
    return clouds[int(np.argmin(score))]


def walls(p):
    rng = np.random.default_rng(22)
    remain = p.copy()
    lines = []
    for _ in range(4):
        best = None
        for __ in range(160):
            if len(remain) < 20:
                break
            a, b = remain[rng.choice(len(remain), 2, replace=False)]
            v = b - a
            if np.linalg.norm(v) < 0.15:
                continue
            v = v / np.linalg.norm(v)
            d = np.abs(np.cross(v, remain - a))
            m = d < 0.012
            if best is None or m.sum() > best.sum():
                best = m
        if best is None or best.sum() < 18:
            break
        q = remain[best]
        _, _, vt = np.linalg.svd(q - q.mean(0))
        v = vt[0]
        span = float(np.ptp(q @ v))
        angle = float(math.atan2(v[1], v[0]) % np.pi)
        if span >= 0.18:
            lines.append({'count': len(q), 'span': span, 'angle': angle})
        remain = remain[~best]
    good = any(abs(np.sin(a['angle'] - b['angle'])) > 0.5 for a in lines for b in lines)
    return lines, good


def compare(source, target):
    # Multi-start search is bounded, not a proof of global uniqueness.
    seeds = [
        (x, y, a)
        for a in np.deg2rad([-15, 0, 15])
        for x in [-0.12, 0, 0.12]
        for y in [-0.12, 0, 0.12]
    ]
    seeds += [
        (x, y, a)
        for a in np.deg2rad([-180, -90, 0, 90])
        for x, y in [(0, 0), (0.5, 0), (-0.5, 0), (0, 0.5), (0, -0.5)]
    ]
    fits = sorted([registration(source, target, s) for s in seeds], key=lambda x: x['score'])
    best = fits[0]
    different = [
        f
        for f in fits
        if np.hypot(f['x'] - best['x'], f['y'] - best['y']) > 0.06
        or abs(wrap(f['yaw'] - best['yaw'])) > np.deg2rad(5)
    ]
    ambiguity = any(f['overlap'] >= 0.75 and f['score'] <= best['score'] + 0.008 for f in different)
    lines, wall_ok = walls(target)
    _, source_wall_ok = walls(source)
    gates = []
    if best['overlap'] < 0.8:
        gates.append('overlap_below_0.80')
    if best['rmse'] > 0.018:
        gates.append('inlier_rmse_above_0.018m')
    if not wall_ok or not source_wall_ok:
        gates.append('two_nonparallel_walls_unverified')
    if ambiguity:
        gates.append('multiple_comparable_registrations')
    return {
        'best': best,
        'alternative': different[:3],
        'ambiguous': ambiguity,
        'walls': lines,
        'gates': gates,
        'search_scope': 'bounded multistart, not global place uniqueness',
    }


def transform(msg):
    q = msg.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    out = np.eye(4)
    out[:3, :3] = rotation
    out[:3, 3] = [msg.translation.x, msg.translation.y, msg.translation.z]
    return out


def chain(transforms, frame):
    edges = {}
    for t in transforms:
        a = t.header.frame_id.lstrip('/')
        b = t.child_frame_id.lstrip('/')
        matrix = transform(t.transform)
        edges.setdefault(a, []).append((b, matrix))
        edges.setdefault(b, []).append((a, np.linalg.inv(matrix)))
    todo = [('base_link', np.eye(4))]
    seen = set()
    while todo:
        name, matrix = todo.pop()
        if name == frame.lstrip('/'):
            return matrix
        if name in seen:
            continue
        seen.add(name)
        todo += [(b, matrix @ mount) for b, mount in edges.get(name, []) if b not in seen]
    raise RuntimeError('static base_link to scan transform unavailable')


def reference_odom(odoms):
    """Validate finite stopped odom before selecting a timestamp-matched sample."""
    xy = np.array([[o.pose.pose.position.x, o.pose.pose.position.y] for o in odoms])
    yaw, odom_stamps = [], []
    for o in odoms:
        q, v = o.pose.pose.orientation, o.twist.twist
        values = [
            o.pose.pose.position.x,
            o.pose.pose.position.y,
            q.x,
            q.y,
            q.z,
            q.w,
            v.linear.x,
            v.linear.y,
            v.angular.z,
        ]
        if (
            not all(math.isfinite(x) for x in values)
            or abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) > 0.01
            or o.header.frame_id != 'odom'
            or o.child_frame_id != 'base_link'
            or math.hypot(v.linear.x, v.linear.y) > 0.005
            or abs(v.angular.z) > 0.01
        ):
            raise RuntimeError('reference odom invalid or moving')
        yaw.append(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        odom_stamps.append(o.header.stamp.sec * 10**9 + o.header.stamp.nanosec)
    if any(not 0 < b - a <= 500_000_000 for a, b in zip(odom_stamps, odom_stamps[1:])):
        raise RuntimeError('reference odom timestamp discontinuity')
    return xy, yaw, odom_stamps


def closed_records(session, start, end):
    """Read a bounded completion-time window from closed, byte-bound SQLite files."""
    import yaml

    result = json.loads((session / 'result.json').read_text())
    if result.get('closed_metadata') is not True:
        raise RuntimeError('reference not closed')
    pid = (session / 'supervisor.pid').read_text().strip()
    try:
        cmd = Path('/proc', pid, 'cmdline').read_bytes()
    except OSError:
        cmd = b''
    if b'task6_writer_recorder.py' in cmd or b'evidence_recorder.py' in cmd:
        raise RuntimeError('reference recorder alive')
    bag = session / 'bag'
    meta = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    files = {name: file_sha256(session / name) for name in ('result.json', 'bag/metadata.yaml')}
    records = {n: [] for n in ['/scan_filtered', '/tf_static', '/odometry/filtered', '/cmd_vel']}
    for rel in meta['relative_file_paths']:
        db = (bag / rel).resolve(strict=True)
        if bag.resolve() not in db.parents:
            raise RuntimeError('database outside bag')
        files[str(db.relative_to(session))] = file_sha256(db)
        con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
        try:
            for name in records:
                lo, hi = (0, end) if name == '/tf_static' else (start, end)
                rows = con.execute(
                    'SELECT m.timestamp,m.data FROM messages m JOIN topics t ON t.id=m.topic_id '
                    'WHERE t.name=? AND m.timestamp BETWEEN ? AND ? ORDER BY m.timestamp',
                    (name, lo, hi),
                ).fetchall()
                records[name] += rows
        finally:
            con.close()
    return records, files


def closed_reference(session, binding):  # noqa: PLR0915 - bounded closed evidence read
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from tf2_msgs.msg import TFMessage

    completed = int(binding['completed_time'] * 1e9)
    start, end = completed - 500_000_000, completed + 3_000_000_000
    records, files = closed_records(session, start, end)
    scan_rows = sorted(records['/scan_filtered'])
    scans = [deserialize_message(d, LaserScan) for _, d in scan_rows]
    if len(scans) < 10:
        raise RuntimeError('reference needs >=10 scans at original completion boundary')
    scan_rows, scans = scan_rows[-30:], scans[-30:]
    stamps = [s.header.stamp.sec * 10**9 + s.header.stamp.nanosec for s in scans]
    if any(
        not binding['original_stamp_ns'] - 100_000_000
        <= stamp
        <= binding['original_stamp_ns'] + 3_500_000_000
        for stamp in stamps
    ):
        raise RuntimeError('reference scan time differs from original source epoch')
    if any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise RuntimeError('reference scan stamps not increasing')

    frames = {s.header.frame_id for s in scans}
    if len(frames) != 1:
        raise RuntimeError('reference scan frame changed')
    static = [
        t for _, d in records['/tf_static'] for t in deserialize_message(d, TFMessage).transforms
    ]
    mount = chain(static, scans[0].header.frame_id)
    odoms = [
        deserialize_message(d, Odometry)
        for t, d in sorted(records['/odometry/filtered'])
        if start <= t <= end
    ]
    if len(odoms) < 10:
        raise RuntimeError('reference stationary odom missing')
    xy, yaw, odom_stamps = reference_odom(odoms)
    span = float(np.linalg.norm(np.ptp(xy, axis=0)))
    yawspan = float(np.ptp(np.unwrap(yaw)))
    if span > 0.008 or yawspan > np.deg2rad(1):
        raise RuntimeError(f'reference final3s moving span={span:.4f} yaw={yawspan:.4f}')
    recent_cmd = [
        deserialize_message(d, Twist) for t, d in records['/cmd_vel'] if start <= t <= end
    ]
    command_values = [
        value
        for c in recent_cmd
        for value in (c.linear.x, c.linear.y, c.linear.z, c.angular.x, c.angular.y, c.angular.z)
    ]
    if not all(math.isfinite(v) and abs(v) <= 0.0001 for v in command_values):
        raise RuntimeError('reference last3s nonzero cmd_vel')
    clouds = [points(s, mount) for s in scans]
    selected = medoid(clouds)
    index = next(i for i, cloud in enumerate(clouds) if cloud is selected)
    paired = min(range(len(odoms)), key=lambda i: abs(odom_stamps[i] - stamps[index]))
    reference = {
        'odom_pose': {
            'x': float(xy[paired, 0]),
            'y': float(xy[paired, 1]),
            'yaw': float(yaw[paired]),
        },
        'source_binding': binding,
        'session': str(session),
        'files': files,
        'scan_count': len(scans),
        'odom_xy_span': span,
        'odom_yaw_span': yawspan,
        'scan_bag_ns': scan_rows[index][0],
        'scan_stamp_ns': stamps[index],
        'odom_stamp_ns': odom_stamps[paired],
        'window_start_bag_ns': scan_rows[0][0],
        'window_end_bag_ns': scan_rows[-1][0],
        'frame': scans[index].header.frame_id,
        'recent_cmd_count': len(recent_cmd),
    }
    validate_reference(reference, binding)
    if any(file_sha256(session / name) != value for name, value in files.items()):
        raise RuntimeError('reference bag changed during read')
    return selected, mount, reference


def paired_current(odoms, stamp):
    """Use the pose at the selected current scan, not merely the final callback pose."""
    paired = min(odoms, key=lambda o: abs(o['stamp_ns'] - stamp))
    if abs(paired['stamp_ns'] - stamp) > 50_000_000:
        raise RuntimeError('current selected scan/odom pairing exceeds 50 ms')
    return paired


def current_capture():  # noqa: PLR0915 - one bounded read-only sampling lifetime
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    n = rclpy.create_node('task6_scan_pose_diagnosis')
    scans = []
    raw = []
    tf = []
    seen = set()
    odoms = []
    commands = []
    started = time.time()

    def cb(msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        age = (n.get_clock().now().nanoseconds - stamp) / 1e9
        if -0.1 <= age <= 0.5 and stamp not in seen:
            seen.add(stamp)
            scans.append(msg)

    def odom_cb(msg):
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        values = [p.x, p.y, q.x, q.y, q.z, q.w, v.linear.x, v.linear.y, v.angular.z]
        if not all(math.isfinite(x) for x in values):
            return
        if abs(sum(x * x for x in [q.x, q.y, q.z, q.w]) - 1) > 0.01:
            return
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        age = (n.get_clock().now().nanoseconds - stamp) / 1e9
        if not -0.1 <= age <= 0.5:
            return
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        peers = n.get_publishers_info_by_topic('/odometry/filtered')
        gid = bytes(peers[0].endpoint_gid).hex() if len(peers) == 1 else ''
        odoms.append(
            dict(
                stamp_ns=stamp,
                pose=dict(x=p.x, y=p.y, yaw=yaw),
                frame=msg.header.frame_id,
                child=msg.child_frame_id,
                gid=gid,
                receipt=time.time(),
                velocity=[v.linear.x, v.linear.y, v.angular.z],
            )
        )

    def command_cb(msg):
        commands.append(
            [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]
        )

    n.create_subscription(LaserScan, '/scan_filtered', cb, qos_profile_sensor_data)
    n.create_subscription(LaserScan, '/scan', lambda m: raw.append(m), qos_profile_sensor_data)
    n.create_subscription(Odometry, '/odometry/filtered', odom_cb, qos_profile_sensor_data)
    n.create_subscription(Twist, '/cmd_vel', command_cb, qos_profile_sensor_data)
    n.create_subscription(
        TFMessage,
        '/tf_static',
        lambda m: tf.extend(m.transforms),
        QoSProfile(
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        ),
    )
    deadline = time.monotonic() + 15
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(n, timeout_sec=0.1)
            if len(scans) >= 10 and len(odoms) >= 10 and tf:
                break
        if len(scans) < 10 or len(odoms) < 10:
            raise RuntimeError('current fresh10frames/odom not received')
        gids = [
            bytes(i.endpoint_gid).hex()
            for i in n.get_publishers_info_by_topic('/odometry/filtered')
        ]
        if len(gids) != 1 or any(o['gid'] != gids[0] for o in odoms):
            raise RuntimeError('current odom identity changed or multiple publishers')
        if n.count_publishers('/cmd_vel') or any(any(abs(v) > 1e-6 for v in c) for c in commands):
            raise RuntimeError('current velocity source present')
        if len({s.header.frame_id for s in scans}) != 1:
            raise RuntimeError('current frame changed')
        last = odoms[-1]
        expected = np.array(list(last['pose'].values()))
        if any(o['frame'] != 'odom' or o['child'] != 'base_link' for o in odoms):
            raise RuntimeError('current odom frame changed')
        for o in odoms:
            p = o['pose']
            if np.hypot(p['x'] - expected[0], p['y'] - expected[1]) > 0.002 or abs(
                wrap(p['yaw'] - expected[2])
            ) > np.deg2rad(0.2):
                raise RuntimeError('current odom moved during scan capture')
            if np.hypot(*o['velocity'][:2]) > 0.005 or abs(o['velocity'][2]) > 0.01:
                raise RuntimeError('current odom velocity not stationary')
        if any(
            b['stamp_ns'] <= a['stamp_ns'] or b['receipt'] - a['receipt'] > 0.5
            for a, b in zip(odoms, odoms[1:])
        ):
            raise RuntimeError('current odom timestamp/gap invalid')
        mount = chain(tf, scans[0].header.frame_id)
        clouds = [points(s, mount) for s in scans[:10]]
        center = medoid(clouds)
        selected = next(i for i, cloud in enumerate(clouds) if cloud is center)
        selected_stamp = (
            scans[selected].header.stamp.sec * 10**9 + scans[selected].header.stamp.nanosec
        )
        paired = paired_current(odoms, selected_stamp)
        stability = [registration(p, center) for p in clouds]
        raw_cloud = points(raw[-1], chain(tf, raw[-1].header.frame_id)) if raw else np.empty((0, 2))
        epoch = {k: paired[k] for k in ['pose', 'stamp_ns', 'frame', 'child', 'gid']}
        epoch.update(
            capture_started_unix=started,
            captured_unix=last['receipt'],
            stable=True,
            odom_count=len(odoms),
            selected_scan_stamp_ns=selected_stamp,
        )
        return center, mount, raw_cloud, stability, scans[0].header.frame_id, epoch
    finally:
        n.destroy_node()
        rclpy.shutdown()


def main():  # noqa: PLR0915 - report assembly and CLI lifecycle
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session', type=Path, required=True)
    p.add_argument('--watch-log', type=Path, required=True)
    a = p.parse_args()
    out = Path.home() / ('task6_scan_compare_' + str(time.time_ns()))
    out.mkdir()
    report = {
        'schema': 2,
        'scope': 'diagnostic only; no motion/rebase/recovery authorization',
        'gates': [],
    }
    book_file = a.session / 'points.json'
    if book_file.exists():
        try:
            book = json.loads(book_file.read_text())
            point = book.get('points', {}).get('P12', {})
            report['book_p12'] = {
                'session_id': book.get('session_id'),
                'recorded': point.get('recorded'),
                'laser': point.get('laser'),
                'pose': point.get('pose'),
            }
        except Exception as e:
            report['book_read_error'] = str(e)
    try:
        source = load_source(a.session / 'points.json')
        binding = source_binding(a.session / 'points.json', source)
        ref, mount, reference = closed_reference(a.session.resolve(), binding)
        cur, current_mount, raw, stability, frame, epoch = current_capture()
        report['source_binding'] = binding
        report['epoch'] = epoch
        report['watch'] = watch_binding(a.watch_log, epoch['capture_started_unix'])
        report.update(compare(cur, ref))
        report['reference'] = reference
        report['current_frame'] = frame
        report['stability'] = stability
        from route_session_state import near
        from route_return_plan import pose

        if not near(pose(reference['odom_pose']), source.last_stop, 0.005, 0.01):
            report['gates'].append('reference_window_not_original_final_stop')
        report['tf_reference'] = mount.tolist()
        report['tf_current'] = current_mount.tolist()
        if not np.allclose(mount, current_mount, atol=0.001):
            report['gates'].append('static_tf_changed')
        if frame != reference['frame']:
            report['gates'].append('scan_frame_changed')
        if any(
            np.hypot(f['x'], f['y']) > 0.012 or abs(f['yaw']) > np.deg2rad(1) or f['overlap'] < 0.8
            for f in stability
        ):
            report['gates'].append('current10frames_not_stable')
        np.savez_compressed(out / 'points.npz', reference=ref, current=cur, raw_current=raw)
        b = report['best']
        report['transform_definition'] = (
            'p_reference=rotation(yaw)*p_current+[x,y]; current base pose in old reference base axes'
        )
        report['same_local_geometry_supported'] = (
            not report['gates']
            and np.hypot(b['x'], b['y']) <= 0.02
            and abs(b['yaw']) <= np.deg2rad(1)
        )
        print('DX %.4f DY %.4f YAW %.3fdeg' % (b['x'], b['y'], np.rad2deg(b['yaw'])))
        print('OVERLAP %.3f RMSE %.4fm' % (b['overlap'], b['rmse']))
        print('GATES ' + (','.join(report['gates']) or 'PASS')[:52])
        print('LOCAL_SAME ' + str(report['same_local_geometry_supported']))
    except Exception as e:
        report['gates'].append(type(e).__name__ + ': ' + str(e))
        print(('DIAG FAILED ' + str(e))[:60])
    save_report(out / 'report.json', report)
    laser = report.get('book_p12', {}).get('laser') or {}
    gaps = laser.get('body_wall_clearance', {})
    print(
        (
            'BOOK P12 F '
            + str(gaps.get('front', {}).get('gap'))
            + ' rotation '
            + str(gaps.get('right', {}).get('gap'))
        )[:60]
    )
    print('OUT ~/' + out.name)
    return 0 if not report['gates'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
