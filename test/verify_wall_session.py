"""Local-only ROS raycast-room fixture for wall sessions; never targets real hardware."""

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

os.environ['ROS_DOMAIN_ID'] = '181'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

ROOT = Path(__file__).resolve().parents[1]
OUT = Path('/tmp/wall_session_test') / str(time.time_ns())
OUT.mkdir(parents=True)
rclpy.init()
node = rclpy.create_node('wall_room_fixture')
odom_pub = node.create_publisher(Odometry, '/odometry/filtered', 10)
scan_pub = node.create_publisher(LaserScan, '/scan_filtered', 10)
pose = [0.0, 0.0, 0.0]
velocity = [0.0, 0.0, 0.0]
tf = TransformStamped()
tf.header.frame_id, tf.child_frame_id = 'base_link', 'laser'
tf.transform.translation.x = 0.02
tf.transform.translation.z = 0.173
tf.transform.rotation.z = 1.0
tf.transform.rotation.w = 0.0
broadcaster = StaticTransformBroadcaster(node)
broadcaster.sendTransform(tf)


def receive(msg):
    velocity[:] = [msg.linear.x, msg.linear.y, msg.angular.z]
    assert math.hypot(*velocity[:2]) <= math.hypot(0.06, 0.012) + 1e-6 and abs(velocity[2]) < 0.501


sub = node.create_subscription(Twist, '/cmd_vel', receive, 10)
last_scan = 0.0
scan_number = 0
bad_scan_start = None
return_scan_start = None


def inject_return_obstacle(values, now):
    """Inject a rear obstacle only once reverse motion starts, across zero-command pauses."""
    global return_scan_start
    return_mode = os.getenv('RETURN_LOSS_MODE', '')
    if return_mode and velocity[0] < -0.005 and return_scan_start is None:
        return_scan_start = now
    if return_scan_start is not None and (
        return_mode == 'permanent' or now - return_scan_start < 0.5
    ):
        # Rear pair at 1 cm body gap: always stop; transient disappears while stationary.
        values[360:362] = [0.20, 0.20]


def inject_narrow_patch(values, scan, now):
    """Synthetic narrow patch that requires reduced side correction while travelling forward."""
    if os.getenv('WALL_CASE') == 'side_limit' and 0.025 < pose[0] < 0.055:
        # A short narrow patch: left wall shifts inward, while right near points constrain correction.
        for i in range(len(values)):
            bearing = math.remainder(
                scan.angle_min + i * scan.angle_increment + math.pi, 2 * math.pi
            )
            if abs(bearing - math.pi / 2) < math.radians(32):
                values[i] -= 0.012 / math.sin(bearing)
        values[523:525] = [0.184, 0.184]

    inject_return_obstacle(values, now)


def tick(previous, scan_enabled=True, front=0.55):
    global last_scan, scan_number, bad_scan_start
    rclpy.spin_once(node, timeout_sec=0.008)
    now = time.monotonic()
    dt = min(now - previous, 0.03)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    pose[0] += (c * velocity[0] - s * velocity[1]) * dt
    pose[1] += (s * velocity[0] + c * velocity[1]) * dt
    pose[2] += velocity[2] * dt
    msg = Odometry()
    msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x, msg.pose.pose.position.y = pose[:2]
    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = (
        math.sin(pose[2] / 2),
        math.cos(pose[2] / 2),
    )
    msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z = velocity
    odom_pub.publish(msg)
    if scan_enabled and now - last_scan > float(os.getenv('WALL_SCAN_PERIOD', '0.07')):
        scan = LaserScan()
        scan.header.frame_id, scan.header.stamp = 'laser', msg.header.stamp
        scan.angle_min, scan.angle_increment, scan.angle_max = -math.pi, math.pi / 360, math.pi
        scan.range_min, scan.range_max = 0.15, 40.0
        x, y = pose[0] + 0.02 * c, pose[1] + 0.02 * s
        scan_number += 1
        noise = float(os.getenv('WALL_HEADING_NOISE', '0')) * (1 if scan_number % 2 else -1)
        values = []
        for i in range(721):
            a = scan.angle_min + i * scan.angle_increment + pose[2] + math.pi + noise
            dx, dy = math.cos(a), math.sin(a)
            candidates = []
            if abs(dx) > 1e-8:
                candidates.append(((front if dx > 0 else -0.3) - x) / dx)
            if abs(dy) > 1e-8:
                candidates.append(((0.3 if dy > 0 else -0.5) - y) / dy)
            distance = min(v for v in candidates if v > 0)
            values.append(distance if distance >= 0.15 else float('inf'))
        mode = os.getenv('WALL_LOSS_MODE', '')
        if mode and pose[0] > 0.025 and bad_scan_start is None:
            bad_scan_start = now
        if (
            mode
            and bad_scan_start is not None
            and (mode == 'permanent' or now - bad_scan_start < 0.6)
        ):
            for i in range(len(values)):
                bearing = scan.angle_min + i * scan.angle_increment + math.pi
                delta = math.atan2(math.sin(bearing - math.pi / 2), math.cos(bearing - math.pi / 2))
                if abs(delta) < math.radians(32) and i % 2 == 0:
                    values[i] += 0.15
        if os.getenv('WALL_SIDE_NOISE'):
            for i in range(len(values)):
                bearing = scan.angle_min + i * scan.angle_increment + math.pi
                delta = math.atan2(math.sin(bearing - math.pi / 2), math.cos(bearing - math.pi / 2))
                if abs(delta) < math.radians(35):
                    values[i] += 0.018 * ((i % 11) / 5 - 1) / abs(math.sin(bearing))
        inject_narrow_patch(values, scan, now)
        scan.ranges = values
        scan_pub.publish(scan)
        last_scan = now
    return now


def row(name, kind, value, follow='none', stop='distance', offset=0.0, bound=0.7, capture='none'):
    return dict(
        name=name,
        kind=kind,
        value=value,
        unit='deg' if kind == 'turn' else 'm',
        wall=dict(follow=follow, stop=stop, offset=offset, max_travel=bound, capture=capture),
    )


def run(name, rows, commands, prepare=False, stale=False, front=0.55, resume=None):
    if resume is None:
        pose[:] = [
            float(os.getenv('WALL_START_X', '0')),
            0.0,
            float(os.getenv('WALL_START_YAW', '0')),
        ]
    velocity[:] = [0.0, 0.0, 0.0]
    directory = OUT / name
    directory.mkdir()
    config = directory / 'route.json'
    config.write_text(json.dumps({'actions': rows}))
    with (directory / 'runtime.log').open('w') as log:
        proc = subprocess.Popen(
            [
                'python3',
                str(ROOT / 'tools/action_session.py'),
                *(
                    ['--resume', str(resume)]
                    if resume
                    else ['--route', str(config), '--start', 'prepare' if prepare else 'current']
                ),
            ],
            stdin=subprocess.PIPE,
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
            cwd=directory,
            start_new_session=True,
        )
        proc.stdin.write(commands)
        proc.stdin.close()
        deadline, previous = time.monotonic() + 150, time.monotonic()
        try:
            while proc.poll() is None and time.monotonic() < deadline:
                previous = tick(previous, not (stale and pose[0] > 0.025), front)
            assert proc.poll() is not None, 'Fixture timeout'
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                proc.wait(timeout=10)
        assert proc.returncode == 0, (directory / 'runtime.log').read_text()[-3500:]
    events = [
        json.loads(line)
        for line in next(directory.glob('action_session_*.jsonl')).read_text().splitlines()
    ]
    assert sum(abs(v) for v in velocity) < 1e-6
    print(name, [e['event'] for e in events], pose, flush=True)
    return events, (directory / 'runtime.log').read_text()


try:
    if os.getenv('WALL_CASE') == 'side_limit':
        events, log = run('side_limit', [row('f', 'forward', 0.12, 'left')], 'next\nquit\n')
        assert 'SIDE_CORRECTION_LIMITED' in log, log
        assert sum(e['event'] == 'completed' for e in events) == 1, log
        assert not any(e['event'] == 'incomplete' for e in events), log
        assert 0.10 < pose[0] < 0.14, pose
        print('PASS side limit', OUT, flush=True)
        raise SystemExit(0)
    if os.getenv('WALL_CASE') == 'return_recovery':
        for mode in ('transient', 'permanent'):
            return_scan_start = None
            os.environ['RETURN_LOSS_MODE'] = mode
            events, log = run(
                'return_' + mode, [row('f', 'forward', 0.08, 'left')], 'next\nback\nBACK\nquit\n'
            )
            assert 'RETURN_RECOVERING' in log, log
            if mode == 'transient':
                assert 'RETURN_RECOVERED' in log, log
                assert sum(e['event'] == 'returned' for e in events) == 1, log
                assert math.hypot(pose[0], pose[1]) < 0.03, pose
            else:
                assert 'RETURN_RECOVERY_TIMEOUT' in log, log
                assert not any(e['event'] == 'returned' for e in events), log
                assert any(e['event'] == 'incomplete' for e in events), log
        print('PASS return recovery', OUT, flush=True)
        raise SystemExit(0)
    if os.getenv('WALL_CASE') == 'directional_return':
        events, log = run(
            'directional_return', [row('f', 'forward', 0.08, 'left')], 'next\nback\nBACK\nquit\n'
        )
        assert sum(e['event'] == 'completed' for e in events) == 1, log
        assert sum(e['event'] == 'returned' for e in events) == 1, log
        assert math.hypot(pose[0], pose[1]) < 0.03, pose
        print('PASS directional return', OUT, flush=True)
        raise SystemExit(0)
    if os.getenv('WALL_CASE') == 'checkpoint':
        rows = [row('one', 'forward', 0.08, 'left'), row('two', 'forward', 0.06, 'left')]
        run('completed_save', rows, 'next\nquit\n')
        saved = next((OUT / 'completed_save').glob('*.checkpoint.json'))
        events, log = run('completed_restore', rows, 'SAME_ODOM\nnext\nquit\n', resume=saved)
        assert any(e['event'] == 'checkpoint_restored' for e in events), log
        assert sum(e['event'] == 'completed' for e in events) == 1, log
        assert 0.115 < pose[0] < 0.15, pose
        rows = [row('partial', 'forward', 0.12, 'left')]
        run('partial_save', rows, 'next\nquit\n', stale=True)
        saved = next((OUT / 'partial_save').glob('*.checkpoint.json'))
        data = json.loads(saved.read_text())
        assert data['partial'] and data['partial']['path_m'] > 0, data
        events, log = run(
            'partial_restore', rows, 'SAME_ODOM\nresume\nRESUME\nquit\n', resume=saved
        )
        assert sum(e['event'] == 'completed' for e in events) == 1, log
        assert 0.10 < pose[0] < 0.13, pose
        print('PASS checkpoint', OUT, flush=True)
        raise SystemExit(0)
    if os.getenv('WALL_CASE') == 'recovery':
        os.environ['WALL_LOSS_MODE'] = 'transient'
        events, log = run('transient_fit_loss', [row('f', 'forward', 0.08, 'left')], 'next\nquit\n')
        assert sum(e['event'] == 'completed' for e in events) == 1, log[-4000:]
        assert 'WALL_RECOVERED' in log, log[-4000:]
        bad_scan_start = None
        os.environ['WALL_LOSS_MODE'] = 'permanent'
        events, log = run(
            'persistent_fit_loss', [row('f', 'forward', 0.08, 'left')], 'next\nquit\n'
        )
        assert any(e['event'] == 'incomplete' for e in events), log[-4000:]
        assert 'WALL_RECOVERY_TIMEOUT' in log, log[-4000:]
        print('PASS recovery', OUT, flush=True)
        raise SystemExit(0)
    if os.getenv('WALL_CASE') == 'independent':
        rows = [row('independent', 'forward', 0.2, 'left', 'front')]
        stop_gap = float(os.getenv('WALL_STOP_GAP', '0.16'))
        rows[0]['wall'].update(follow_clearance=0.12, stop_clearance=stop_gap)
        events, log = run('independent_clearances', rows, 'next\nquit\n')
        reports = [e['data']['wall_result'] for e in events if e['event'] == 'wall_result']
        assert len(reports) == 1, log[-3500:]
        report = reports[0]
        assert abs(report['wall_clearances_m']['left'] - 0.12) <= 0.008
        assert abs(report['wall_clearances_m']['front'] - stop_gap) <= 0.008
        assert report['follow_target_m'] == 0.12 and report['stop_target_m'] == stop_gap
        assert abs(report['reference_m'] - 0.14) < 0.001
        print('PASS independent', OUT, flush=True)
        raise SystemExit(0)
    if os.getenv('WALL_CASE') == 'prepare':
        events, log = run(
            'right_heading_left_reference',
            [row('f', 'forward', 0.03, 'left')],
            'next\nquit\n',
            prepare=True,
        )
        assert 'right-wall alignment' in log
        assert sum(e['event'] == 'completed' for e in events) == 1, log[-3500:]
        print('PASS preparation', OUT, flush=True)
        raise SystemExit(0)
    rows = [
        row('forward', 'forward', 0.08, 'left'),
        row('front', 'forward', 0.2, 'left', 'front', -0.01),
        row('right', 'right', 0.2, stop='right'),
        row('turn', 'turn', -90, capture='left'),
    ]
    events, log = run(
        'follow_front_side_turn_back', rows, 'next\nnext\nnext\nnext\nback\nBACK\nquit\n'
    )
    assert sum(e['event'] == 'completed' for e in events) == 4, log[-3500:]
    assert sum(e['event'] == 'returned' for e in events) == 1
    reports = [e['data']['wall_result'] for e in events if e['event'] == 'wall_result']
    assert abs(reports[1]['wall_clearances_m']['front'] - 0.13) < 0.01
    assert abs(reports[2]['wall_clearances_m']['right'] - 0.14) < 0.01
    events, log = run('stale_scan', [row('f', 'forward', 0.2, 'left')], 'next\nquit\n', stale=True)
    assert any(e['event'] == 'incomplete' for e in events) and 'stale' in log
    events, log = run(
        'maximum_travel',
        [row('f', 'forward', 0.08, 'left', 'front', bound=0.10)],
        'next\nquit\n',
        front=0.55,
    )
    assert any(e['event'] == 'incomplete' for e in events) and 'MAX_TRAVEL' in log
    events, log = run(
        'right_heading_left_reference',
        [row('f', 'forward', 0.03, 'left')],
        'next\nquit\n',
        prepare=True,
    )
    assert 'right-wall alignment' in log
    assert sum(e['event'] == 'completed' for e in events) == 1, log[-3500:]
    print('PASS', OUT, flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
