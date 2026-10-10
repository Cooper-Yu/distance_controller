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
    assert math.hypot(*velocity[:2]) < 0.051 and abs(velocity[2]) < 0.501


sub = node.create_subscription(Twist, '/cmd_vel', receive, 10)
last_scan = 0.0


def tick(previous, scan_enabled=True, front=0.55):
    global last_scan
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
    if scan_enabled and now - last_scan > 0.07:
        scan = LaserScan()
        scan.header.frame_id, scan.header.stamp = 'laser', msg.header.stamp
        scan.angle_min, scan.angle_increment, scan.angle_max = -math.pi, math.pi / 360, math.pi
        scan.range_min, scan.range_max = 0.15, 40.0
        x, y = pose[0] + 0.02 * c, pose[1] + 0.02 * s
        values = []
        for i in range(721):
            a = scan.angle_min + i * scan.angle_increment + pose[2] + math.pi
            dx, dy = math.cos(a), math.sin(a)
            candidates = []
            if abs(dx) > 1e-8:
                candidates.append(((front if dx > 0 else -0.3) - x) / dx)
            if abs(dy) > 1e-8:
                candidates.append(((0.3 if dy > 0 else -0.5) - y) / dy)
            distance = min(v for v in candidates if v > 0)
            values.append(distance if distance >= 0.15 else float('inf'))
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


def run(name, rows, commands, prepare=False, stale=False, front=0.55):
    pose[:] = [0.0, 0.0, 0.0]
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
                '--route',
                str(config),
                '--start',
                'prepare' if prepare else 'current',
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
    if os.getenv('WALL_CASE') == 'prepare':
        events, log = run(
            'left_preparation', [row('f', 'forward', 0.03, 'left')], 'next\nquit\n', prepare=True
        )
        assert 'left-wall alignment' in log
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
        'left_preparation', [row('f', 'forward', 0.03, 'left')], 'next\nquit\n', prepare=True
    )
    assert 'left-wall alignment' in log
    assert sum(e['event'] == 'completed' for e in events) == 1, log[-3500:]
    print('PASS', OUT, flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
