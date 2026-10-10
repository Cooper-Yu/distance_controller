"""Isolated localhost/domain179 integration of the interactive route session."""

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

os.environ['ROS_DOMAIN_ID'] = '179'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

ROOT = Path(__file__).resolve().parents[1]
OUT = Path('/tmp/action_session_test') / str(time.time_ns())
OUT.mkdir(parents=True)
rclpy.init()
node = rclpy.create_node('action_session_plant')
pub = node.create_publisher(Odometry, '/odometry/filtered', 10)
scan_pub = node.create_publisher(LaserScan, '/scan_filtered', 10)
tf_pub = StaticTransformBroadcaster(node)
tf = TransformStamped()
tf.header.frame_id, tf.child_frame_id = 'base_link', 'laser'
tf.transform.rotation.z, tf.transform.rotation.w = 1.0, 0.0
tf_pub.sendTransform(tf)
pose = [1.0, 2.0, 0.4]
cmd = [0.0, 0.0, 0.0]


def receive(msg):
    cmd[:] = [msg.linear.x, msg.linear.y, msg.angular.z]
    assert math.hypot(*cmd[:2]) <= 0.031 and abs(cmd[2]) <= 0.501


sub = node.create_subscription(Twist, '/cmd_vel', receive, 10)


def tick(previous):
    rclpy.spin_once(node, timeout_sec=0.01)
    now = time.monotonic()
    dt = min(now - previous, 0.03)
    vx, vy, wz = cmd
    pose[0] += (math.cos(pose[2]) * vx - math.sin(pose[2]) * vy) * dt
    pose[1] += (math.sin(pose[2]) * vx + math.cos(pose[2]) * vy) * dt
    pose[2] += wz * dt
    msg = Odometry()
    msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x, msg.pose.pose.position.y = pose[:2]
    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = (
        math.sin(pose[2] / 2),
        math.cos(pose[2] / 2),
    )
    msg.twist.twist.linear.x, msg.twist.twist.linear.y = vx, vy
    msg.twist.twist.angular.z = wz
    pub.publish(msg)
    scan = LaserScan()
    scan.header.frame_id, scan.header.stamp = 'laser', msg.header.stamp
    scan.angle_min, scan.angle_increment, scan.angle_max = -math.pi, math.pi / 180, math.pi
    scan.range_min, scan.range_max, scan.ranges = 0.15, 40.0, [1.0] * 361
    scan_pub.publish(scan)
    return now


def run(name, commands, interrupt=False):
    directory = OUT / name
    directory.mkdir()
    route = directory / 'route.json'
    rows = [
        dict(name='move', kind='forward', value=0.4 if interrupt else 0.07, unit='m'),
        dict(name='turn', kind='turn', value=30, unit='deg'),
        dict(name='side', kind='left', value=0.06, unit='m'),
    ]
    route.write_text(json.dumps({'actions': rows}))
    start = pose.copy()
    with (directory / 'runtime.log').open('w') as log:
        proc = subprocess.Popen(
            [
                'python3',
                str(ROOT / 'tools/action_session.py'),
                '--route',
                str(route),
                '--start',
                'current',
            ],
            stdin=subprocess.PIPE,
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
            cwd=directory,
            start_new_session=True,
        )
        proc.stdin.write(commands)
        proc.stdin.flush()
        if not interrupt:
            proc.stdin.close()
        deadline, previous = time.monotonic() + 160, time.monotonic()
        sent = False
        try:
            while proc.poll() is None and time.monotonic() < deadline:
                previous = tick(previous)
                if (
                    interrupt
                    and not sent
                    and math.hypot(pose[0] - start[0], pose[1] - start[1]) > 0.04
                ):
                    os.kill(proc.pid, signal.SIGINT)
                    proc.stdin.write('status\nback\nBACK\nset 1 0.06\nnext\nquit\n')
                    proc.stdin.close()
                    sent = True
            assert proc.poll() is not None, 'integration timeout'
        finally:
            if proc.poll() is None:
                os.kill(proc.pid, signal.SIGINT)
                proc.stdin.close()
                proc.wait(timeout=15)
        assert proc.returncode == 0, (directory / 'runtime.log').read_text()[-2500:]
    events = [
        json.loads(line)
        for line in next(directory.glob('action_session_*.jsonl')).read_text().splitlines()
    ]
    print(name, 'events', [e['event'] for e in events], 'end', pose, flush=True)
    return events


try:
    events = run(
        'edit_return',
        'next\nnext\nback\nBACK\nset 2 -20\nnext\nnext\nback\nBACK\nset 3 0.04\nnext\nmeasure\nquit\n',
    )
    assert sum(e['event'] == 'completed' for e in events) == 5
    assert sum(e['event'] == 'returned' for e in events) == 2
    assert not any(e['event'] == 'incomplete' for e in events)
    finished = [e['data'] for e in events if e['event'] == 'completed']
    origin = events[0]['data']['pose']
    last = finished[-1]['target']
    yaw = origin['yaw'] - math.radians(20)
    assert (
        abs(last['x'] - (origin['x'] + 0.07 * math.cos(origin['yaw']) - 0.04 * math.sin(yaw)))
        < 1e-6
    )
    assert any(e['laser'] for e in events)
    events = run('interrupt_retry', 'next\n', True)
    assert any(e['event'] == 'incomplete' for e in events)
    assert sum(e['event'] == 'returned' for e in events) == 1
    assert sum(e['event'] == 'completed' for e in events) == 1
    assert abs(cmd[0]) + abs(cmd[1]) + abs(cmd[2]) < 1e-6
    events = run('continuous', 'run\nRUN\nquit\n')
    assert sum(e['event'] == 'completed' for e in events) == 3
    assert not any(e['event'] == 'incomplete' for e in events)
    print('PASS:', OUT, flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
