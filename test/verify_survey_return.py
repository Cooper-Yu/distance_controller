"""Local synthetic odom test. Domain 177, localhost only; no hardware connection."""

import math
import json
import os
from pathlib import Path
import signal
import subprocess
import time

os.environ['ROS_DOMAIN_ID'] = '177'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from nav_msgs.msg import Odometry

ROOT = Path(__file__).resolve().parents[1]
OUT = Path('/tmp/survey_return_verification') / str(time.time_ns())
OUT.mkdir(parents=True, exist_ok=True)
rclpy.init()
node = rclpy.create_node('return_fixture')
pub = node.create_publisher(Odometry, '/odometry/filtered', 10)
scan_pub = node.create_publisher(LaserScan, '/scan_filtered', 10)
tf_pub = StaticTransformBroadcaster(node)
transform = TransformStamped()
transform.header.frame_id = 'base_link'
transform.child_frame_id = 'laser'
transform.transform.rotation.z = 1.0
transform.transform.rotation.w = 0.0
transform.transform.translation.x = 0.02
tf_pub.sendTransform(transform)
pose = [1.2, -0.4, 0.7]
cmd = [0.0, 0.0, 0.0]


def receive(msg):
    cmd[:] = [msg.linear.x, msg.linear.y, msg.angular.z]


sub = node.create_subscription(Twist, '/cmd_vel', receive, 10)


def run_case(name, args, publish=True, scan_age=0.0):
    start = pose.copy()
    with (OUT / (name + '.log')).open('w') as log:
        proc = subprocess.Popen(
            ['python3', str(ROOT / 'tools/survey_return.py'), *args],
            stdin=subprocess.PIPE,
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
            cwd=OUT,
        )
        step = args[args.index('--step') + 1] if '--step' in args else '0'
        proc.stdin.write('RUN ' + step + '\n')
        proc.stdin.close()
        previous = time.monotonic()
        deadline = previous + 65
        try:
            while proc.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.01)
                now = time.monotonic()
                dt = min(now - previous, 0.03)
                previous = now
                scan = LaserScan()
                ns = node.get_clock().now().nanoseconds - int(scan_age * 1e9)
                scan.header.stamp.sec, scan.header.stamp.nanosec = divmod(ns, 1_000_000_000)
                scan.header.frame_id = 'laser'
                scan.angle_min, scan.angle_increment = 0.0, math.pi / 180
                scan.angle_max = 359 * scan.angle_increment
                scan.range_min, scan.range_max = 0.15, 40.0
                values = [1.0] * 360
                for center, distance in [(0, 4.0), (90, 3.0), (180, 1.0), (270, 2.0)]:
                    for offset in range(-6, 7):
                        values[(center + offset) % 360] = distance
                scan.ranges = values
                scan_pub.publish(scan)
                vx, vy, wz = cmd
                pose[0] += (math.cos(pose[2]) * vx - math.sin(pose[2]) * vy) * dt
                pose[1] += (math.sin(pose[2]) * vx + math.cos(pose[2]) * vy) * dt
                pose[2] += wz * dt
                if publish:
                    msg = Odometry()
                    msg.header.stamp = node.get_clock().now().to_msg()
                    msg.header.frame_id = 'odom'
                    msg.child_frame_id = 'base_link'
                    msg.pose.pose.position.x, msg.pose.pose.position.y = pose[:2]
                    msg.pose.pose.orientation.z = math.sin(pose[2] / 2)
                    msg.pose.pose.orientation.w = math.cos(pose[2] / 2)
                    msg.twist.twist.linear.x, msg.twist.twist.linear.y = vx, vy
                    msg.twist.twist.angular.z = wz
                    pub.publish(msg)
            assert proc.poll() is not None, 'Fixture timed out'
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                proc.wait(timeout=10)
    print(name, 'exit', proc.returncode, 'start', start, 'end', pose, flush=True)
    return proc.returncode, start


try:
    code, start = run_case('backward', ['--step', '2', '--distance', '0.06', '--execute'])
    assert code == 0
    dx, dy = pose[0] - start[0], pose[1] - start[1]
    along = dx * math.cos(start[2]) + dy * math.sin(start[2])
    assert -0.08 < along < -0.04, along
    assert abs(-dx * math.sin(start[2]) + dy * math.cos(start[2])) < 0.01
    code, start = run_case('turn', ['--step', '3', '--execute'])
    assert code == 0
    assert abs(pose[2] - start[2] + math.pi / 2) < 0.02
    code, start = run_case(
        'missing_odom', ['--step', '2', '--distance', '0.06', '--execute'], False
    )
    assert code == 2
    assert math.hypot(pose[0] - start[0], pose[1] - start[1]) < 0.001
    blocker = node.create_publisher(Twist, '/cmd_vel', 10)
    code, start = run_case('conflict', ['--step', '2', '--execute'])
    assert code == 2
    assert 'Existing /cmd_vel' in (OUT / 'conflict.log').read_text()
    node.destroy_publisher(blocker)
    code, start = run_case('read_only', ['--measure'])
    assert code == 0 and pose == start
    latest = max(OUT.glob('laser_observation_*.json'), key=lambda p: p.stat().st_mtime_ns)
    observation = json.loads(latest.read_text())[0]
    assert observation['status'] == 'ok'
    assert observation['distances']['right']['median_m'] == 3.0
    code, start = run_case('stale_scan', ['--measure'], scan_age=2.0)
    assert code == 0 and pose == start
    latest = max(OUT.glob('laser_observation_*.json'), key=lambda p: p.stat().st_mtime_ns)
    observation = json.loads(latest.read_text())[0]
    assert observation['status'] == 'stale_or_not_new_scan'
    assert observation['distances'] is None
    successful = [json.loads(p.read_text()) for p in OUT.glob('return_step_*.json')]
    for audit in successful:
        if audit['success'] and 'laser_observations' in audit:
            assert [x['phase'] for x in audit['laser_observations']] == ['before', 'after']
            assert all(x['status'] == 'ok' for x in audit['laser_observations'])
    print('PASS: motion/failures, before/after scan audit, read-only, stale scan')
finally:
    node.destroy_node()
    rclpy.shutdown()
