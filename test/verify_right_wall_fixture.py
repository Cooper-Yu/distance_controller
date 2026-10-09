"""Isolated synthetic corridor: never publish to the robot domain. Source ROS and overlay first."""

import os

os.environ['ROS_DOMAIN_ID'] = '166'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import json
import math
import re
import signal
import subprocess
import time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

root = Path(os.environ.get('FIXTURE_OUTPUT_ROOT', '/tmp/distance_controller_fixture'))
out = root / 'runtime_logs' / ('right_wall_fixture_' + time.strftime('%Y%m%d_%H%M%S'))
out.mkdir(parents=True)
prefix = subprocess.check_output(
    ['ros2', 'pkg', 'prefix', 'distance_controller'], text=True
).strip()
exe = str(Path(prefix) / 'lib/distance_controller/distance_controller')
rclpy.init()
node = rclpy.create_node('centering_fixture')
broadcaster = StaticTransformBroadcaster(node)
results = []
try:
    for case in [
        'left',
        'right',
        'scan_loss',
        'invalid_ranges',
        'old_stamp',
        'wrong_frame',
        'no_tf',
        'no_scan',
        'travel_limit',
        'duration_limit',
        'rear_missing',
        'rear_close',
        'short_wall',
        'unstable_wall',
        'wrap',
        'noisy_wall',
        'recovery',
    ]:
        if os.environ.get('CENTER_CASE') and case != os.environ['CENTER_CASE']:
            continue
        corridor_angle = 3.13 if case == 'wrap' else -0.35 if case == 'right' else 0.20
        success = case in ['left', 'right', 'wrap', 'recovery']
        topic = '/centering_fixture/' + case
        base = 'body_' + case
        laser = 'laser_' + case
        cmd = [0.0, 0.0, 0.0]
        history = []

        def receive(msg):
            cmd[:] = [msg.linear.x, msg.linear.y, msg.angular.z]
            history.append((time.monotonic(), *cmd))

        sub = node.create_subscription(Twist, topic + '/cmd', receive, 10)
        pub = node.create_publisher(Odometry, topic + '/odom', 10)
        scanpub = node.create_publisher(LaserScan, topic + '/scan', qos_profile_sensor_data)
        tf = TransformStamped()
        tf.header.stamp = node.get_clock().now().to_msg()
        tf.header.frame_id = base
        tf.child_frame_id = laser
        tf.transform.translation.x = 0.02
        tf.transform.translation.z = 0.173
        tf.transform.rotation.z = 1.0
        tf.transform.rotation.w = 0.0
        if case != 'no_tf':
            broadcaster.sendTransform(tf)
        args = [
            exe,
            '2',
            '--ros-args',
            '-r',
            '__ns:=' + topic,
            '-p',
            'odom_topic:=' + topic + '/odom',
            '-p',
            'cmd_vel_topic:=' + topic + '/cmd',
            '-p',
            'scan_topic:=' + topic + '/scan',
            '-p',
            'base_frame:=' + base,
            '-p',
            'forward_distance:=0.12',
            '-p',
            'lateral_distance:=0.08',
            '-p',
            'heading_gain:=3.0',
            '-p',
            'max_yaw_rate:=0.5',
            '-p',
            'centering_gain:=2.0',
            '-p',
            'alignment_settle_duration:=0.15',
            '-p',
            'dwell_duration:=0.1',
        ]
        if case in [
            'no_tf',
            'no_scan',
            'duration_limit',
            'short_wall',
            'unstable_wall',
            'noisy_wall',
        ]:
            args += ['-p', 'preparation_timeout:=2.0']
        if case == 'short_wall':
            args += ['-p', 'wall_heading_min_span:=2.0']
        if case == 'travel_limit':
            args += ['-p', 'preparation_max_travel:=0.01']
        logpath = out / (case + '.log')
        with logpath.open('w') as log:
            proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
            x, y, yaw = (
                (0.04 if case == 'left' else -0.04 if case == 'right' else 0.0),
                (0.05 if case == 'right' else -0.05),
                (-0.2 if case == 'right' else 0.2 if case in ['left', 'wrap', 'recovery'] else 0.0),
            )
            began = prev = time.monotonic()
            next_scan = began
            first_move = None
            trigger = None
            rotated = False
            shifted = False
            centered = None
            fault_seen = None
            disturbed = False
            recovered = False
            try:
                while time.monotonic() - began < 45:
                    rclpy.spin_once(node, timeout_sec=0.008)
                    now = time.monotonic()
                    dt = min(now - prev, 0.03)
                    prev = now
                    text = logpath.read_text()
                    if 'Centered A recorded=' in text and centered is None:
                        m = re.search(r'Centered A recorded=\(([-\d.]+), ([-\d.]+)\)', text)
                        ox, oy = float(m[1]), float(m[2])
                        centered = (
                            math.cos(corridor_angle) * ox + math.sin(corridor_angle) * oy,
                            -math.sin(corridor_angle) * ox + math.cos(corridor_angle) * oy,
                        )
                        assert abs(centered[1]) <= 0.011 and abs(centered[0]) <= 0.011
                        reference = float(re.search(r'heading_reference=([-\d.]+)', text)[1])
                        assert (
                            abs(
                                math.atan2(
                                    math.sin(reference - corridor_angle),
                                    math.cos(reference - corridor_angle),
                                )
                            )
                            < 0.021
                        )
                        targets = re.findall(r'Segment \d/4 target=\(([-\d.]+), ([-\d.]+)\)', text)
                    if case == 'recovery' and centered and not disturbed:
                        yaw += 0.25
                        disturbed = now
                    vx, vy, wz = cmd
                    if (
                        disturbed
                        and now - disturbed > 0.15
                        and abs(yaw + corridor_angle - reference) > 0.15
                    ):
                        assert (
                            math.hypot(vx, vy) < 1e-10
                            and wz * (yaw + corridor_angle - reference) < 0
                        )
                        recovered = True
                    assert all(math.isfinite(v) for v in cmd) and abs(wz) <= 0.500001
                    if centered is None:
                        assert math.hypot(vx, vy) <= 0.030001
                        if 'Initial alignment complete' not in text:
                            assert abs(vx) < 1e-10 and abs(vy) < 1e-10
                        assert abs(vy) <= 0.030001
                        if abs(wz) > 0.001 and first_move is None:
                            rotated = True
                        if abs(vy) > 0.001:
                            shifted = True
                            if first_move is None:
                                first_move = now
                                if success:
                                    assert (
                                        abs(yaw) <= 0.025
                                        and vy * y < 0
                                        and (abs(x) < 0.011 or vx * x < 0)
                                    ), (case, yaw, x, y, cmd)
                    if case in ['short_wall', 'unstable_wall', 'noisy_wall']:
                        assert all(abs(v) < 1e-10 for v in cmd), (case, cmd)
                    x += (math.cos(yaw) * vx - math.sin(yaw) * vy) * dt
                    y += (math.sin(yaw) * vx + math.cos(yaw) * vy) * dt
                    yaw += wz * dt
                    if case == 'duration_limit':
                        y = -0.05
                    if first_move and now - first_move > 0.2 and trigger is None:
                        trigger = now
                    msg = Odometry()
                    msg.header.frame_id = 'odom'
                    msg.child_frame_id = base
                    msg.header.stamp = node.get_clock().now().to_msg()
                    msg.pose.pose.position.x = (
                        math.cos(corridor_angle) * x - math.sin(corridor_angle) * y
                    )
                    msg.pose.pose.position.y = (
                        math.sin(corridor_angle) * x + math.cos(corridor_angle) * y
                    )
                    msg.pose.pose.orientation.z = math.sin((yaw + corridor_angle) / 2)
                    msg.pose.pose.orientation.w = math.cos((yaw + corridor_angle) / 2)
                    msg.twist.twist.linear.x = vx
                    msg.twist.twist.linear.y = vy
                    msg.twist.twist.angular.z = wz
                    pub.publish(msg)
                    if (
                        now >= next_scan
                        and case != 'no_scan'
                        and centered is None
                        and not (case == 'scan_loss' and trigger)
                    ):
                        next_scan = now + 0.05
                        scan = LaserScan()
                        scan.header.frame_id = (
                            'missing_frame' if case == 'wrong_frame' and trigger else laser
                        )
                        scan.header.stamp = node.get_clock().now().to_msg()
                        if case == 'old_stamp' and trigger:
                            scan.header.stamp.sec -= 2
                        scan.angle_min = -math.pi
                        scan.angle_max = math.pi
                        scan.angle_increment = 2 * math.pi / 720
                        scan.range_min = 0.15
                        scan.range_max = 40.0
                        laser_y = y + 0.02 * math.sin(yaw)
                        laser_x = x + 0.02 * math.cos(yaw)
                        ranges = []
                        for i in range(721):
                            scan_yaw = (
                                yaw + (0.12 if int((now - began) / 0.15) % 2 else -0.12)
                                if case == 'unstable_wall'
                                else yaw
                            )
                            direction = math.sin(
                                scan_yaw + math.pi + scan.angle_min + i * scan.angle_increment
                            )
                            distance = (
                                ((0.30 if direction > 0 else -0.30) - laser_y) / direction
                                if abs(direction) > 1e-8
                                else float('inf')
                            )
                            rear_direction = math.cos(
                                scan_yaw + math.pi + scan.angle_min + i * scan.angle_increment
                            )
                            rear_hit = (
                                (-0.28 - laser_x) / rear_direction
                                if rear_direction < -1e-8
                                else float('inf')
                            )
                            distance = min(distance, rear_hit)
                            body_angle = math.atan2(
                                math.sin(math.pi + scan.angle_min + i * scan.angle_increment),
                                math.cos(math.pi + scan.angle_min + i * scan.angle_increment),
                            )
                            if abs(abs(body_angle) - math.pi) < 0.09 and trigger:
                                if case == 'rear_missing':
                                    distance = float('inf')
                                if case == 'rear_close':
                                    distance = 0.23
                            if case == 'noisy_wall' and 0.20 < abs(body_angle + math.pi / 2) < 0.52:
                                distance += 0.035 if i % 2 else -0.035
                            ranges.append(distance if 0.15 <= distance <= 40 else float('inf'))
                        if case == 'invalid_ranges' and trigger:
                            ranges = [float('nan')] * 721
                        scan.ranges = ranges
                        scanpub.publish(scan)
                    if not success and 'Initial preparation fault:' in text:
                        if fault_seen is None:
                            fault_seen = now
                        if now - fault_seen > 0.35:
                            assert all(abs(v) < 1e-10 for v in cmd) and centered is None
                            recent = history[-5:]
                            assert len(recent) == 5 and all(
                                all(abs(v) < 1e-10 for v in row[1:]) for row in recent
                            )
                            break
                    if proc.poll() is not None:
                        break
                if success:
                    if case == 'recovery':
                        assert recovered
                    assert proc.poll() == 0 and centered and rotated and shifted, (
                        case,
                        proc.poll(),
                        text[-1200:],
                    )
                    assert math.hypot(x - centered[0], y - centered[1]) < 0.011
                    assert 'Initial preparation fault' not in text and 'Route completed' in text
                    assert re.findall(r'Entering DONE state \| segment=(\d)', text) == [
                        '0',
                        '1',
                        '2',
                        '3',
                    ]
                    assert all(abs(v) < 1e-10 for v in cmd)
                    targets = [
                        tuple(map(float, t))
                        for t in re.findall(r'Segment \d/4 target=\(([-\d.]+), ([-\d.]+)\)', text)
                    ]
                    assert len(targets) == 4
                    for (tx, ty), (dx, dy) in zip(
                        targets, [(0.12, 0), (0.12, -0.08), (0.12, 0), (0, 0)]
                    ):
                        assert (
                            math.hypot(
                                tx - ox - math.cos(reference) * dx + math.sin(reference) * dy,
                                ty - oy - math.sin(reference) * dx - math.cos(reference) * dy,
                            )
                            < 0.000003
                        )
                    results.append(
                        {
                            'case': case,
                            'passed': True,
                            'recorded_A': centered,
                            'final_pose': [x, y, yaw],
                            'scan_disabled_after_centering': True,
                        }
                    )
                else:
                    assert 'Initial preparation fault:' in text and centered is None, (
                        case,
                        text[-1000:],
                    )
                    results.append({'case': case, 'passed': True, 'stopped_and_latched': True})
            finally:
                if proc.poll() is None:
                    proc.send_signal(signal.SIGINT)
                    try:
                        proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait()
                node.destroy_subscription(sub)
                node.destroy_publisher(pub)
                node.destroy_publisher(scanpub)
finally:
    (out / 'results.json').write_text(json.dumps(results, indent=2))
    node.destroy_node()
    rclpy.shutdown()
print(json.dumps({'out': str(out), 'results': results}))
