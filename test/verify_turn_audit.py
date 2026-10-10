"""Non-installed, domain-isolated Gazebo diagnostic. Clearance only logs; contacts stop.

This deliberately calls the existing turn controller directly, never changes the
production WallRunner and is not a selectable real-robot policy.
"""

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

if os.environ.get('ROS_DOMAIN_ID') != '184' or os.environ.get('IGN_PARTITION') != 'cp18_turn_audit':
    raise SystemExit('Dedicated simulation domain/partition required')
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from wall_geometry import scan_points, swept_clearance, point_clearance

out = Path(os.environ['TURN_AUDIT_OUT'])
rclpy.init()
node = rclpy.create_node(
    'simulation_turn_audit',
    parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)],
)
buffer = Buffer()
listener = TransformListener(buffer, node)
state = {
    'scan': None,
    'odom': None,
    'contact_packets': 0,
    'wall_contacts': 0,
    'warnings': 0,
    'min_gap': 1.0,
    'contact_time': 0.0,
    'scan_time': 0.0,
    'odom_time': 0.0,
}
stop = threading.Event()
trace = (out / 'audit.jsonl').open('w')


def contacts():
    block = []
    with (out / 'contacts.txt').open('w') as log:
        for line in contact_proc.stdout:
            log.write(line)
            if line.strip():
                block.append(line)
                continue
            text = ''.join(block)
            block = []
            if not text:
                continue
            state['contact_packets'] += 1
            state['contact_time'] = time.monotonic()
            if 'rosbot_xl' in text and 'audit_wall' in text:
                state['wall_contacts'] += 1
            if stop.is_set():
                return


contact_proc = subprocess.Popen(
    ['ign', 'topic', '-e', '-t', '/turn_audit/contacts'], stdout=subprocess.PIPE, text=True
)
threading.Thread(target=contacts, daemon=True).start()


def scan(msg):
    state.update(scan=msg, scan_time=time.monotonic())


def odom(msg):
    state.update(odom=msg, odom_time=time.monotonic())


node.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data)
node.create_subscription(Odometry, '/odometry/filtered', odom, qos_profile_sensor_data)
zero = node.create_publisher(Twist, '/cmd_vel', 10)
child = None
result = 'startup_failed'
last_stamp = None
try:
    deadline = time.monotonic() + 25
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if (
            state['scan']
            and state['odom']
            and state['contact_packets'] > 2
            and buffer.can_transform('base_link', state['scan'].header.frame_id, Time())
        ):
            break
    if not state['scan'] or not state['odom'] or state['contact_packets'] < 3:
        raise RuntimeError('Required simulation scan/odom/contact evidence unavailable')
    if state['wall_contacts']:
        raise RuntimeError('Initial wall contact; invalid experiment placement')
    (out / 'initial_points.json').write_text(
        json.dumps(
            scan_points(
                state['scan'],
                buffer.lookup_transform(
                    'base_link', state['scan'].header.frame_id, Time()
                ).transform,
            )[0]
        )
    )
    start = state['odom'].pose.pose.orientation
    start_yaw = math.atan2(2 * start.w * start.z, 1 - 2 * start.z**2)
    with (out / 'controller.log').open('w') as log:
        child = subprocess.Popen(
            [
                'ros2',
                'run',
                'turn_controller',
                'turn_controller',
                '2',
                '--skip-preparation',
                '--ros-args',
                '-p',
                'use_sim_time:=true',
                '-p',
                'turn_angles:=[-1.5707963267948966]',
                '-p',
                'max_angular_speed:=0.20',
                '-p',
                'segment_timeout:=60.0',
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        deadline = time.monotonic() + 100
        while child.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if any(now - state[k] > 2 for k in ['contact_time', 'scan_time', 'odom_time']):
                raise RuntimeError('Simulation feedback/contact stream stale')
            if state['wall_contacts']:
                result = 'contact_stop'
                break
            msg = state['scan']
            stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            if stamp == last_stamp:
                continue
            tf = buffer.lookup_transform('base_link', msg.header.frame_id, Time())
            points, _ = scan_points(msg, tf.transform)
            if not points:
                raise RuntimeError('Empty scan')
            gap = swept_clearance(points, wz=-0.2)
            state['min_gap'] = min(state['min_gap'], gap)
            state['warnings'] += gap < 0.02
            q = state['odom'].pose.pose.orientation
            yaw = math.atan2(2 * q.w * q.z, 1 - 2 * q.z**2)
            trace.write(
                json.dumps(
                    {
                        'stamp': stamp,
                        'yaw': yaw,
                        'gap': gap,
                        'would_stop': gap < 0.02,
                        'nearest': min(points, key=lambda p: point_clearance(*p[:2])),
                    }
                )
                + '\n'
            )
            last_stamp = stamp
        if result != 'contact_stop':
            result = 'completed' if child.poll() == 0 else 'controller_failure_or_timeout'
finally:
    if child and child.poll() is None:
        os.killpg(child.pid, signal.SIGINT)
    for _ in range(20):
        zero.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.025)
    if child:
        try:
            child.wait(timeout=4)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGKILL)
            child.wait()
    stop.set()
    contact_proc.terminate()
    contact_proc.wait(timeout=3)
    q = state['odom'].pose.pose.orientation if state['odom'] else None
    summary = {
        'result': result,
        **{k: state[k] for k in ('contact_packets', 'wall_contacts', 'warnings', 'min_gap')},
    }
    if q:
        summary['final_yaw'] = math.atan2(2 * q.w * q.z, 1 - 2 * q.z**2)
    (out / 'summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary), flush=True)
    trace.close()
    node.destroy_node()
    rclpy.shutdown()

if result not in ('completed', 'contact_stop'):
    raise SystemExit(2)
