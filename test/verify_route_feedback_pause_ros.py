#!/usr/bin/env python3
"""Localhost-only ROS fixture. Run in a separate ROS_DOMAIN_ID, never a robot domain."""

import math
import json
import os
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from route_feedback_pause import RouteFeedbackPause
from wall_runtime import WallRunner


def main():  # noqa: PLR0912, PLR0915 - ordered fixture fault modes and cleanup
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '195':
        raise RuntimeError('Fixture requires ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=195')
    audit_path = None
    if os.environ.get('TASK6_FIXTURE_AUDIT_DIR'):
        audit_dir = Path(os.environ['TASK6_FIXTURE_AUDIT_DIR']).resolve()
        if not audit_dir.is_relative_to(Path('/tmp')) or audit_dir == Path('/tmp'):
            raise RuntimeError('Fixture audit directory must be below /tmp')
        audit_dir.mkdir(parents=True, exist_ok=True)
        audit_path = audit_dir / 'points.jsonl'
        with audit_path.open('x'):
            pass
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

    case = sys.argv[1] if len(sys.argv) > 1 else 'identity'
    lagged_motion = case in ('lagged_motion', 'drift_excess')
    runner = WallRunner(0.12 if lagged_motion else 0.04)
    runner.observation_scan_timeout = 1.0  # Same post-preparation route policy.
    producer = Node('feedback_pause_fixture')
    topics = ('/rosbot_xl_base_controller/odom', '/odometry/filtered')
    pubs = [producer.create_publisher(Odometry, topic, qos_profile_sensor_data) for topic in topics]
    scan_pub = producer.create_publisher(LaserScan, '/scan_filtered', qos_profile_sensor_data)
    commands, events = [], []

    def emit(event, **data):
        events.append((time.monotonic(), event, data))
        if audit_path is not None:
            with audit_path.open('a') as stream:
                stream.write(json.dumps(dict(time=time.time(), event=event, **data)) + '\n')

    producer.create_subscription(
        Twist,
        '/cmd_vel',
        lambda msg: commands.append(
            (time.monotonic(), (msg.linear.x, msg.linear.y, msg.angular.z))
        ),
        10,
    )
    tf = StaticTransformBroadcaster(producer)
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'base_link', 'fixture_laser'
    transform.transform.rotation.w = 1.0
    transform.header.stamp = producer.get_clock().now().to_msg()
    tf.sendTransform(transform)
    mode = dict(until=0.0, scan_until=0.0, forever=False, replace=False, cancel_at=None, offset=0.0)

    def tick():
        now = time.monotonic()
        if mode['replace']:
            producer.destroy_publisher(pubs[0])
            pubs[0] = producer.create_publisher(Odometry, topics[0], qos_profile_sensor_data)
            mode['replace'] = False
        if mode['cancel_at'] is not None and now >= mode['cancel_at']:
            runner.cancel_requested = True
        stamp = producer.get_clock().now().to_msg()
        if not mode['forever'] and now >= mode['until']:
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id = stamp, 'odom'
            odom.child_frame_id, odom.pose.pose.orientation.w = 'base_link', 1.0
            odom.pose.pose.position.x = mode['offset']
            for pub in pubs:
                pub.publish(odom)
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, 'fixture_laser'
        scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi, math.pi / 180
        scan.range_min, scan.range_max, scan.ranges = 0.05, 10.0, [2.0] * 361
        if now >= mode['scan_until']:
            scan_pub.publish(scan)

    producer.create_timer(0.025, tick)
    executor = SingleThreadedExecutor()
    executor.add_node(producer)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    try:
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            runner.pump()
        pause = RouteFeedbackPause(runner, emit, lambda: None)
        runner.feedback_pause = pause
        runner.velocity_pub = runner.node.create_publisher(Twist, '/cmd_vel', 10)
        runner.send((0.12 if lagged_motion else 0.01, 0.0, 0.0))
        runner.pump()
        scan_only = len(sys.argv) > 1 and sys.argv[1] == 'scan'
        if lagged_motion:
            deadline = time.monotonic() + 0.3
            while runner.guarded_command is None or runner.guarded_command[0][0] != 0.12:
                assert time.monotonic() < deadline
                runner.pump()
            mode.update(
                until=time.monotonic() + 0.9, offset=0.11 if case == 'drift_excess' else 0.06
            )
        else:
            mode['scan_until' if scan_only else 'until'] = time.monotonic() + 0.9
        deadline = time.monotonic() + 3
        try:
            while pause.generation == 0 and time.monotonic() < deadline:
                runner.pump()
        except RuntimeError as error:
            if case != 'drift_excess':
                raise
            assert 'drift exceeded 4 cm after timestamp allowance' in str(error), error
            assert not any(name == 'feedback_pause_completed' for _, name, _ in events)
            print('PASS real ROS excess displacement: zero hold and terminal rejection')
            return 0
        assert case != 'drift_excess', 'Excess drift accepted'
        assert pause.generation == 1, events
        if lagged_motion:
            proof = next(
                data['displacement_checks']
                for _, name, data in events
                if name == 'feedback_pause_completed'
            )
            assert all(
                row['displacement_m'] > 0.04 and row['total_bound_m'] > 0.07
                for row in proof.values()
            ), proof
            print('PASS real ROS delayed old-anchor 6cm displacement: bounded recovery')
        started = next(t for t, name, _ in events if name == 'feedback_pause_started')
        completed = next(t for t, name, _ in events if name == 'feedback_pause_completed')
        assert completed - started >= 0.3
        held = [command for t, command in commands if started + 0.05 <= t <= completed]
        assert len(held) >= 3 and all(not any(command) for command in held), held
        assert (
            max(
                b - a
                for a, b in zip(
                    [t for t, _ in commands if started <= t <= completed],
                    [t for t, _ in commands if started <= t <= completed][1:],
                )
            )
            < 0.2
        )
        print(
            f'PASS real ROS {"scan-only" if scan_only else "dual-source"} gap: '
            'zero hold, distinct stopped recovery, same owner'
        )

        # Each fault mode gets its own process/epoch and synthetic owner.
        fault = case if not scan_only and not lagged_motion else 'identity'
        assert fault in ('identity', 'timeout', 'cancel')
        if fault == 'identity':
            mode['replace'] = True
        else:
            mode['forever'] = True
            if fault == 'cancel':
                mode['cancel_at'] = time.monotonic() + 0.6
        before_fault = len(events)
        fault_started = time.monotonic()
        deadline = fault_started + 7
        try:
            while time.monotonic() < deadline:
                runner.pump()
        except (RuntimeError, KeyboardInterrupt) as error:
            if fault == 'identity':
                assert 'identity changed' in str(error), error
            elif fault == 'timeout':
                assert 'timeout' in str(error), error
                assert time.monotonic() - fault_started < 5.8
            else:
                assert isinstance(error, KeyboardInterrupt), error
        else:
            raise AssertionError(f'{fault} was accepted')
        assert all(name != 'feedback_pause_completed' for _, name, _ in events[before_fault:])
        print(f'PASS real ROS {fault}: abort, no recovery completion')
        return 0
    finally:
        # This fixture owns only the synthetic localhost publisher.
        if runner.velocity_pub is not None:
            runner.send((0.0, 0.0, 0.0))
            runner.node.destroy_publisher(runner.velocity_pub)
            runner.velocity_pub = None
        executor.shutdown()
        worker.join(timeout=2)
        producer.destroy_node()
        runner.close(False)


if __name__ == '__main__':
    raise SystemExit(main())
