#!/usr/bin/env python3
"""Isolated Humble ROS synthetic odom/cmd plant exercising real Console.follow."""

import io
import json
import math
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from motion_console import Console
from motion_model import Motion
from wall_runtime import WallRunner


def run_case(settling):  # noqa: PLR0915 - keep one bounded ROS lifetime with cleanup
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    runner = WallRunner(0.03)
    plant = dict(yaw=0.009, wz=0.0, previous=time.monotonic(), commands=[])
    odom = runner.node.create_publisher(Odometry, '/odometry/filtered', 10)

    def command(msg):
        plant['wz'] = msg.angular.z
        plant['commands'].append(msg.angular.z)

    runner.node.create_subscription(Twist, '/cmd_vel', command, 10)

    def feedback():
        now = time.monotonic()
        plant['yaw'] += plant['wz'] * (now - plant['previous'])
        plant['previous'] = now
        msg = Odometry()
        msg.header.stamp = runner.node.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
        msg.pose.pose.position.x = 0.1
        msg.pose.pose.orientation.z = math.sin(plant['yaw'] / 2)
        msg.pose.pose.orientation.w = math.cos(plant['yaw'] / 2)
        msg.twist.twist.angular.z = plant['wz']
        odom.publish(msg)

    runner.node.create_timer(0.02, feedback)
    audit = io.StringIO()
    console = Console(runner, None, SimpleNamespace(poll=lambda: (False, None)), audit, 0.1)
    try:
        until = time.monotonic() + 0.5
        while time.monotonic() < until:
            runner.pump()
        assert not runner.node.get_publishers_info_by_topic('/cmd_vel'), (
            'Domain has foreign publisher'
        )
        assert not runner.client.service_is_ready(), 'Domain has foreign step service'
        assert len(runner.node.get_publishers_info_by_topic('/odometry/filtered')) == 1
        runner.pose()
        runner.velocity_pub = runner.node.create_publisher(Twist, '/cmd_vel', 10)
        motion = Motion('path', x=0.1, points=((0.0, 0.0), (0.1, 0.0)))
        console.follow(Pose(0, 0, 0), motion, 6, settling=settling)
        before_bias = plant['yaw']
        # Identical deterministic post-follow stop disturbance in both cases.
        plant['yaw'] += 0.002
        console.stop_motion()
        final = runner.pose()
        ready, _ = console.stopped_check(Pose(0, 0, 0), final, motion, 1)
        nonzero = [w for w in plant['commands'] if abs(w) > 1e-9]
        if settling:
            assert nonzero and all(w < 0 for w in nonzero), nonzero
            assert ready and abs(final.yaw) < 0.01, final
            assert max(map(abs, nonzero)) <= 0.1
        else:
            assert not nonzero, nonzero
            assert not ready and abs(final.yaw - 0.011) < 1e-8, final
        return dict(
            settling=settling,
            before_bias_rad=before_bias,
            final_yaw_rad=final.yaw,
            final_ready=ready,
            correction_commands=len(nonzero),
            max_angular_command=max(map(abs, plant['commands']), default=0),
            audit=[json.loads(line) for line in audit.getvalue().splitlines()],
        )
    finally:
        console.stop_motion()
        runner.close(False)


def main():
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '193':
        raise RuntimeError('Require isolated localhost domain 193')
    results = [run_case(False), run_case(True)]
    root = Path('/tmp/settle_heading_margin') / str(time.time_ns())
    root.mkdir(parents=True)
    (root / 'report.json').write_text(json.dumps(results, indent=2) + '\n')
    print('SETTLE_HEADING_MARGIN_PASS', root, json.dumps(results), flush=True)


if __name__ == '__main__':
    main()
