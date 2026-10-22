#!/usr/bin/env python3
"""Real ROS late competing publisher/service discovery; no motion commands."""

import io
import json
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from motion_console import Console
from motion_model import Motion
from wall_runtime import WallRunner


def check_case(kind):  # noqa: PLR0915 - preserve one owned ROS lifetime and cleanup
    import rclpy
    from distance_controller.srv import ExecuteStep
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    runner = WallRunner(0.03, alignment_wall='right')
    foreign = rclpy.create_node('late_foreign_' + kind)
    odom = runner.node.create_publisher(Odometry, '/odometry/filtered', 10)
    commands = []
    runner.node.create_subscription(Twist, '/cmd_vel', commands.append, 10)

    def feedback():
        msg = Odometry()
        msg.header.stamp = runner.node.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
        msg.pose.pose.orientation.w = 1.0
        odom.publish(msg)

    runner.node.create_timer(0.02, feedback)
    audit = io.StringIO()
    console = Console(runner, None, SimpleNamespace(poll=lambda: (False, None)), audit, 0.1)
    emit = console.emit
    injected = None
    started = time.monotonic()

    def inject(event, **data):
        nonlocal injected
        emit(event, **data)
        if event != 'started':
            return
        injected = time.monotonic()
        if kind == 'publisher':
            foreign.create_publisher(Twist, '/cmd_vel', 10)
        else:
            foreign.create_service(ExecuteStep, '/distance_controller/step', lambda _req, res: res)
        until = time.monotonic() + 2
        while time.monotonic() < until:
            rclpy.spin_once(runner.node, timeout_sec=0.01)
            ready = (
                runner.node.get_publishers_info_by_topic('/cmd_vel')
                if kind == 'publisher'
                else runner.client.service_is_ready()
            )
            if ready:
                return
        raise AssertionError('Fixture competitor not discovered')

    console.emit = inject
    try:
        try:
            console.move(Motion('move', x=0.01))
        except RuntimeError as error:
            assert 'Existing /cmd_vel publisher or step service' in str(error), str(error)
            rejected = str(error)
        else:
            raise AssertionError('Late competitor admitted')
        assert injected is not None and injected - started >= 2
        assert runner.velocity_pub is None and not commands
        rows = [json.loads(row) for row in audit.getvalue().splitlines()]
        assert not any(row['event'] == 'publisher_created' for row in rows)
        return dict(
            kind=kind,
            error=rejected,
            entry_wait_s=injected - started,
            total_s=time.monotonic() - started,
            commands=len(commands),
            events=rows,
        )
    finally:
        foreign.destroy_node()
        runner.close(False)


def main():
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '194':
        raise RuntimeError('Require isolated localhost domain 194')
    root = Path('/tmp/console_exclusive') / str(time.time_ns())
    root.mkdir(parents=True)
    results = [check_case(kind) for kind in ('publisher', 'service')]
    (root / 'report.json').write_text(json.dumps(results, indent=2) + '\n')
    print('TWO_LATE_COMPETITORS_REJECTED', root, json.dumps(results), flush=True)


if __name__ == '__main__':
    main()
