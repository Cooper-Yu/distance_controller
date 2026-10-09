#!/usr/bin/env python3
"""Submit one bounded step to an idle manual controller; never publishes cmd_vel."""

import argparse
import sys
import rclpy
from distance_controller.srv import ExecuteStep


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'action',
        choices=[
            'forward',
            'backward',
            'left',
            'right',
            'relative',
            'target',
            'front_wall',
            'resume',
            'status',
            'finish',
            'cancel',
        ],
    )
    parser.add_argument('distance', nargs='?', type=float, default=0.0)
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--speed', type=float, default=0.0)
    parser.add_argument('--dwell', type=float, default=1.0)
    parser.add_argument('--frame', choices=['route', 'heading'], default='route')
    parser.add_argument('--side-centering', action='store_true')
    parser.add_argument('--max-travel', type=float, default=0.0)
    parser.add_argument('--timeout', type=float, default=0.0)
    parser.add_argument('--label', default='')
    parser.add_argument('--service', default='/distance_controller/step')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('manual_step_client')
    try:
        client = node.create_client(ExecuteStep, args.service)
        if not client.wait_for_service(timeout_sec=5.0):
            print('Service unavailable; start scene 2 with manual_mode:=true.', file=sys.stderr)
            return 2
        request = ExecuteStep.Request()
        for field in [
            'action',
            'distance',
            'x',
            'y',
            'speed',
            'dwell',
            'frame',
            'side_centering',
            'max_travel',
            'timeout',
            'label',
        ]:
            setattr(request, field, getattr(args, field))
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if not future.done():
            print(
                'Acceptance unknown; query status before retrying. Do not resubmit blindly.',
                file=sys.stderr,
            )
            return 2
        response = future.result()
        print(('ACCEPTED: ' if response.accepted else 'REJECTED: ') + response.message)
        return 0 if response.accepted else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
