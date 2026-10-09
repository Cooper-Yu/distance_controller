#!/usr/bin/env python3
"""Submit one bounded step to an idle manual controller; never publishes cmd_vel."""

import argparse
import sys
import rclpy
from distance_controller.srv import ExecuteStep


def parse_arguments():
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
            'history',
            'backtrack',
            'return_to',
            'finish',
            'cancel',
        ],
    )
    parser.add_argument('value', nargs='?', help='Distance (m), backtrack count, or return label')
    parser.add_argument('--visit-id', type=int, default=None)
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
    args.distance, args.steps = 0.0, 0
    args.use_visit_id = args.visit_id is not None
    args.visit_id = args.visit_id if args.use_visit_id else 0
    try:
        if args.action == 'backtrack':
            args.steps = int(args.value) if args.value is not None else 1
            if not 1 <= args.steps <= 4294967295:
                raise ValueError('backtrack count must be a positive uint32')
        elif args.action == 'return_to':
            if args.use_visit_id == (args.value is not None):
                raise ValueError('return_to requires a label OR --visit-id')
            if not 0 <= args.visit_id <= 18446744073709551615:
                raise ValueError('visit ID must be a nonnegative uint64')
            args.label = args.value or ''
        elif args.value is not None:
            if args.action not in ['forward', 'backward', 'left', 'right', 'front_wall']:
                raise ValueError('this action does not take a positional value')
            args.distance = float(args.value)
        if args.use_visit_id and args.action != 'return_to':
            raise ValueError('--visit-id is only for return_to')
    except ValueError as error:
        parser.error(str(error))
    return args


def main():
    args = parse_arguments()
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
            'steps',
            'use_visit_id',
            'visit_id',
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
