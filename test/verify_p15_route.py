#!/usr/bin/env python3
"""Full P15 route and cancellation during P10 turn; isolated synthetic ROS only."""

import json
import math
import os
import tempfile
import rclpy
from verify_motion_console import Fixture


def run(mode):
    f = Fixture(tempfile.mkdtemp(prefix='p15_route_' + mode + '_'))
    try:
        f.wait(lambda: 'motion> ' in f.text())
        f.send('path_plan P15')
        f.wait(lambda: 'P15 path P01 heading=' in f.text())
        assert not f.commands
        f.send('path P15')
        if mode == 'stop':
            f.wait(lambda: 'Recorded: P10 ' in f.text() and f.velocity[2] < -0.1, 200)
            f.send('stop')
            f.wait(lambda: 'Operator stop' in f.text(), 10)
            points = json.loads(f.path.read_text())['points']
            assert 'P10_after_turn' not in points and 'P11' not in points
        else:
            f.wait(lambda: 'P15 route completed; stopped' in f.text(), 310)
            points = json.loads(f.path.read_text())['points']
            assert len(points) == 21, list(points)
            assert 'P15_after_turn' not in points
            for label, x, y, yaw in [
                ('P09', 1.4065502783181423, -1.5878885984779116, -90),
                ('P11', 0.9418674948445646, -1.9058474122860602, -180),
                ('P12', 0.9221371725514836, -1.0915868730786444, 90),
                ('P15', 0.02899960878708014, -0.5596575672149936, -180),
            ]:
                p = points[label]['relative_to_p01']
                assert math.hypot(p['x_m'] - x, p['y_m'] - y) < 0.005, p
                error = math.radians(p['yaw_deg'] - yaw)
                assert abs(math.atan2(math.sin(error), math.cos(error))) < 0.01, p
            rows = [
                json.loads(line) for line in f.path.with_suffix('.jsonl').read_text().splitlines()
            ]
            turns = [
                round(math.degrees(row['motion']['angle']))
                for row in rows
                if row['event'] == 'started' and row['motion']['kind'] == 'turn'
            ]
            assert turns == [-90, -90, -90, 90, -90, 90], turns
            assert len([row for row in rows if row['event'] == 'post_turn_reconnect']) == 6
            f.send('path P15')
            f.wait(lambda: 'already recorded' in f.text())
        assert f.velocity == (0.0, 0.0, 0.0)
        print('PASS P15', mode, f.path.parent, flush=True)
    finally:
        f.close()


if __name__ == '__main__':
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert int(os.environ.get('ROS_DOMAIN_ID', '0')) >= 190
    rclpy.init()
    try:
        for mode in ('complete', 'stop'):
            run(mode)
    finally:
        rclpy.shutdown()
