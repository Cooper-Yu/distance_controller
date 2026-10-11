#!/usr/bin/env python3
"""P01-P03 route completion and second-leg cancellation on synthetic odom only."""

import json
import math
import os
import tempfile
import rclpy
from verify_motion_console import Fixture


def run(mode):
    directory = tempfile.mkdtemp(prefix='p03_route_' + mode + '_')
    f = Fixture(directory)
    try:
        f.wait(lambda: 'motion> ' in f.text())
        f.send('path_plan P03_turn')
        f.wait(lambda: '20 points' in f.text())
        assert not f.commands
        f.send('path P03_turn')
        f.wait(lambda: f.y > 3.05, 65)
        if mode == 'complete':
            f.wait(lambda: 'P03 route completed' in f.text(), 65)
            data = json.loads(f.path.read_text())['points']
            for label, x, y in [
                ('P02', 0.8402388396167952, -0.0026000285269909117),
                ('P03', 1.773003723416002, -0.0026000285269909117),
            ]:
                p = data[label]['relative_to_p01']
                assert math.hypot(p['x_m'] - x, p['y_m'] - y) < 0.005, p
            p = data['P03_after_turn']['relative_to_p01']
            assert abs(p['yaw_deg'] + 90) < math.degrees(0.01), p
            f.send('path P03_turn')
            f.wait(lambda: 'already recorded' in f.text())
        else:
            if mode == 'stop':
                f.send('stop')
                marker = 'Operator stop'
            else:
                f.feedback_enabled = False
                marker = 'Odom stale'
            f.wait(lambda: marker in f.text(), 8)
            data = json.loads(f.path.read_text())['points']
            assert 'P02' in data and 'P03' not in data and 'P03_after_turn' not in data
        assert f.velocity == (0.0, 0.0, 0.0)
        print('PASS P03 route', mode, directory, flush=True)
    finally:
        f.close()


if __name__ == '__main__':
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert int(os.environ.get('ROS_DOMAIN_ID', '0')) >= 190
    rclpy.init()
    try:
        for mode in ('complete', 'stop', 'stale'):
            run(mode)
    finally:
        rclpy.shutdown()
