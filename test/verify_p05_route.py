#!/usr/bin/env python3
"""Full fixed-P01 P05 route and final-strafe interruption in isolated synthetic ROS."""

import json
import math
import os
import tempfile
import rclpy
from verify_motion_console import Fixture


def run(mode):
    directory = tempfile.mkdtemp(prefix='p05_route_' + mode + '_')
    f = Fixture(directory)
    try:
        f.wait(lambda: 'motion> ' in f.text())
        f.send('path_plan P05')
        f.wait(lambda: 'P05 path: 9 points' in f.text())
        assert not f.commands
        f.send('path P05')
        f.wait(lambda: 'Recorded: P04 ' in f.text(), 100)
        if mode == 'complete':
            f.wait(lambda: 'P05 route completed; stopped' in f.text(), 40)
            data = json.loads(f.path.read_text())['points']
            for label, x, y in [
                ('P04', 1.781439293373129, -0.5239300031759923),
                ('P05', 1.3921754778962125, -0.5250984301851296),
            ]:
                p = data[label]['relative_to_p01']
                assert math.hypot(p['x_m'] - x, p['y_m'] - y) < 0.005, p
                assert abs(p['yaw_deg'] + 90) < math.degrees(0.01), p
            f.send('path P05')
            f.wait(lambda: 'already recorded' in f.text())
        else:
            f.wait(lambda: f.y < 3.68, 30)
            f.send('stop')
            f.wait(lambda: 'Operator stop' in f.text(), 8)
            assert 'P05' not in json.loads(f.path.read_text())['points']
        assert f.velocity == (0.0, 0.0, 0.0)
        print('PASS P05', mode, directory, flush=True)
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
