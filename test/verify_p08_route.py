#!/usr/bin/env python3
"""Full fixed-P01 P08 route and final-forward interruption in isolated synthetic ROS."""

import json
import math
import os
import tempfile
import rclpy
from verify_motion_console import Fixture


def run(mode):
    directory = tempfile.mkdtemp(prefix='p08_route_' + mode + '_')
    f = Fixture(directory)
    try:
        f.wait(lambda: 'motion> ' in f.text())
        f.send('path_plan P08')
        f.wait(lambda: 'P08 path: 12 points' in f.text())
        assert not f.commands
        f.send('path P08')
        f.wait(lambda: 'Recorded: P07 ' in f.text(), 150)
        if mode == 'complete':
            f.wait(lambda: 'P08 route completed; stopped' in f.text(), 40)
            data = json.loads(f.path.read_text())['points']
            for label, x, y in [
                ('P04', 1.781439293373129, -0.5239300031759923),
                ('P07', 1.823473683378812, -1.0444770488254893),
                ('P08', 1.8030806164410214, -1.5875291803742129),
            ]:
                p = data[label]['relative_to_p01']
                assert math.hypot(p['x_m'] - x, p['y_m'] - y) < 0.005, p
                assert abs(p['yaw_deg'] + 90) < math.degrees(0.01), p
            f.send('path P08')
            f.wait(lambda: 'already recorded' in f.text())
        else:
            f.wait(lambda: f.x > 2.18, 30)
            f.send('stop')
            f.wait(lambda: 'Operator stop' in f.text(), 8)
            assert 'P08' not in json.loads(f.path.read_text())['points']
        assert f.velocity == (0.0, 0.0, 0.0)
        print('PASS P08', mode, directory, flush=True)
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
