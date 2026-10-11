#!/usr/bin/env python3
"""Installed CLI continuous path test on an isolated synthetic ROS domain."""

import json
import math
import os
from pathlib import Path
import tempfile

import rclpy
from verify_motion_console import Fixture


def run(directory, mode):
    f = Fixture(directory)
    try:
        f.wait(lambda: 'motion> ' in f.text())
        f.send('path_plan')
        f.wait(lambda: '19 points' in f.text())
        assert not f.commands
        f.send('path P02')
        if mode == 'complete':
            f.wait(lambda: f.y > 2.25, 30)
            f.x -= 0.008  # Small lateral disturbance, still inside continuity limits.
            f.yaw += 0.02
            f.wait(lambda: 'P02 comparison:' in f.text(), 60)
            data = json.loads(f.path.read_text())
            point = data['points']['P02']['relative_to_p01']
            assert (
                math.hypot(point['x_m'] - 0.840238839617, point['y_m'] + 0.002600028527) < 0.005
            ), point
            assert abs(point['yaw_deg']) < math.degrees(0.01), point
            events = [json.loads(v) for v in f.path.with_suffix('.jsonl').read_text().splitlines()]
            assert len([e for e in events if e['event'] == 'started']) == 1
            assert len([e for e in events if e['event'] == 'completed']) == 1
            running = [e for e in events if e['event'] == 'progress']
            assert all(math.hypot(*e['command'][:2]) > 0.001 for e in running[2:-3])
            peak = max(math.hypot(*v[:2]) for v in f.commands)
            speed = float(os.getenv('MOTION_TEST_SPEED', '0.04'))
            assert 0.95 * speed <= peak <= speed + 1e-6, peak
            print('Verified peak command speed:', peak)
            count = len(f.commands)
            f.send('path P02')
            f.wait(lambda: 'already recorded' in f.text())
            assert len(f.commands) == count
            print(
                'PASS fixed rotated P01, disturbance correction, continuous path, endpoint, repeat rejection',
                point,
            )
        else:
            f.wait(lambda: f.y > 2.08, 20)
            if mode == 'stop':
                f.send('stop')
                marker = 'Operator stop'
            else:
                f.feedback_enabled = False
                marker = 'Odom stale'
            f.wait(lambda: marker in f.text(), 8)
            assert f.velocity == (0.0, 0.0, 0.0)
            assert 'P02' not in json.loads(f.path.read_text())['points']
            if mode == 'stop':
                f.send('path P02')
                f.wait(lambda: 'within 1 cm' in f.text())
            print('PASS path cancellation/no false P02:', mode)
    finally:
        print('Runtime evidence:', Path(directory) / 'console.log')
        f.close()


if __name__ == '__main__':
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert int(os.environ.get('ROS_DOMAIN_ID', '0')) >= 190
    rclpy.init()
    try:
        for mode in ('complete', 'stop', 'stale'):
            run(tempfile.mkdtemp(prefix='teaching_path_' + mode + '_'), mode)
    finally:
        rclpy.shutdown()
