#!/usr/bin/env python3
"""Inject 12 mm translation during P03 rotation on an isolated synthetic ROS plant."""

import json
import math
import os
import tempfile
import rclpy
from verify_motion_console import Fixture


class DriftingFixture(Fixture):
    def tick(self):
        before = self.yaw
        super().tick()
        # Fixture starts facing +Y; world +X is P01-right.
        self.x += abs(self.yaw - before) * 0.012 / (math.pi / 2)


if __name__ == '__main__':
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert int(os.environ.get('ROS_DOMAIN_ID', '0')) >= 190
    rclpy.init()
    f = DriftingFixture(tempfile.mkdtemp(prefix='post_turn_reconnect_'))
    try:
        f.wait(lambda: 'motion> ' in f.text())
        f.send('path P04')
        f.wait(lambda: 'P04 route completed; stopped' in f.text(), 110)
        rows = [json.loads(line) for line in f.path.with_suffix('.jsonl').read_text().splitlines()]
        event = next(row for row in rows if row['event'] == 'post_turn_reconnect')
        displacement = math.dist(event['original']['points'][0], event['adjusted']['points'][0])
        assert 0.01 < displacement < 0.02, displacement
        assert event['original']['points'][-1] == event['adjusted']['points'][-1]
        assert f.velocity == (0.0, 0.0, 0.0)
        print('PASS injected post-turn drift', displacement, f.path.parent)
    finally:
        f.close()
        rclpy.shutdown()
