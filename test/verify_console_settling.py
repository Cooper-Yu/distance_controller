#!/usr/bin/env python3
"""Inject yaw changes in final zero-command interval; isolated synthetic ROS only."""

import json
import math
import os
import tempfile
import time
import rclpy
from verify_motion_console import Fixture


class SettlingFixture(Fixture):
    def __init__(self, directory, mode):
        self.mode, self.moved, self.zero_since, self.injections = mode, False, None, 0
        super().__init__(directory)

    def tick(self):
        super().tick()
        if any(abs(v) > 1e-5 for v in self.velocity):
            self.moved = True
            self.zero_since = None
        elif self.moved:
            self.zero_since = self.zero_since or time.monotonic()
            if time.monotonic() - self.zero_since > 0.8:
                self.moved = False
                if self.injections and self.mode != 'repeat':
                    return
                self.injections += 1
                if self.mode == 'stale':
                    self.feedback_enabled = False
                else:
                    self.yaw = math.pi / 2 + (0.04 if self.mode == 'large' else 0.0115)
                    if self.mode == 'stop':
                        self.send('stop')


def run(mode):
    f = SettlingFixture(tempfile.mkdtemp(prefix='console_settle_' + mode + '_'), mode)
    try:
        f.wait(lambda: 'motion> ' in f.text())
        f.send('move forward .10; record END')
        if mode == 'once':
            f.wait(lambda: 'Recorded: END ' in f.text(), 35)
            assert f.injections == 1
        else:
            f.wait(lambda: 'STOPPED:' in f.text(), 50)
            assert 'END' not in json.loads(f.path.read_text())['points']
        rows = [json.loads(x) for x in f.path.with_suffix('.jsonl').read_text().splitlines()]
        retries = [row for row in rows if row['event'] == 'settle_retry']
        assert len(retries) == ({'once': 1, 'repeat': 2}.get(mode, 0)), (mode, rows)
        if mode == 'once':
            final = [row for row in rows if row['event'] == 'final_check']
            assert not final[0]['ready'] and final[-1]['ready']
            assert abs(final[-1]['yaw_error_rad']) < 0.01
        assert f.velocity == (0.0, 0.0, 0.0)
        print('PASS settling', mode, f.path.parent, flush=True)
    finally:
        f.close()


if __name__ == '__main__':
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert int(os.environ.get('ROS_DOMAIN_ID', '0')) >= 190
    rclpy.init()
    try:
        for mode in ('once', 'repeat', 'large', 'stop', 'stale'):
            run(mode)
    finally:
        rclpy.shutdown()
