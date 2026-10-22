"""Schema2 watcher invalid-feedback evidence needed by calibration."""

import json
from pathlib import Path
import sys
import time

from nav_msgs.msg import Odometry
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools/task6_ops'))
from odom_reset_watch import Watch


@pytest.mark.parametrize('fault', ['quaternion', 'stale', 'duplicate'])
def test_invalid_feedback_emits_persistent_rejection(tmp_path, fault):
    path = tmp_path / 'watch.jsonl'
    watcher = Watch(path, 65536)
    message = Odometry()
    message.header.frame_id, message.child_frame_id = 'odom', 'base_link'
    message.pose.pose.orientation.w = 1.0
    stamp = time.time_ns()
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
    watcher.odom('/odometry/filtered', message)
    if fault == 'quaternion':
        message.pose.pose.orientation.w = 0.0
    elif fault == 'stale':
        message.header.stamp.sec -= 1
    watcher.odom('/odometry/filtered', message)
    watcher.close('test')
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    expected = 'odom_change' if fault == 'duplicate' else 'invalid_odom'
    assert any(row['event'] == expected for row in rows)
