#!/usr/bin/env python3
"""Synthetic stopped-pose correction timing; no claim about real motor convergence."""

import fcntl
import json
import math
import os
from pathlib import Path
import sys
import time

from verify_current_frame_session import Fixture, events, stopped
from verify_current_frame_step import ODOM, rows, sensor, wait_for, write


def budget_sensor(folder):
    from rclpy.publisher import Publisher
    from unittest.mock import patch

    original = Publisher.publish
    last_motion, offset = None, 0.0

    def publish(publisher, message):
        nonlocal last_motion, offset
        if publisher.topic_name not in ODOM or (folder / 'mode').read_text() != 'settle':
            return original(publisher, message)
        velocity = message.twist.twist
        now = time.monotonic()
        if any((velocity.linear.x, velocity.linear.y, velocity.angular.z)):
            last_motion = now
        elif offset == 0.0 and last_motion is not None and now - last_motion > 1.10:
            offset = 0.0105
            write(
                folder / 'injection.json',
                dict(
                    time=time.time(), monotonic=now, stopped_duration=now - last_motion, yaw=offset
                ),
            )
        q = message.pose.pose.orientation
        z, w = q.z, q.w
        yaw = 2 * math.atan2(z, w) + offset
        q.z, q.w = math.sin(yaw / 2), math.cos(yaw / 2)
        original(publisher, message)
        q.z, q.w = z, w

    with patch.object(Publisher, 'publish', publish):
        sensor(folder)


class BudgetFixture(Fixture):
    def spawn(self, argv, name, *, input_pipe=False):
        if name == 'sensor':
            argv = [sys.executable, __file__, '--sensor', str(self.folder)]
        return super().spawn(argv, name, input_pipe=input_pipe)


def verify(fixture, case, code):
    folder = fixture.folder
    audit = rows(folder / 'session.jsonl')
    started = events(folder, 'step_started')[0]
    commands = rows(folder / 'commands.jsonl')
    end = events(folder, 'step_completed')[0] if case == 'settle' else audit[-1]
    elapsed = end['time'] - started['time']
    retries = events(folder, 'settle_retry')
    if case == 'settle':
        assert code == 0 and 10 < elapsed < 15, (code, elapsed)
        assert retries and retries[0]['attempt'] == 1
        checks = events(folder, 'final_check')
        assert not checks[0]['ready'] and checks[-1]['ready']
        assert 0.01 < abs(checks[0]['yaw_error_rad']) < math.radians(1)
    else:
        assert code == 2 and 'step timeout (15 seconds)' in end['reason'], (code, end)
        assert not events(folder, 'step_completed')
    nonzero = [r for r in commands if any(r['velocity'])]
    assert nonzero and commands[-1]['velocity'] == [0.0, 0.0, 0.0]
    stop = next(r for r in commands if r['time'] > nonzero[-1]['time'] and not any(r['velocity']))
    if case == 'timeout':
        assert 14.8 < stop['time'] - started['time'] < 15.3
    wait_for(lambda: fixture.snapshot()['publishers'] == 0, seconds=5)
    assert fixture.snapshot()['maxpub'] == 1
    stopped(fixture.recorder)
    closed = json.loads((fixture.recording / 'result.json').read_text())
    assert closed['closed_metadata'] and closed['exception'] is None
    with open('/tmp/distance_controller_survey_return.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    result = dict(
        case=case,
        exit=code,
        step_elapsed_to_terminal_s=elapsed,
        last_nonzero_after_start_s=nonzero[-1]['time'] - started['time'],
        first_final_zero_after_start_s=stop['time'] - started['time'],
        retries=retries,
        final_checks=events(folder, 'final_check'),
        terminal=audit[-1],
        pub0=True,
        maxpub=1,
        lastzero=True,
        lock_free=True,
        closed_metadata=True,
        synthetic_only=True,
    )
    write(folder / 'verification.json', result)
    print(json.dumps(result), flush=True)


def run_case(root, case):
    fixture = BudgetFixture(root / case)
    try:
        (fixture.folder / 'mode').write_text(case)
        fixture.start()
        fixture.step()
        fixture.moving()
        if case == 'settle':
            fixture.ready(2)
            fixture.send('quit')
        code = fixture.backend.wait(timeout=25)
        verify(fixture, case, code)
    finally:
        fixture.close()


if __name__ == '__main__':
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert os.environ.get('ROS_DOMAIN_ID') == '198'
    if sys.argv[1] == '--sensor':
        budget_sensor(Path(sys.argv[2]))
    else:
        for name in sys.argv[2:] or ['settle', 'timeout']:
            run_case(Path(sys.argv[1]), name)
