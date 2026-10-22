#!/usr/bin/env python3
"""Isolated installed session checks; synthetic sensors, real ROS and closed recorder.

Requires explicit localhost/domain 198. Never connects to real robot or old route evidence.
"""

import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from verify_current_frame_step import ODOM, ROOT, rows, sensor, wait_for, write

RUNNER = (
    ROOT.parents[1] / 'install/distance_controller/lib/distance_controller/current_frame_session'
)
RECORDER = ROOT / 'tools/task6_ops/evidence_recorder.py'


def sensor_with_content_fault(folder):
    """Inject one same-GID/same-stamp pose change in the synthetic publisher only."""
    from rclpy.publisher import Publisher
    from unittest.mock import patch

    original = Publisher.publish
    injected = False

    def publish(publisher, message):
        nonlocal injected
        original(publisher, message)
        mode = folder / 'mode'
        if (
            not injected
            and publisher.topic_name == ODOM[0]
            and mode.exists()
            and mode.read_text() == 'filtered_content'
        ):
            old_x = message.pose.pose.position.x
            message.pose.pose.position.x += 0.001
            original(publisher, message)
            message.pose.pose.position.x = old_x
            injected = True

    with patch.object(Publisher, 'publish', publish):
        sensor(folder)


def events(folder, name):
    return [r for r in rows(folder / 'session.jsonl') if r['event'] == name]


def stopped(child):
    if child.poll() is None:
        child.send_signal(signal.SIGINT)
        try:
            child.wait(timeout=8)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait()
            raise AssertionError('Child required SIGKILL') from None


class Fixture:
    def __init__(self, folder):
        self.folder = folder
        folder.mkdir(parents=True)
        self.children, self.handles = [], []
        self.backend = None

    def spawn(self, argv, name, *, input_pipe=False):
        handle = (self.folder / (name + '.log')).open('w')
        self.handles.append(handle)
        child = subprocess.Popen(
            argv,
            stdin=subprocess.PIPE if input_pipe else subprocess.DEVNULL,
            stdout=handle,
            stderr=subprocess.STDOUT,
            text=True,
        )
        self.children.append(child)
        return child

    def start(self, *, limit=60):
        sensors = self.spawn([sys.executable, __file__, '--sensor', str(self.folder)], 'sensor')
        wait_for(lambda: (self.folder / 'sensor.json').exists(), sensors)
        wait_for(lambda: all(len(x) == 1 for x in self.snapshot()['gids'].values()), sensors)
        self.initial = self.snapshot()
        self.recorder = self.spawn(
            [
                sys.executable,
                str(RECORDER),
                '--seconds',
                '90',
                '--root',
                str(self.folder / 'recording'),
            ],
            'recorder',
        )
        wait_for(lambda: (self.folder / 'recording/latest_session.txt').exists(), self.recorder)
        self.recording = Path((self.folder / 'recording/latest_session.txt').read_text().strip())
        wait_for(
            lambda: (
                (self.recording / 'health.json').exists()
                and all(
                    json.loads((self.recording / 'health.json').read_text())['message_counts'][t]
                    > 0
                    for t in (*ODOM, '/scan_filtered', '/tf_static')
                )
            ),
            self.recorder,
        )
        self.backend = self.spawn(
            [
                str(RUNNER),
                '--step-distance',
                '.03',
                '--speed',
                '.01',
                '--odom-publisher-gid',
                self.initial['gids'][ODOM[0]][0],
                '--raw-odom-publisher-gid',
                self.initial['gids'][ODOM[1]][0],
                '--output',
                str(self.folder / 'session.json'),
                '--supervised-manual',
                '--max-seconds',
                str(limit),
            ],
            'runner',
            input_pipe=True,
        )
        self.ready(1)

    def snapshot(self):
        return json.loads((self.folder / 'sensor.json').read_text())

    def send(self, line):
        self.backend.stdin.write(line + '\n')
        self.backend.stdin.flush()

    def ready(self, count):
        wait_for(lambda: len(events(self.folder, 'session_ready')) >= count, self.backend)
        wait_for(lambda: self.snapshot()['publishers'] == 0, self.backend, seconds=3)
        assert self.snapshot()['velocity'] == [0.0, 0.0, 0.0]
        return events(self.folder, 'session_ready')[-1]['ready_token']

    def step(self):
        token = events(self.folder, 'session_ready')[-1]['ready_token']
        self.send('step ' + token)
        return token

    def moving(self):
        wait_for(
            lambda: any(any(r['velocity']) for r in rows(self.folder / 'commands.jsonl')),
            self.backend,
        )

    def close(self):
        for child in reversed(self.children):
            stopped(child)
        for handle in self.handles:
            handle.close()


def exercise(fixture, case):
    fixture.start(limit=4 if case == 'deadline' else 60)
    if case in ('ready_epoch', 'filtered_content'):
        (fixture.folder / 'mode').write_text('epoch' if case == 'ready_epoch' else case)
    elif case == 'deadline':
        pass
    else:
        token = fixture.step()
        fixture.moving()
        if case == 'cancel':
            fixture.backend.send_signal(signal.SIGINT)
        elif case in ('stale', 'filtered_stale'):
            (fixture.folder / 'mode').write_text(case)
        else:
            if case == 'busy_duplicate':
                fixture.send('step ' + token)
                fixture.send('step ' + token)
            new_token = fixture.ready(2)
            assert new_token != token
            if case == 'two_steps':
                fixture.step()
                fixture.ready(3)
            else:
                # No queued command may start after READY; old tokens remain consumed.
                fixture.send('step ' + token)
                time.sleep(0.5)
                assert fixture.backend.poll() is None
                assert len(events(fixture.folder, 'step_started')) == 1
            fixture.send('quit')
    return fixture.backend.wait(timeout=20)


def verify(fixture, case, code):
    folder = fixture.folder
    audit = rows(folder / 'session.jsonl')
    success = case in ('two_steps', 'busy_duplicate')
    assert code == (0 if success else 2), (case, code, (folder / 'runner.log').read_text()[-1500:])
    wait_for(lambda: fixture.snapshot()['publishers'] == 0, seconds=5)
    snapshot = fixture.snapshot()
    commands = rows(folder / 'commands.jsonl')
    assert snapshot['maxpub'] <= 1
    assert not commands or commands[-1]['velocity'] == [0.0, 0.0, 0.0]
    assert audit[-1]['event'] == ('session_closed' if success else 'session_failed')
    starts, ends = events(folder, 'step_started'), events(folder, 'step_completed')
    ready = events(folder, 'session_ready')
    anchor = ready[0]['anchor']
    assert all(r['anchor'] == anchor for r in ready)
    if success:
        expected = 2 if case == 'two_steps' else 1
        assert len(starts) == len(ends) == expected
        # Actual Console path dispatch must keep the original frozen frame.
        dispatched = events(folder, 'started')
        assert len(dispatched) == expected
        assert all(r['start'] == anchor for r in dispatched)
        assert all(r['motion']['kind'] == 'path' and r['motion']['angle'] == 0 for r in dispatched)
        for row in dispatched:
            points = row['motion']['points']
            assert points[0][1] == points[-1][1] == 0.0
            assert abs(points[-1][0] - points[0][0] + 0.03) < 1e-12
        if expected == 2:
            assert dispatched[1]['motion']['points'][0][0] < -0.02
    if case in ('ready_epoch', 'filtered_content', 'deadline'):
        assert not starts and not commands
    stopped(fixture.recorder)
    result = json.loads((fixture.recording / 'result.json').read_text())
    assert result['closed_metadata'] and result['exception'] is None
    assert all(result['message_counts'][t] > 0 for t in (*ODOM, '/scan_filtered', '/tf_static'))
    if commands:
        assert result['message_counts']['/cmd_vel'] > 0
    assert (fixture.recording / 'bag/metadata.yaml').is_file()
    with open('/tmp/distance_controller_survey_return.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    result = dict(
        case=case,
        exit=code,
        complete=len(ends),
        started=len(starts),
        ready=len(ready),
        anchor=anchor,
        lastzero=not commands or not any(commands[-1]['velocity']),
        pub0=True,
        maxpub=snapshot['maxpub'],
        lock_free=True,
        closed_bag=str(fixture.recording),
        terminal=audit[-1],
        command_count=len(commands),
        scope='Synthetic ROS feedback only; no physical clearance or motor qualification',
    )
    write(folder / 'verification.json', result)
    print(json.dumps(result), flush=True)


def run_case(root, case):
    fixture = Fixture(root / case)
    try:
        code = exercise(fixture, case)
        verify(fixture, case, code)
    finally:
        fixture.close()


if __name__ == '__main__':
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert os.environ.get('ROS_DOMAIN_ID') == '198'
    if sys.argv[1] == '--sensor':
        sensor_with_content_fault(Path(sys.argv[2]))
    else:
        for mode in sys.argv[2:] or [
            'two_steps',
            'ready_epoch',
            'filtered_content',
            'stale',
            'filtered_stale',
            'cancel',
            'busy_duplicate',
            'deadline',
        ]:
            run_case(Path(sys.argv[1]), mode)
