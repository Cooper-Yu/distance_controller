#!/usr/bin/env python3
"""READY-only pairing wait with actual timestamped ROS streams, no production mocks."""

import fcntl
import json
import os
from pathlib import Path
import sys
import time

from verify_current_frame_session import Fixture, events, stopped
from verify_current_frame_step import ODOM, rows, sensor, wait_for, write


def pairing_sensor(folder):
    from rclpy.publisher import Publisher
    from unittest.mock import patch

    original = Publisher.publish
    last_odom_ns, previous_odom_ns, sent_for_odom, pulse = None, None, None, None

    def publish(publisher, message):
        nonlocal last_odom_ns, previous_odom_ns, sent_for_odom, pulse
        mode_path = folder / 'pair_mode'
        mode = mode_path.read_text().strip() if mode_path.exists() else 'normal'
        topic = publisher.topic_name
        if topic not in (ODOM[0], '/scan_filtered'):
            return original(publisher, message)
        stamp = message.header.stamp
        now_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if topic == ODOM[0]:
            if mode == 'gap' and last_odom_ns and now_ns - last_odom_ns < 250_000_000:
                return None
            previous_odom_ns, last_odom_ns = last_odom_ns, now_ns
            return original(publisher, message)
        if mode == 'gap':
            if (
                previous_odom_ns is None
                or last_odom_ns - previous_odom_ns < 250_000_000
                or sent_for_odom == last_odom_ns
            ):
                return None
            sent_for_odom = last_odom_ns
            ns = previous_odom_ns + 165_000_000
            stamp.sec, stamp.nanosec = divmod(ns, 1_000_000_000)
        elif mode.startswith('recover'):
            if mode == 'recover0' or pulse == mode:
                return None
            pulse = mode
        with (folder / 'injected_scans.jsonl').open('a') as log:
            log.write(
                json.dumps(
                    dict(
                        mode=mode,
                        stamp_ns=stamp.sec * 1_000_000_000 + stamp.nanosec,
                        before_ns=previous_odom_ns,
                        after_ns=last_odom_ns,
                        emitted=time.time(),
                    )
                )
                + '\n'
            )
        return original(publisher, message)

    with patch.object(Publisher, 'publish', publish):
        sensor(folder)


class PairFixture(Fixture):
    def spawn(self, argv, name, *, input_pipe=False):
        if name == 'sensor':
            argv = [sys.executable, __file__, '--sensor', str(self.folder)]
        return super().spawn(argv, name, input_pipe=input_pipe)

    def mode(self, value):
        (self.folder / 'pair_mode').write_text(value)


def wait_event(fixture):
    wait_for(lambda: bool(events(fixture.folder, 'session_waiting')), fixture.backend, seconds=5)
    return events(fixture.folder, 'session_waiting')[-1]


def recover(fixture, old_token):
    fixture.send('step ' + old_token)
    fixture.mode('recover0')
    time.sleep(0.08)
    assert len(events(fixture.folder, 'session_ready')) == 1
    for count in (1, 2, 3):
        fixture.mode('recover' + str(count))
        wait_for(
            lambda: any(
                r['mode'] == 'recover' + str(count)
                for r in rows(fixture.folder / 'injected_scans.jsonl')
            ),
            fixture.backend,
        )
        time.sleep(0.04)
        if count < 3:
            assert len(events(fixture.folder, 'session_ready')) == 1
            assert not events(fixture.folder, 'step_started')
    new_token = fixture.ready(2)
    assert new_token != old_token
    restored = events(fixture.folder, 'session_ready')[-1]['recovered_scan_stamps']
    assert len(restored) == len(set(restored)) == 3
    assert min(restored) > events(fixture.folder, 'session_waiting')[-1]['scan_stamp_ns']
    fixture.send('step ' + old_token)
    time.sleep(0.05)
    assert not events(fixture.folder, 'step_started')
    fixture.mode('normal')
    fixture.send('quit')


def exercise(fixture, case):
    fixture.start()
    token = events(fixture.folder, 'session_ready')[-1]['ready_token']
    if case == 'active_gap':
        fixture.step()
        fixture.moving()
    fixture.mode('gap')
    if case == 'active_gap':
        return fixture.backend.wait(timeout=8)
    waiting = wait_event(fixture)
    assert fixture.snapshot()['publishers'] == 0
    if case == 'recover':
        recover(fixture, token)
    elif case == 'gid':
        (fixture.folder / 'mode').write_text('epoch')
    code = fixture.backend.wait(timeout=8)
    if case == 'timeout':
        failed = events(fixture.folder, 'session_failed')[-1]
        assert 1.9 <= failed['time'] - waiting['time'] <= 2.6
    return code


def verify(fixture, case, code):
    folder = fixture.folder
    expected = 0 if case == 'recover' else 2
    assert code == expected, (case, code, (folder / 'runner.log').read_text()[-1800:])
    wait_for(lambda: fixture.snapshot()['publishers'] == 0, seconds=5)
    terminal = rows(folder / 'session.jsonl')[-1]
    reasons = {
        'timeout': 'pairing wait exceeded 2 seconds',
        'active_gap': 'Odom bracket too wide for scan: 0.165 s',
        'gid': 'publisher identity/count changed',
    }
    if case in reasons:
        assert reasons[case] in terminal['reason']
    commands = rows(folder / 'commands.jsonl')
    snapshot = fixture.snapshot()
    assert snapshot['maxpub'] <= 1
    if case != 'active_gap':
        assert not commands and not events(folder, 'step_started')
    else:
        assert any(any(r['velocity']) for r in commands)
        assert commands[-1]['velocity'] == [0.0, 0.0, 0.0]
        assert not events(folder, 'session_waiting')
    stopped(fixture.recorder)
    closed = json.loads((fixture.recording / 'result.json').read_text())
    assert closed['closed_metadata'] and closed['exception'] is None
    with open('/tmp/distance_controller_survey_return.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    result = dict(
        case=case,
        exit=code,
        terminal=rows(folder / 'session.jsonl')[-1],
        waiting=events(folder, 'session_waiting'),
        ready=events(folder, 'session_ready'),
        commands=len(commands),
        pub0=True,
        maxpub=snapshot['maxpub'],
        closed_metadata=True,
        lock_free=True,
        synthetic_only=True,
    )
    write(folder / 'verification.json', result)
    print(json.dumps(result), flush=True)


def run_case(root, case):
    fixture = PairFixture(root / case)
    try:
        verify(fixture, case, exercise(fixture, case))
    finally:
        fixture.close()


if __name__ == '__main__':
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert os.environ.get('ROS_DOMAIN_ID') == '198'
    if sys.argv[1] == '--sensor':
        pairing_sensor(Path(sys.argv[2]))
    else:
        for case_name in sys.argv[2:] or ['recover', 'timeout', 'active_gap', 'gid']:
            run_case(Path(sys.argv[1]), case_name)
