"""Isolated ROS regression checks for repository operational support."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]
OPS = ROOT / 'tools' / 'task6_ops'
sys.path.insert(0, str(OPS))
from common import singleton, write_json  # noqa: E402


def environment():
    return dict(os.environ, ROS_DOMAIN_ID='223', ROS_LOCALHOST_ONLY='1')


def stop_process(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


def test_singleton_and_atomic_report(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.setenv('ROS_DOMAIN_ID', '223')
    first = singleton('test')
    with pytest.raises(RuntimeError, match='another instance'):
        singleton('test')
    monkeypatch.setenv('ROS_DOMAIN_ID', '0223')
    with pytest.raises(RuntimeError, match='another instance'):
        singleton('test')
    first.close()
    singleton('test').close()
    monkeypatch.setenv('ROS_DOMAIN_ID', '233')
    with pytest.raises(ValueError, match='0..232'):
        singleton('test')
    path = tmp_path / 'health.json'
    write_json(path, {'count': 1})
    with pytest.raises(TypeError):
        write_json(path, {'bad': object()})
    assert json.loads(path.read_text()) == {'count': 1}


def test_recorder_throttle_lock_close_and_capacity(tmp_path):  # noqa: PLR0915
    fixture = subprocess.Popen(
        [sys.executable, str(ROOT / 'test' / 'task6_ops_fixture.py')],
        env=environment(),
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    recorder = None
    try:
        time.sleep(1)
        root = tmp_path / 'recording'
        args = [
            sys.executable,
            str(OPS / 'evidence_recorder.py'),
            '--root',
            str(root),
            '--seconds',
            '6',
            '--image-hz',
            '2',
        ]
        recorder = subprocess.Popen(
            args,
            env=environment(),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        deadline = time.monotonic() + 5
        while not (root / 'latest_session.txt').exists() and time.monotonic() < deadline:
            time.sleep(0.05)
        assert (root / 'latest_session.txt').exists()
        duplicate = subprocess.run(
            args, env=environment(), capture_output=True, text=True, timeout=5
        )
        assert duplicate.returncode != 0 and 'another instance' in duplicate.stderr
        output, _ = recorder.communicate(timeout=12)
        assert recorder.returncode == 0, output
        session = Path((root / 'latest_session.txt').read_text())
        result = json.loads((session / 'result.json').read_text())
        counts = result['message_counts']
        assert result['closed_metadata'] and result['exception'] is None
        assert 8 <= counts['/image_raw/compressed'] <= 13
        assert counts['/scan'] >= 45 and counts['/odometry/filtered'] >= 90
        assert counts['/tf_static'] >= 1 and counts['/cmd_vel'] == 0
        assert result['image_dropped'] > 30
        assert result['image_received'] == result['image_dropped'] + counts['/image_raw/compressed']
        info = subprocess.run(
            ['ros2', 'bag', 'info', str(session / 'bag')],
            env=environment(),
            capture_output=True,
            text=True,
            timeout=10,
        )
        assert info.returncode == 0, info.stderr
        capped = subprocess.run(
            args + ['--max-mib', '.1'],
            env=environment(),
            capture_output=True,
            text=True,
            timeout=10,
        )
        assert capped.returncode == 0, capped.stdout + capped.stderr
        final = Path((root / 'latest_session.txt').read_text())
        cap = json.loads((final / 'result.json').read_text())
        assert cap['stop_reason'] == 'bag_size_limit' and cap['closed_metadata']
        assert final != session
    finally:
        if recorder is not None:
            stop_process(recorder)
        stop_process(fixture)


def test_non_tty_signal_shutdown(tmp_path):
    root = tmp_path / 'signal_recording'
    recorder = subprocess.Popen(
        [sys.executable, str(OPS / 'evidence_recorder.py'), '--root', str(root), '--seconds', '30'],
        env=environment(),
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        deadline = time.monotonic() + 5
        session = None
        while time.monotonic() < deadline:
            pointer = root / 'latest_session.txt'
            if pointer.exists():
                session = Path(pointer.read_text())
                if (session / 'health.json').exists():
                    break
            time.sleep(0.05)
        assert session is not None and (session / 'health.json').exists()
        recorder.send_signal(signal.SIGINT)
        assert recorder.wait(timeout=5) == 0
        result = json.loads((session / 'result.json').read_text())
        assert result['closed_metadata'] and result['stop_reason'] == 'requested_stop'
    finally:
        stop_process(recorder)
    output = tmp_path / 'watch.jsonl'
    watcher = subprocess.run(
        [
            sys.executable,
            str(OPS / 'odom_reset_watch.py'),
            '--output',
            str(output),
            '--seconds',
            '1',
            '--base-topic',
            '/rosbot_xl_base_controller/odom',
        ],
        env=environment(),
        stdin=subprocess.DEVNULL,
        capture_output=True,
        text=True,
        timeout=8,
    )
    assert watcher.returncode == 0, watcher.stdout + watcher.stderr
    summary = json.loads(output.with_suffix('.summary.json').read_text())
    assert summary['reason'] == 'duration' and summary['causal_conclusion'].startswith(
        'not established'
    )
