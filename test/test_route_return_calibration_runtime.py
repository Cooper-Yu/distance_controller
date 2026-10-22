"""Isolated ROS attachment only; synthetic geometry is not robot recovery evidence."""

import importlib.util
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

from test_route_return import fixture
from test_route_return_calibration import reference_fixture

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from route_return_calibration import source_binding, verify_watch, watch_binding
from route_return_plan import load_source

ROOT = Path(__file__).resolve().parents[1]


def test_calibrated_attach_quit_preserves_original_evidence(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '223')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    ops = ROOT / 'tools' / 'task6_ops'
    spec = importlib.util.spec_from_file_location('scan_compare_runtime', ops / 'scan_compare.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    log = tmp_path / 'watch.jsonl'
    children = []
    try:
        commands = [
            [str(ROOT / 'test' / 'task6_ops_fixture.py')],
            [
                str(ops / 'odom_reset_watch.py'),
                '--output',
                str(log),
                '--seconds',
                '30',
                '--base-topic',
                '/rosbot_xl_base_controller/odom',
            ],
        ]
        for command in commands:
            children.append(
                subprocess.Popen(
                    [sys.executable, *command],
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            )
        time.sleep(1)
        *_, epoch = module.current_capture()
        report = dict(epoch=epoch, watch=watch_binding(log, epoch['capture_started_unix']))
        time.sleep(2.1)
        verify_watch(report)
        run_attach(tmp_path, report)
    finally:
        for child in children:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
        for child in children:
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=5)


def run_attach(folder, report):
    book, rows = fixture()
    rows[-1]['time'] = time.time() - 30
    path = folder / 'points.json'
    path.write_text(json.dumps(book))
    path.with_suffix('.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))
    original = load_source(path)
    report.update(
        schema=2,
        source_binding=source_binding(path, original),
        gates=[],
        ambiguous=False,
        same_local_geometry_supported=True,
        best=dict(x=0.0, y=0.0, yaw=0.0, rmse=0.001, overlap=1.0),
        reference=reference_fixture(source_binding(path, original), folder),
    )
    report_path = folder / 'synthetic_geometry_report.json'
    report_path.write_text(json.dumps(report))
    before = path.read_bytes(), path.with_suffix('.jsonl').read_bytes()
    result = subprocess.run(
        [
            sys.executable,
            str(ROOT / 'tools' / 'route_return_runner.py'),
            '--source',
            str(path),
            '--confirm-source-session',
            original.session_id,
            '--calibration',
            str(report_path),
            '--odom-publisher-gid',
            report['epoch']['gid'],
            '--output',
            str(folder / 'return.json'),
            '--laser-log-only',
        ],
        input='quit\n',
        capture_output=True,
        text=True,
        timeout=15,
    )
    (folder / 'runner.log').write_text(result.stdout + result.stderr)
    assert result.returncode == 0, result.stdout + result.stderr
    evidence = json.loads((folder / 'return.calibration.json').read_text())
    assert evidence['source']['source_hash'] == original.source_hash
    assert before == (path.read_bytes(), path.with_suffix('.jsonl').read_bytes())
    assert not path.with_suffix('.return_claim.json').exists()
    rows = [json.loads(line) for line in (folder / 'return.jsonl').read_text().splitlines()]
    assert any(row['event'] == 'return_epoch_calibrated' for row in rows)
    assert not any(row['event'] == 'return_action_started' for row in rows)
