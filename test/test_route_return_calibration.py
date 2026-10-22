"""Coordinate-transfer direction, provenance gates and continuous observation tests."""

from copy import deepcopy
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from route_return_calibration import compose, inverse, mapped_source, validate_report, verify_watch
from route_return_plan import ReturnSequence
from route_session_state import near
from test_route_return import source


def reference_fixture(binding, folder=None):
    from route_return_calibration import file_sha256

    completed = int(binding['completed_time'] * 1e9)
    reference = dict(
        odom_pose=deepcopy(binding['last_stop']),
        source_binding=deepcopy(binding),
        scan_bag_ns=completed,
        window_start_bag_ns=completed,
        window_end_bag_ns=completed + 1_000_000_000,
        scan_stamp_ns=binding['original_stamp_ns'],
        odom_stamp_ns=binding['original_stamp_ns'],
    )
    if folder is not None:
        # Synthetic identity bytes for attach tests; not a claim to parse a real bag.
        (folder / 'bag').mkdir(exist_ok=True)
        names = ('result.json', 'bag/metadata.yaml', 'bag/synthetic.db3')
        for name in names:
            (folder / name).write_text('synthetic closed evidence')
        reference.update(
            session=str(folder), files={name: file_sha256(folder / name) for name in names}
        )
    return reference


def report_fixture():
    original = source()
    binding = dict(
        source_hash=original.source_hash,
        session_id=original.session_id,
        completed_time=90.0,
        last_stop=asdict(original.last_stop),
        original_stamp_ns=original.stamp_ns,
    )
    report = dict(
        schema=2,
        source_binding=binding,
        gates=[],
        ambiguous=False,
        same_local_geometry_supported=True,
        best=dict(x=0.0007, y=-0.0008, yaw=0.0003, rmse=0.0096, overlap=0.955),
        epoch=dict(
            pose=dict(x=0.1, y=-0.2, yaw=0.4),
            stamp_ns=100000000000,
            frame='odom',
            child='base_link',
            gid='ab' * 24,
            stable=True,
            capture_started_unix=103.0,
            captured_unix=105.0,
        ),
        reference=reference_fixture(binding),
    )
    return original, binding, report


def test_se2_transfer_preserves_original_route_and_inverse_actions():
    original, binding, report = report_fixture()
    validate_report(report, binding, 107)
    mapped, transfer = mapped_source(original, report)
    fit = report['best']
    measured = Pose(**report['epoch']['pose'])
    assert near(
        compose(transfer, compose(original.last_stop, Pose(fit['x'], fit['y'], fit['yaw']))),
        measured,
        1e-9,
        1e-9,
    )
    assert near(compose(inverse(transfer), mapped.origin), original.origin, 1e-9, 1e-9)
    assert mapped.source_hash == original.source_hash and mapped.session_id == original.session_id
    assert mapped.stamp_ns == report['epoch']['stamp_ns'] and original.stamp_ns != mapped.stamp_ns
    assert [a.points for a in mapped.actions] == [a.points for a in original.actions]
    sequence = ReturnSequence(mapped)
    for action in reversed(mapped.actions):
        motion = sequence.begin(sequence.last_stop)
        if action.kind == 'path':
            assert motion.points[-1] == pytest.approx(
                (
                    math.cos(mapped.origin.yaw) * (action.start.x - mapped.origin.x)
                    + math.sin(mapped.origin.yaw) * (action.start.y - mapped.origin.y),
                    -math.sin(mapped.origin.yaw) * (action.start.x - mapped.origin.x)
                    + math.cos(mapped.origin.yaw) * (action.start.y - mapped.origin.y),
                )
            )
        sequence.finish(action.start)
    assert sequence.cursor == 0 and sequence.last_stop == mapped.origin


@pytest.mark.parametrize(
    'fault',
    [
        'hash',
        'uuid',
        'time',
        'expired',
        'ambiguous',
        'moving',
        'reference',
        'nan',
        'residual',
        'gid',
    ],
)
def test_calibration_rejects_invalid_provenance_and_measurements(fault):
    _, binding, report = report_fixture()
    if fault in ('hash', 'uuid', 'time'):
        report['source_binding'] = deepcopy(binding)
        key = {'hash': 'source_hash', 'uuid': 'session_id', 'time': 'completed_time'}[fault]
        report['source_binding'][key] = 'changed'
    elif fault == 'expired':
        report['epoch']['captured_unix'] = -100.0
    elif fault == 'ambiguous':
        report['ambiguous'] = True
    elif fault == 'moving':
        report['epoch']['stable'] = False
    elif fault == 'reference':
        report['reference']['odom_pose']['x'] += 0.02
    elif fault == 'nan':
        report['best']['x'] = math.nan
    elif fault == 'residual':
        report['best']['rmse'] = 0.03
    elif fault == 'gid':
        report['epoch']['gid'] = 'bad'
    with pytest.raises(ValueError):
        validate_report(report, binding, 107.0)


def watcher_fixture(path, report):
    epoch = report['epoch']
    values = list(epoch['pose'].values())
    topic = '/odometry/filtered'
    started = dict(event='started', schema=2, pid=123, process_start_ticks='10', wall_s=100.0)

    def sample(stamp):
        return dict(
            event='sample',
            wall_s=stamp,
            publisher_gids={topic: [[epoch['gid'], '/', 'ekf']]},
            latest={topic: dict(pose=values, frame='odom', child='base_link')},
            interval_bounds={topic: dict(min=values[:], max=values[:])},
            receipt_age_s={topic: 0.01},
            command_idle=dict(last_command=None),
        )

    prefix = (json.dumps(started) + '\n' + json.dumps(sample(104.0)) + '\n').encode()
    report['watch'] = dict(
        path=str(path),
        prefix_bytes=len(prefix),
        prefix_sha256=hashlib.sha256(prefix).hexdigest(),
        pid=123,
        process_start_ticks='10',
    )
    path.write_bytes(prefix + (json.dumps(sample(106.0)) + '\n').encode())
    return sample


def test_watcher_unchanged_prefix_and_continuous_stationary_interval(tmp_path):
    _, _, report = report_fixture()
    path = tmp_path / 'watch.jsonl'
    watcher_fixture(path, report)
    assert verify_watch(report, 107.0, False) == 106.0
    with pytest.raises(ValueError, match='stale'):
        verify_watch(report, 111.0, False)
    raw = path.read_bytes()
    path.write_bytes(raw.replace(b'ekf', b'bad', 1))
    with pytest.raises(ValueError, match='prefix'):
        verify_watch(report, 107.0, False)


@pytest.mark.parametrize(
    'event',
    ['motion', 'command', 'gid', 'gap', 'reset', 'finished', 'bad_odom', 'bad_cmd', 'invalid_odom'],
)
def test_watcher_rejects_motion_and_epoch_discontinuity(tmp_path, event):
    _, _, report = report_fixture()
    path = tmp_path / 'watch.jsonl'
    sample = watcher_fixture(path, report)
    topic = '/odometry/filtered'
    row = sample(106.5)
    if event == 'motion':
        row['interval_bounds'][topic]['max'][0] += 0.004
    elif event == 'command':
        row = dict(event='command_state', wall_s=106.5, command=dict(nonzero=True))
    elif event == 'gid':
        row['publisher_gids'][topic][0][0] = 'cd' * 24
    elif event == 'gap':
        row = dict(event='feedback_gap', wall_s=106.5, topic=topic)
    elif event == 'reset':
        row = dict(event='odom_change', wall_s=106.5, topic=topic)
    elif event == 'bad_odom':
        row = dict(event='nonfinite_odom', wall_s=106.5, topic=topic)
    elif event == 'invalid_odom':
        row = dict(event='invalid_odom', wall_s=106.5, topic=topic)
    elif event == 'bad_cmd':
        row = dict(event='nonfinite_cmd', wall_s=106.5)
    elif event == 'finished':
        row = dict(event='finished', wall_s=106.5)
    with path.open('a') as output:
        output.write(json.dumps(row) + '\n')
    with pytest.raises(ValueError):
        verify_watch(report, 107.0, False)


def test_production_scan_geometry_and_json_roundtrip(tmp_path):
    import importlib.util
    import numpy as np
    from types import SimpleNamespace

    location = Path(__file__).resolve().parents[1] / 'tools' / 'task6_ops' / 'scan_compare.py'
    spec = importlib.util.spec_from_file_location('scan_compare_checked', location)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    cloud = np.vstack(
        [
            np.column_stack((np.linspace(-0.35, 1.2, 95), np.full(95, 0.4))),
            np.column_stack((np.full(90, 1.2), np.linspace(-0.8, 0.4, 90))),
            np.column_stack((np.linspace(-0.35, 0.1, 40), np.full(40, -0.8))),
        ]
    )
    offset = np.array([0.016, -0.011])
    angle = np.deg2rad(0.7)
    current = (cloud - offset) @ module.rot(angle)
    result = module.compare(current, cloud)
    assert result['gates'] == []
    assert [result['best']['x'], result['best']['y']] == pytest.approx(offset, abs=0.004)
    scan = SimpleNamespace(
        ranges=[1.0, 1.0], angle_min=0.0, angle_increment=np.pi / 2, range_min=0.1, range_max=5.0
    )
    assert module.points(scan, np.eye(4)) == pytest.approx(np.array([[1, 0], [0, 1]]))
    result['same_local_geometry_supported'] = np.bool_(True)
    module.save_report(tmp_path / 'report.json', result)
    assert (
        json.loads((tmp_path / 'report.json').read_text())['same_local_geometry_supported'] is True
    )


def test_watcher_rejects_missing_initial_capture_interval(tmp_path):
    _, _, report = report_fixture()
    path = tmp_path / 'watch.jsonl'
    watcher_fixture(path, report)
    report['epoch']['capture_started_unix'] = 100.0
    with pytest.raises(ValueError, match='initial capture'):
        verify_watch(report, 107.0, False)


def test_source_binding_rejects_evidence_replaced_after_load(tmp_path):
    from dataclasses import replace

    from route_return_calibration import source_binding

    path = tmp_path / 'points.json'
    path.write_bytes(b'{}')
    audit = b'{"event":"route_completed","time":90.0}\n'.replace(b'\\n', b'\n')
    path.with_suffix('.jsonl').write_bytes(audit)
    original = replace(source(), source_hash=hashlib.sha256(b'{}\n' + audit).hexdigest())
    assert source_binding(path, original)['completed_time'] == 90.0
    path.write_bytes(b'{"changed":true}')
    with pytest.raises(ValueError, match='source evidence changed'):
        source_binding(path, original)


def test_transform_uses_actual_reference_scan_pose_and_continuous_yaw():
    original, binding, report = report_fixture()
    reference = deepcopy(binding['last_stop'])
    reference['x'] += 0.003
    reference['yaw'] -= 2 * math.pi
    report['reference']['odom_pose'] = reference
    validate_report(report, binding, 107)
    mapped, transfer = mapped_source(original, report)
    reference['yaw'] += 2 * math.pi
    fit = report['best']
    assert near(
        compose(transfer, compose(Pose(**reference), Pose(fit['x'], fit['y'], fit['yaw']))),
        Pose(**report['epoch']['pose']),
        1e-9,
        1e-9,
    )
    assert mapped.origin.yaw == pytest.approx(original.origin.yaw + transfer.yaw)
