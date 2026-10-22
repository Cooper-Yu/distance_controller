"""Evidence-bound SE(2) coordinate transfer after an observed odometry epoch reset.

This maps the existing physical P01 route into a new odom frame. It never changes
source evidence, creates a new physical P01, publishes motion, or proves global localization.
"""

from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
import time

from action_plan import Pose
from route_return_plan import pose
from route_session_state import near


def compose(a, b):
    """T_a_b composition, preserving the continuous yaw representation."""
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose(a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, a.yaw + b.yaw)


def inverse(a):
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose(-c * a.x - s * a.y, s * a.x - c * a.y, -a.yaw)


def source_binding(path, source):
    path = Path(path)
    book, audit = path.read_bytes(), path.with_suffix('.jsonl').read_bytes()
    if hashlib.sha256(book + b'\n' + audit).hexdigest() != source.source_hash:
        raise ValueError('Original source evidence changed after loading')
    rows = [json.loads(line) for line in audit.splitlines()]
    final = rows[-1]
    ended = final.get('time')
    if (
        final.get('event') != 'route_completed'
        or not isinstance(ended, (int, float))
        or not math.isfinite(ended)
    ):
        raise ValueError('Source completed boundary requires a finite event time')
    return dict(
        source_hash=source.source_hash,
        session_id=source.session_id,
        completed_time=ended,
        last_stop=asdict(source.last_stop),
        original_stamp_ns=source.stamp_ns,
    )


def watch_rows(path):
    raw = Path(path).read_bytes()
    boundary = raw.rfind(b'\n') + 1
    if not boundary:
        raise ValueError('Watcher has no complete evidence rows')
    raw = raw[:boundary]
    return raw, [json.loads(line) for line in raw.splitlines()]


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def validate_reference(reference, binding):
    """Bind the selected scan, its paired odom and window to the original final stop."""
    if reference.get('source_binding') != binding:
        raise ValueError('Reference source boundary mismatch')
    keys = (
        'scan_bag_ns',
        'scan_stamp_ns',
        'odom_stamp_ns',
        'window_start_bag_ns',
        'window_end_bag_ns',
    )
    if any(type(reference.get(k)) is not int or reference[k] <= 0 for k in keys):
        raise ValueError('Reference timestamp evidence missing')
    start, end = reference['window_start_bag_ns'], reference['window_end_bag_ns']
    completed = int(binding['completed_time'] * 1e9)
    if (
        not completed - 500_000_000
        <= start
        <= reference['scan_bag_ns']
        <= end
        <= completed + 3_000_000_000
    ):
        raise ValueError('Reference window outside original completion boundary')
    if (
        not binding['original_stamp_ns'] - 100_000_000
        <= reference['scan_stamp_ns']
        <= binding['original_stamp_ns'] + 3_500_000_000
    ):
        raise ValueError('Reference scan belongs to another odom time interval')
    if abs(reference['scan_stamp_ns'] - reference['odom_stamp_ns']) > 50_000_000:
        raise ValueError('Reference selected scan/odom pairing exceeds 50 ms')
    if not near(pose(reference['odom_pose']), pose(binding['last_stop']), 0.005, 0.01):
        raise ValueError('Reference scan window differs from original final stop')


def verify_reference_files(reference, source_path):
    """Check closed evidence bytes once at attach, not inside a velocity callback."""
    session = Path(reference['session']).resolve(strict=True)
    if session != Path(source_path).resolve(strict=True).parent:
        raise ValueError('Reference bag must belong to the original source session directory')
    files = reference.get('files', {})
    if not {'result.json', 'bag/metadata.yaml'} <= files.keys() or not any(
        key.startswith('bag/') and key.endswith('.db3') for key in files
    ):
        raise ValueError('Reference closed bag identity is incomplete')
    for name, expected in files.items():
        path = (session / name).resolve(strict=True)
        if session not in path.parents or file_sha256(path) != expected:
            raise ValueError('Reference closed bag bytes changed')


def watch_binding(path, capture_started):
    raw, rows = watch_rows(path)
    first = rows[0]
    if (
        first.get('event') != 'started'
        or first.get('schema') != 2
        or first['wall_s'] > capture_started
    ):
        raise ValueError('Schema2 watcher must run before scan capture')
    return dict(
        path=str(Path(path).resolve()),
        prefix_bytes=len(raw),
        prefix_sha256=hashlib.sha256(raw).hexdigest(),
        pid=first['pid'],
        process_start_ticks=first['process_start_ticks'],
        started_unix=first['wall_s'],
    )


def check_process(binding):
    pid = str(binding['pid'])
    cmd = Path('/proc', pid, 'cmdline').read_bytes()
    ticks = Path('/proc', pid, 'stat').read_text().rsplit(')', 1)[1].split()[19]
    if b'odom_reset_watch.py' not in cmd or ticks != str(binding['process_start_ticks']):
        raise ValueError('Original watcher process is no longer running')


def verify_watch(report, now=None, process_check=True):  # noqa: PLR0912, PLR0915
    """Require unbroken logged pose/identity observations from capture to attachment."""
    now = time.time() if now is None else now
    binding, epoch = report['watch'], report['epoch']
    raw, rows = watch_rows(binding['path'])
    length = binding['prefix_bytes']
    if len(raw) < length or hashlib.sha256(raw[:length]).hexdigest() != binding['prefix_sha256']:
        raise ValueError('Watcher evidence prefix changed')
    if process_check:
        check_process(binding)
    if any(row.get('event') == 'finished' for row in rows):
        raise ValueError('Watcher already finished')
    first = rows[0]
    if first.get('pid') != binding['pid'] or first.get('schema') != 2:
        raise ValueError('Watcher identity/schema mismatch')
    after = [row for row in rows if row['wall_s'] >= epoch['capture_started_unix']]
    samples = [row for row in after if row['event'] == 'sample']
    if not samples or not 0 <= now - samples[-1]['wall_s'] <= 3:
        raise ValueError('Watcher sample stale or missing')
    if samples[0]['wall_s'] - epoch['capture_started_unix'] > 3:
        raise ValueError('Watcher missing initial capture coverage')
    if samples[-1]['wall_s'] < epoch['captured_unix']:
        raise ValueError('Watcher has not yet covered completed scan capture')
    if any(b['wall_s'] - a['wall_s'] > 3 for a, b in zip(samples, samples[1:])):
        raise ValueError('Watcher observation timeline gap')
    topic = '/odometry/filtered'
    expected = pose(epoch['pose'])
    for row in after:
        event = row['event']
        if event == 'nonfinite_cmd' or (
            event in ('nonfinite_odom', 'invalid_odom') and row.get('topic') == topic
        ):
            raise ValueError('Invalid feedback observed since scan capture')
        if event in ('odom_change', 'feedback_gap') and row.get('topic') == topic:
            raise ValueError('Odom changed or was lost since scan capture')
        if event == 'command_state' and row['command']['nonzero']:
            raise ValueError('Nonzero command observed since scan capture')
        if event == 'publisher_graph' and row.get('topic') == topic:
            if [item[0] for item in row['current']] != [epoch['gid']]:
                raise ValueError('Odom publisher changed since scan capture')
        if event != 'sample':
            continue
        if [item[0] for item in row['publisher_gids'].get(topic, [])] != [epoch['gid']]:
            raise ValueError('Watcher observed different publisher identity/count')
        state = row['latest'].get(topic)
        bounds = row.get('interval_bounds', {}).get(topic)
        if not state or not bounds or row['receipt_age_s'].get(topic, 100) > 0.5:
            raise ValueError('Watcher lacks fresh interval coverage')
        if state['frame'] != 'odom' or state['child'] != 'base_link':
            raise ValueError('Watcher frame changed')
        for endpoint in ('min', 'max'):
            if not near(Pose(*bounds[endpoint]), expected, 0.002, math.radians(0.2)):
                raise ValueError('Pose moved since scan capture')
        command = row['command_idle'].get('last_command')
        if command and command['nonzero']:
            raise ValueError('Watcher observed active command')
    return samples[-1]['wall_s']


def validate_report(report, binding, now=None):
    now = time.time() if now is None else now
    if report.get('schema') != 2 or report.get('source_binding') != binding:
        raise ValueError('Calibration source hash/UUID/completed boundary mismatch')
    if (
        report.get('gates') != []
        or report.get('ambiguous') is not False
        or report.get('same_local_geometry_supported') is not True
    ):
        raise ValueError('Scan geometry not accepted or ambiguous')
    epoch = report['epoch']
    if not 0 <= now - epoch['captured_unix'] <= 120:
        raise ValueError('Scan calibration expired')
    if epoch.get('stable') is not True or epoch['frame'] != 'odom' or epoch['child'] != 'base_link':
        raise ValueError('New odometry epoch not stable')
    if type(epoch['stamp_ns']) is not int or epoch['stamp_ns'] <= 0:
        raise ValueError('Invalid new epoch stamp')
    if (
        not isinstance(epoch['gid'], str)
        or len(epoch['gid']) != 48
        or any(c not in '0123456789abcdef' for c in epoch['gid'])
    ):
        raise ValueError('Invalid new publisher GID')
    best = report['best']
    values = [best[k] for k in ('x', 'y', 'yaw', 'rmse', 'overlap')]
    if not all(type(v) in (int, float) and math.isfinite(v) for v in values):
        raise ValueError('Nonfinite scan result')
    if (
        best['rmse'] > 0.018
        or best['overlap'] < 0.8
        or math.hypot(best['x'], best['y']) > 0.02
        or abs(best['yaw']) > math.radians(1)
    ):
        raise ValueError('Scan residual/overlap/local pose outside calibration limits')
    validate_reference(report['reference'], binding)
    return pose(epoch['pose'])


def mapped_source(source, report):
    fit = report['best']
    reference_current = Pose(fit['x'], fit['y'], fit['yaw'])
    reference = pose(report['reference']['odom_pose'])
    yaw_shift = round((source.last_stop.yaw - reference.yaw) / (2 * math.pi)) * 2 * math.pi
    reference = replace(reference, yaw=reference.yaw + yaw_shift)
    old_current = compose(reference, reference_current)
    new_old = compose(pose(report['epoch']['pose']), inverse(old_current))
    actions = tuple(
        replace(a, start=compose(new_old, a.start), end=compose(new_old, a.end))
        for a in source.actions
    )
    result = replace(
        source,
        origin=compose(new_old, source.origin),
        last_stop=compose(new_old, source.last_stop),
        stamp_ns=report['epoch']['stamp_ns'],
        actions=actions,
    )
    return result, new_old


def load_calibration(path, source_path, source, now=None):
    raw = Path(path).read_bytes()
    report = json.loads(raw)
    binding = source_binding(source_path, source)
    validate_report(report, binding, now)
    verify_reference_files(report['reference'], source_path)
    verify_watch(report, now)
    mapped, new_old = mapped_source(source, report)
    evidence = dict(
        schema=1,
        source=binding,
        scan_report=str(Path(path).resolve()),
        scan_report_sha256=hashlib.sha256(raw).hexdigest(),
        transform_new_from_old=asdict(new_old),
        epoch=report['epoch'],
        original_source_origin=asdict(source.origin),
        mapped_origin=asdict(mapped.origin),
        original_last_stop=asdict(source.last_stop),
        mapped_last_stop=asdict(mapped.last_stop),
        original_stamp_ns=source.stamp_ns,
        new_stamp_ns=mapped.stamp_ns,
        policy='single_use_existing_route_no_absolute_localization_claim',
    )
    return mapped, report, evidence
