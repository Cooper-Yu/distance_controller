"""Admit a stopped completed path prefix from a failed run without rewriting its audit."""

import hashlib
import json
import math
from pathlib import Path
import sqlite3

from route_return_plan import ReturnSource, one, pose, validate_completed
from route_session_state import near
from task6_route_model import validate_manifest

ODOM = ('/odometry/filtered', '/rosbot_xl_base_controller/odom')
FAILURE = 'Missing/stale source timestamp: /scan_filtered'


def read_json(path):
    return json.loads(path.read_text())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_digest(path):
    return hashlib.sha256(
        path.read_bytes() + b'\n' + path.with_suffix('.jsonl').read_bytes()
    ).hexdigest()


def audit_prefix(book, rows):
    """Only accepted path boundaries; a scheduled next action is not an executed motion."""
    if book.get('schema') != 1 or book.get('frame') != 'odom' or not book.get('session_id'):
        raise ValueError('Expected original formal odom TeachingBook')
    started = one(rows, 'route_started')
    raw = [{k: v for k, v in a.items() if k != 'points' or v} for a in started['actions']]
    actions = validate_manifest(
        dict(schema=1, scene=started['scene'], frame=started['frame'], actions=raw)
    )
    n = len(book['points']) - 1
    if not 1 <= n < len(actions) or any(a.kind != 'path' for a in actions[:n]):
        raise ValueError('Only a nonempty completed path-only prefix is supported')
    if set(book['points']) != {'P01', *(a.name for a in actions[:n])}:
        raise ValueError('Prefix point names are not contiguous')
    p01 = one([r for r in rows if r.get('name') == 'P01'], 'record')
    if p01['point'] != book['points']['P01'] or pose(p01['point']['pose']) != pose(book['origin']):
        raise ValueError('Original P01 binding mismatch')
    boundaries = [i for i, row in enumerate(rows) if row.get('event') == 'route_action_started']
    if len(boundaries) not in (n, n + 1):
        raise ValueError('Unexpected action boundary count')
    completed, previous = [], pose(book['origin'])
    for index in range(n):
        start = boundaries[index]
        end = boundaries[index + 1] if index + 1 < len(boundaries) else len(rows)
        block = rows[start:end]
        if block[0].get('index') != index or block[0].get('name') != actions[index].name:
            raise ValueError('Noncontiguous prefix action order')
        completed.append(validate_completed(actions[index], block, book, previous))
        previous = completed[-1].end
    last_record = one([r for r in rows if r.get('name') == actions[n - 1].name], 'record')
    tail = rows[rows.index(last_record) + 1 :]
    if (
        not tail
        or tail[-1].get('event') != 'route_failed'
        or one(rows, 'route_failed').get('exit_code') != 2
    ):
        raise ValueError('Expected original failed boundary, never a forged successful run')
    if any(r.get('event') not in ('route_action_started', 'route_failed') for r in tail):
        raise ValueError('Motion/evidence after completed prefix cannot be resumed')
    if len(boundaries) == n + 1:
        next_row = rows[boundaries[-1]]
        if next_row.get('index') != n or next_row.get('name') != actions[n].name:
            raise ValueError('Unexpected scheduled next action')
    if len([r for r in rows if r.get('event') == 'started']) != n:
        raise ValueError('Partial next motion exists')
    verify_point_stamps(book, actions[:n])
    return tuple(completed), last_record['time'], p01['time']


def verify_point_stamps(book, actions):
    stamps = [book['points'][name]['odom_stamp_ns'] for name in ['P01', *(a.name for a in actions)]]
    if any(type(v) is not int or v <= 0 for v in stamps) or any(
        b <= a for a, b in zip(stamps, stamps[1:])
    ):
        raise ValueError('Invalid original point chronology')


def stopped(values):
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError('Nonfinite velocity')
    return math.hypot(*values[:2]) < 0.003 and abs(values[2]) < 0.01


def verify_snapshot(snapshot, gids, last_stop):
    if snapshot.get('error') != FAILURE or snapshot.get('phase') != 'official_runner':
        raise ValueError('Only scan freshness failure at a completed boundary is supported')
    probe = snapshot['probe']
    if probe['cmd_endpoints']:
        raise ValueError('Boundary snapshot still has a controller publisher')
    for topic in ODOM:
        entry = probe['topics'][topic]
        if [p['gid'] for p in entry['publishers']] != gids[topic]:
            raise ValueError('Boundary odometry identity changed')
        sample = entry['sample']
        if (sample['frame'], sample['child']) != ('odom', 'base_link') or not stopped(
            sample['velocity']
        ):
            raise ValueError('Boundary odometry was not stopped in the original frame')
        if not 0 <= entry['receipt_age_s'] <= 0.5 or not -0.1 <= entry['source_age_s'] <= 0.5:
            raise ValueError('Boundary odometry stale')
    actual = pose(dict(zip(('x', 'y', 'yaw'), probe['topics'][ODOM[0]]['sample']['pose'])))
    if not near(actual, last_stop, 0.005, 0.01):
        raise ValueError('Failure snapshot moved from last completed stop')


def verify_watch(rows, gids, start, finish):
    graph = {}
    for row in rows:
        if row.get('wall_s', 0) > finish + 3:
            break
        topic = row.get('topic')
        if row.get('event') == 'publisher_graph' and topic in ODOM:
            current = [v[0] for v in row['current']]
            if row['wall_s'] >= start and current != gids[topic]:
                raise ValueError('Source watcher odometry identity changed')
            graph[topic] = current
        if row.get('wall_s', 0) >= start and row.get('event') in (
            'odom_change',
            'feedback_gap',
            'nonfinite_odom',
            'invalid_odom',
            'nonfinite_cmd',
        ):
            raise ValueError('Source watcher reported continuity failure')
    if graph != gids:
        raise ValueError('Missing source dual-odometry watcher identities')
    samples = [r for r in rows if r.get('event') == 'sample']
    if not samples or samples[0]['wall_s'] > start or samples[-1]['wall_s'] < finish:
        raise ValueError('Watcher does not cover source prefix boundary')


def closed_recording(directory):
    latest = Path((directory / 'evidence/latest_session.txt').read_text().strip())
    recording = directory / 'evidence' / latest.name
    if not recording.is_dir() or not recording.resolve().is_relative_to(
        (directory / 'evidence').resolve()
    ):
        raise ValueError('Recording is outside original source run')
    result = read_json(recording / 'result.json')
    if (
        result.get('closed_metadata') is not True
        or result.get('exception')
        or any((recording / 'bag').glob('*-wal'))
    ):
        raise ValueError('Original recorder did not close successfully')
    if not (recording / 'bag/metadata.yaml').is_file():
        raise ValueError('Closed recording metadata unavailable')
    return recording


def odom_record(receipt, message):
    p, q, v = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist
    if not all(math.isfinite(x) for x in (q.x, q.y, q.z, q.w)) or (
        abs(math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) - 1) > 0.01
    ):
        raise ValueError('Invalid bag quaternion')
    return dict(
        receipt=receipt,
        stamp_ns=message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec,
        pose=dict(
            x=p.x,
            y=p.y,
            yaw=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
        ),
        frames=(message.header.frame_id, message.child_frame_id),
        velocity=(v.linear.x, v.linear.y, v.angular.z),
    )


def read_boundary_bag(bag):
    """Read message bytes only; no rclpy initialization or publishers."""
    import yaml
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message

    paths = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information'][
        'relative_file_paths'
    ]
    streams, commands = {t: [] for t in ODOM}, []
    for name in paths:
        path = (bag / name).resolve(strict=True)
        if not path.is_relative_to(bag.resolve()) or path.suffix != '.db3':
            raise ValueError('Bag file outside source recording')
        with sqlite3.connect(path.as_uri() + '?mode=ro', uri=True) as db:
            topics = dict(db.execute('select id,name from topics'))
            for topic, ns, blob in db.execute(
                'select topic_id,timestamp,data from messages order by timestamp,id'
            ):
                topic = topics[topic]
                if topic in ODOM:
                    streams[topic].append(
                        odom_record(ns / 1e9, deserialize_message(blob, Odometry))
                    )
                elif topic == '/cmd_vel':
                    v = deserialize_message(blob, Twist)
                    commands.append((ns / 1e9, (v.linear.x, v.linear.y, v.angular.z)))
    return {t: sorted(v, key=lambda r: r['receipt']) for t, v in streams.items()}, sorted(commands)


def verify_bag(streams, commands, boundary, last_stop, snapshot):
    if not commands or any(not math.isfinite(v) for _, cmd in commands for v in cmd):
        raise ValueError('Missing/nonfinite command evidence')
    if any(abs(v) > 0 for v in commands[-1][1]) or any(
        t >= boundary and any(abs(v) > 0 for v in cmd) for t, cmd in commands
    ):
        raise ValueError('Nonzero command after completed prefix boundary')
    last = {}
    for topic in ODOM:
        rows = streams[topic]
        if not rows or rows[-1]['receipt'] < snapshot['unix_time']:
            raise ValueError('Closed bag does not cover cancellation')
        tail = [r for r in rows if r['receipt'] >= rows[-1]['receipt'] - 0.6]
        if len(tail) < 3 or tail[-1]['receipt'] - tail[0]['receipt'] < 0.5:
            raise ValueError('Insufficient final stopped odometry')
        if any(
            r['frames'] != ('odom', 'base_link')
            or not stopped(r['velocity'])
            or not near(pose(r['pose']), pose(tail[-1]['pose']), 0.003, 0.005)
            for r in tail
        ):
            raise ValueError('Final recorded odometry not stationary')
        if any(
            b['stamp_ns'] <= a['stamp_ns'] or b['stamp_ns'] - a['stamp_ns'] > 500_000_000
            for a, b in zip(rows, rows[1:])
        ):
            raise ValueError('Bag odometry chronology/continuity failure')
        last[topic] = rows[-1]
    if not near(pose(last[ODOM[0]]['pose']), last_stop, 0.005, 0.01):
        raise ValueError('Final bag pose moved from completed stop')
    raw_snapshot = pose(
        dict(zip(('x', 'y', 'yaw'), snapshot['probe']['topics'][ODOM[1]]['sample']['pose']))
    )
    if not near(pose(last[ODOM[1]]['pose']), raw_snapshot, 0.005, 0.01):
        raise ValueError('Final raw odometry moved from boundary snapshot')
    return last


def load_prefix_run(directory):
    directory = Path(directory).resolve(strict=True)
    paths = {
        name: directory / name
        for name in (
            'points.json',
            'points.jsonl',
            'result.json',
            'preflight.json',
            'run.started.json',
            'failure_snapshot.json',
            'odom_watch.jsonl',
            'config.json',
        )
    }
    before = {str(p): digest(p) for p in paths.values()}
    book = read_json(paths['points.json'])
    rows = [json.loads(line) for line in paths['points.jsonl'].read_text().splitlines()]
    actions, boundary, start = audit_prefix(book, rows)
    result, snapshot = read_json(paths['result.json']), read_json(paths['failure_snapshot.json'])
    if (
        result.get('success') is not False
        or result.get('error') != FAILURE
        or result['cleanup'].get('zero_and_released') is not True
        or result['cleanup']['evidence'].get('closed_metadata') is not True
    ):
        raise ValueError('Require original failed run with stopped/released closed cleanup')
    gids = read_json(paths['run.started.json'])['gids']
    if (
        gids != read_json(paths['preflight.json'])['gids']
        or set(gids) != set(ODOM)
        or any(len(v) != 1 for v in gids.values())
    ):
        raise ValueError('Original dual odometry epoch unavailable')
    verify_snapshot(snapshot, gids, actions[-1].end)
    watch = [json.loads(line) for line in paths['odom_watch.jsonl'].read_text().splitlines()]
    verify_watch(watch, gids, start, snapshot['unix_time'])
    recording = closed_recording(directory)
    for p in (
        recording / 'result.json',
        recording / 'bag/metadata.yaml',
        *sorted((recording / 'bag').glob('*.db3')),
    ):
        before[str(p)] = digest(p)
    final = verify_bag(*read_boundary_bag(recording / 'bag'), boundary, actions[-1].end, snapshot)
    if any(digest(Path(p)) != sha for p, sha in before.items()):
        raise ValueError('Source changed during prefix validation')
    source = ReturnSource(
        book['session_id'],
        pose(book['origin']),
        actions[-1].end,
        book['points'][actions[-1].name]['odom_stamp_ns'],
        source_digest(paths['points.json']),
        actions,
    )
    metadata = dict(
        mode='completed_stopped_prefix_recovery',
        source_run_dir=str(directory),
        source_session=source.session_id,
        source_hash=source.source_hash,
        prefix_names=[a.name for a in actions],
        gids=gids,
        recording=str(recording),
        binding_files=before,
        raw_last_stop=final[ODOM[1]]['pose'],
        raw_stamp_ns=final[ODOM[1]]['stamp_ns'],
        boundary_time=boundary,
        final_filtered=final[ODOM[0]],
        original_failure=FAILURE,
        task6_acceptance=False,
    )
    return source, metadata
