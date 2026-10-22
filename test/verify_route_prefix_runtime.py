#!/usr/bin/env python3
"""Bounded localhost actual runner cases; synthetic provenance is not real return acceptance."""

import fcntl
import json
import math
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import time

from test_route_prefix_return import fixture

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from route_prefix_return_plan import ODOM, FAILURE, load_prefix_run, digest


ROOT = Path(__file__).resolve().parents[1]
RUNNER = (
    ROOT.parents[1]
    / 'install/distance_controller/lib/distance_controller/route_prefix_return_runner'
)


def write(path, value):
    path.write_text(json.dumps(value))


def rows(path):
    return [json.loads(x) for x in path.read_text().splitlines()] if path.exists() else []


def sensor(folder):  # noqa: PLR0915 - bounded test node lifecycle
    import rclpy
    from geometry_msgs.msg import Twist, TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import QoSProfile, DurabilityPolicy

    rclpy.init()
    node = rclpy.create_node('prefix_fixture')
    pubs = [node.create_publisher(Odometry, t, 10) for t in ODOM]
    scan_pub = node.create_publisher(LaserScan, '/scan_filtered', 10)
    static = node.create_publisher(
        TFMessage, '/tf_static', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    )
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = 'base_link', 'laser'
    tf.transform.rotation.w = 1.0
    static.publish(TFMessage(transforms=[tf]))
    book, _ = fixture()
    state = dict(
        pose=list(book['points']['P03']['pose'].values()),
        velocity=[0.0, 0.0, 0.0],
        received=0.0,
        maxpub=0,
    )
    log = (folder / 'commands.jsonl').open('w', buffering=1)

    def command(msg):
        state['velocity'] = [msg.linear.x, msg.linear.y, msg.angular.z]
        state['received'] = time.monotonic()
        log.write(json.dumps(dict(time=time.time(), velocity=state['velocity'])) + '\n')

    node.create_subscription(Twist, '/cmd_vel', command, 10)
    previous, count, changed = time.monotonic(), 0, False

    def tick():
        nonlocal previous, count, changed
        now = time.monotonic()
        dt, previous = min(now - previous, 0.05), now
        mode = (folder / 'mode').read_text().strip() if (folder / 'mode').exists() else 'normal'
        if mode == 'epoch' and not changed:
            node.destroy_publisher(pubs[1])
            pubs[1] = node.create_publisher(Odometry, ODOM[1], 10)
            changed = True
        velocity = state['velocity'] if now - state['received'] < 0.3 else [0.0, 0.0, 0.0]
        x, y, yaw = state['pose']
        vx, vy, wz = velocity
        state['pose'] = [
            x + (math.cos(yaw) * vx - math.sin(yaw) * vy) * dt,
            y + (math.sin(yaw) * vx + math.cos(yaw) * vy) * dt,
            yaw + wz * dt,
        ]
        msg = Odometry()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y = state['pose'][:2]
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = (
            math.sin(state['pose'][2] / 2),
            math.cos(state['pose'][2] / 2),
        )
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z = velocity
        pubs[0].publish(msg)
        if mode != 'stale':
            pubs[1].publish(msg)
        if count % 4 == 0:
            scan = LaserScan(
                angle_min=-math.pi,
                angle_max=math.pi,
                angle_increment=math.pi / 360,
                range_min=0.1,
                range_max=10.0,
                ranges=[1.0] * 720,
            )
            scan.header.stamp, scan.header.frame_id = msg.header.stamp, 'laser'
            scan_pub.publish(scan)
            count_pub = node.count_publishers('/cmd_vel')
            state['maxpub'] = max(state['maxpub'], count_pub)
            write(
                folder / 'sensor.tmp',
                dict(
                    time=time.time(),
                    pose=state['pose'],
                    velocity=velocity,
                    publishers=count_pub,
                    maxpub=state['maxpub'],
                    gids={
                        t: [
                            bytes(i.endpoint_gid).hex()
                            for i in node.get_publishers_info_by_topic(t)
                        ]
                        for t in ODOM
                    },
                ),
            )
            (folder / 'sensor.tmp').replace(folder / 'sensor.json')
        count += 1

    node.create_timer(0.02, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def make_source(directory, info):  # noqa: PLR0915 - fixture artifact assembly
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message
    import yaml

    directory.mkdir()
    book, audit = fixture()
    now = time.time() - 2
    for row in audit:
        if 'time' in row:
            row['time'] += now - 3
    for i, name in enumerate(book['points']):
        book['points'][name]['odom_stamp_ns'] = int((now - 3 + i) * 1e9)
    for row in audit:
        if row['event'] == 'record':
            row['point'] = book['points'][row['name']]
    write(directory / 'points.json', book)
    (directory / 'points.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in audit))
    for name in ('preflight.json', 'run.started.json'):
        write(directory / name, dict(gids=info['gids']))
    snapshot = dict(
        unix_time=now + 0.1,
        error=FAILURE,
        phase='official_runner',
        probe=dict(
            cmd_endpoints=[],
            topics={
                t: dict(
                    publishers=[dict(gid=info['gids'][t][0])],
                    source_age_s=0.01,
                    receipt_age_s=0.01,
                    sample=dict(
                        frame='odom', child='base_link', velocity=[0, 0, 0], pose=info['pose']
                    ),
                )
                for t in ODOM
            },
        ),
    )
    write(directory / 'failure_snapshot.json', snapshot)
    write(
        directory / 'result.json',
        dict(
            success=False,
            error=FAILURE,
            cleanup=dict(zero_and_released=True, evidence=dict(closed_metadata=True)),
        ),
    )
    write(directory / 'config.json', dict(scope='synthetic source for isolated runtime'))
    watch = [
        dict(
            event='publisher_graph',
            topic=t,
            current=[[info['gids'][t][0], '', 'fixture']],
            wall_s=now - 4,
        )
        for t in ODOM
    ]
    watch += [dict(event='sample', wall_s=t) for t in (now - 4, now - 2, now, now + 1)]
    (directory / 'odom_watch.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in watch))
    recording = directory / 'evidence' / 'fixture'
    bag = recording / 'bag'
    bag.mkdir(parents=True)
    (directory / 'evidence/latest_session.txt').write_text(str(recording))
    write(recording / 'result.json', dict(closed_metadata=True))
    (bag / 'metadata.yaml').write_text(
        yaml.safe_dump(dict(rosbag2_bagfile_information=dict(relative_file_paths=['bag_0.db3'])))
    )
    with sqlite3.connect(bag / 'bag_0.db3') as db:
        db.execute('create table topics(id integer,name text)')
        db.execute(
            'create table messages(id integer primary key,topic_id integer,timestamp integer,data blob)'
        )
        db.executemany(
            'insert into topics values(?,?)', [(1, ODOM[0]), (2, ODOM[1]), (3, '/cmd_vel')]
        )
        for i in range(9):
            stamp = int((now - 0.1 + i * 0.1) * 1e9)
            msg = Odometry()
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
            msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
            msg.pose.pose.position.x, msg.pose.pose.position.y = info['pose'][:2]
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = (
                math.sin(info['pose'][2] / 2),
                math.cos(info['pose'][2] / 2),
            )
            for topic in (1, 2):
                db.execute(
                    'insert into messages(topic_id,timestamp,data) values(?,?,?)',
                    (topic, stamp, serialize_message(msg)),
                )
        db.execute(
            'insert into messages(topic_id,timestamp,data) values(?,?,?)',
            (3, int((now - 0.2) * 1e9), serialize_message(Twist())),
        )
    return load_prefix_run(directory)


def wait_for(predicate, child=None, seconds=20):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if predicate():
            return
        assert child is None or child.poll() is None, 'Child exited early'
        time.sleep(0.05)
    raise AssertionError('Fixture timeout')


def run_case(folder, mode):  # noqa: PLR0915 - ordered process/evidence lifecycle
    folder.mkdir(parents=True)
    children = []
    handles = []
    try:
        handle = (folder / 'sensor.log').open('w')
        handles.append(handle)
        sensor_child = subprocess.Popen(
            [sys.executable, __file__, '--sensor', str(folder)],
            stdout=handle,
            stderr=subprocess.STDOUT,
        )
        children.append(sensor_child)
        wait_for(lambda: (folder / 'sensor.json').exists(), sensor_child)
        info = json.loads((folder / 'sensor.json').read_text())
        wait_for(
            lambda: all(
                len(x) == 1
                for x in json.loads((folder / 'sensor.json').read_text())['gids'].values()
            ),
            sensor_child,
        )
        info = json.loads((folder / 'sensor.json').read_text())
        source, metadata = make_source(folder / 'source', info)
        original = dict(metadata['binding_files'])
        handle = (folder / 'runner.log').open('w')
        handles.append(handle)
        argv = [
            str(RUNNER),
            '--source-run-dir',
            str(folder / 'source'),
            '--confirm-same-odom',
            source.session_id,
            '--odom-publisher-gid',
            info['gids'][ODOM[0]][0],
            '--raw-odom-publisher-gid',
            info['gids'][ODOM[1]][0],
            '--output',
            str(folder / 'return.json'),
            '--laser-log-only',
        ]
        child = subprocess.Popen(
            argv, stdin=subprocess.PIPE, stdout=handle, stderr=subprocess.STDOUT, text=True
        )
        children.append(child)
        audit = folder / 'return.jsonl'
        wait_for(lambda: any(r['event'] == 'return_session_started' for r in rows(audit)), child)
        assert not any(any(v for v in r['velocity']) for r in rows(folder / 'commands.jsonl'))
        child.stdin.write('next\n')
        child.stdin.flush()
        wait_for(
            lambda: any(any(v for v in r['velocity']) for r in rows(folder / 'commands.jsonl')),
            child,
        )
        if mode == 'success':
            wait_for(
                lambda: (
                    len([r for r in rows(audit) if r['event'] == 'return_action_completed']) == 1
                ),
                child,
            )
            time.sleep(0.2)
            assert len([r for r in rows(audit) if r['event'] == 'return_action_started']) == 1
            child.stdin.write('next\n')
            child.stdin.flush()
        elif mode == 'cancel':
            child.send_signal(signal.SIGINT)
        else:
            (folder / 'mode').write_text(mode)
        code = child.wait(timeout=25)
        assert code == (0 if mode == 'success' else 2), (mode, code)
        wait_for(
            lambda: json.loads((folder / 'sensor.json').read_text())['publishers'] == 0, seconds=5
        )
        info = json.loads((folder / 'sensor.json').read_text())
        commands = rows(folder / 'commands.jsonl')
        assert commands[-1]['velocity'] == [0.0, 0.0, 0.0]
        assert info['maxpub'] <= 1
        assert all(digest(Path(p)) == sha for p, sha in original.items())
        assert (folder / 'source/points.return_claim.json').exists()
        retry = subprocess.run(argv, capture_output=True, text=True, timeout=10)
        assert retry.returncode == 2 and 'already has a return attempt' in retry.stdout
        with open('/tmp/distance_controller_survey_return.lock', 'a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if mode == 'success':
            assert len([r for r in rows(audit) if r['event'] == 'return_action_completed']) == 2
            assert math.dist(info['pose'][:2], [0, 0]) < 0.005
        result = dict(
            case=mode,
            exit=code,
            lastzero=True,
            pub0=True,
            maxpub=info['maxpub'],
            original_unchanged=True,
            claim_restart_refused=True,
            lock_free=True,
            completed=len([r for r in rows(audit) if r['event'] == 'return_action_completed']),
        )
        write(folder / 'verification.json', result)
        print(json.dumps(result), flush=True)
    finally:
        for child in reversed(children):
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
        for handle in handles:
            handle.close()


if __name__ == '__main__':
    if sys.argv[1] == '--sensor':
        sensor(Path(sys.argv[2]))
    else:
        folder = Path(sys.argv[1])
        for name in sys.argv[2:] or ['success', 'cancel', 'stale', 'epoch']:
            run_case(folder / name, name)
