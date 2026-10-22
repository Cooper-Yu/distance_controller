#!/usr/bin/env python3
"""Actual installed manual-step safety checks on isolated localhost only, synthetic sensors."""

import fcntl
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from route_prefix_return_plan import ODOM

ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT.parents[1] / 'install/distance_controller/lib/distance_controller/current_frame_step'


def write(path, value):
    path.write_text(json.dumps(value))


def rows(path):
    if not path.exists():
        return []
    return [json.loads(x) for x in path.read_text().splitlines(keepends=True) if x.endswith('\n')]


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
    state = dict(
        pose=[0.4, 0.2, 0.3],
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
        if mode in ('epoch', 'filtered_epoch') and not changed:
            index = 1 if mode == 'epoch' else 0
            node.destroy_publisher(pubs[index])
            pubs[index] = node.create_publisher(Odometry, ODOM[index], 10)
            changed = True
        velocity = state['velocity'] if now - state['received'] < 0.3 else [0.0, 0.0, 0.0]
        x, y, yaw = state['pose']
        if not changed and (
            mode in ('heading_bound', 'distance_bound')
            or (
                mode == 'pre_first_shift'
                and any(r['event'] == 'started' for r in rows(folder / 'step.jsonl'))
            )
        ):
            if mode == 'heading_bound':
                state['pose'][2] += 0.036
            else:
                state['pose'][0] += 0.006 if mode == 'pre_first_shift' else 0.08
            x, y, yaw = state['pose']
            changed = True
        if mode == 'timeout':
            velocity = [0.0, 0.0, 0.0]
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
        if mode != 'filtered_stale':
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
            if mode in ('unknown', 'dryrun', 'pre_first_shift'):
                scan.ranges[0] = float('nan')
                scan.ranges[359] = float('nan')
            if mode != 'scan_stale':
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


def wait_for(predicate, child=None, seconds=20):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if predicate():
            return
        assert child is None or child.poll() is None, 'Child exited early'
        time.sleep(0.05)
    raise AssertionError('Fixture timeout')


def verify_case(folder, mode, code, start):
    audit = rows(folder / 'step.jsonl')
    expected = 0 if mode in ('success', 'unknown', 'dryrun') else 2
    assert code == expected, (mode, code, (folder / 'runner.log').read_text()[-2000:])
    wait_for(lambda: json.loads((folder / 'sensor.json').read_text())['publishers'] == 0, seconds=5)
    info = json.loads((folder / 'sensor.json').read_text())
    commands = rows(folder / 'commands.jsonl')
    assert info['maxpub'] <= 1
    if mode == 'pre_first_shift':
        assert not any(any(r['velocity']) for r in commands)
    if mode == 'dryrun':
        assert not commands
        assert audit[-1]['event'] == 'strict_dry_run'
        assert audit[-1]['decision']['state'] == 'unknown'
    else:
        assert commands and commands[-1]['velocity'] == [0.0, 0.0, 0.0]
        assert all(
            math.hypot(*r['velocity'][:2]) <= 0.030000001 and abs(r['velocity'][2]) <= 0.100000001
            for r in commands
        )
    if mode in ('success', 'unknown'):
        assert audit[-1]['event'] == 'local_step_completed'
        points = json.loads((folder / 'step.json').read_text())['points']
        assert list(points) == ['LOCAL_START', 'LOCAL_END']
        assert 0.045 <= math.dist(start[:2], info['pose'][:2]) <= 0.055
        if mode == 'unknown':
            assert any(
                r['event'] == 'laser_guard'
                and r['decision']['state'] == 'unknown'
                and not r['enforced']
                for r in audit
            )
    elif mode != 'dryrun':
        assert audit[-1]['event'] == 'local_step_failed'
    with open('/tmp/distance_controller_survey_return.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    result = dict(
        case=mode,
        exit=code,
        lastzero=not commands or commands[-1]['velocity'] == [0.0, 0.0, 0.0],
        pub0=True,
        maxpub=info['maxpub'],
        lock_free=True,
        command_count=len(commands),
        terminal=audit[-1]['event'],
        reason=audit[-1].get('reason'),
        scope='synthetic manual step, no obstacle acceptance',
    )
    write(folder / 'verification.json', result)
    print(json.dumps(result), flush=True)


def run_case(folder, mode):  # noqa: PLR0915 - ordered isolated process/evidence lifecycle
    folder.mkdir(parents=True)
    children, handles = [], []
    try:
        if mode in ('unknown', 'dryrun', 'pre_first_shift'):
            (folder / 'mode').write_text(mode)
        handle = (folder / 'sensor.log').open('w')
        handles.append(handle)
        sensors = subprocess.Popen(
            [sys.executable, __file__, '--sensor', str(folder)],
            stdout=handle,
            stderr=subprocess.STDOUT,
        )
        children.append(sensors)
        wait_for(lambda: (folder / 'sensor.json').exists(), sensors)
        wait_for(
            lambda: all(
                len(v) == 1
                for v in json.loads((folder / 'sensor.json').read_text())['gids'].values()
            ),
            sensors,
        )
        info = json.loads((folder / 'sensor.json').read_text())
        start = info['pose']
        handle = (folder / 'runner.log').open('w')
        handles.append(handle)
        argv = [
            str(RUNNER),
            '--distance',
            '.05',
            '--speed',
            '.03',
            '--odom-publisher-gid',
            info['gids'][ODOM[0]][0],
            '--raw-odom-publisher-gid',
            info['gids'][ODOM[1]][0],
            '--output',
            str(folder / 'step.json'),
            '--strict-dry-run' if mode == 'dryrun' else '--supervised-manual',
        ]
        child = subprocess.Popen(
            argv, stdin=subprocess.DEVNULL, stdout=handle, stderr=subprocess.STDOUT
        )
        children.append(child)
        if mode not in ('dryrun', 'success', 'unknown', 'pre_first_shift'):
            wait_for(
                lambda: any(any(v for v in r['velocity']) for r in rows(folder / 'commands.jsonl')),
                child,
            )
            if mode == 'cancel':
                child.send_signal(signal.SIGINT)
            else:
                (folder / 'mode').write_text(mode)
        code = child.wait(timeout=25)
        verify_case(folder, mode, code, start)
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
        for name in sys.argv[2:] or [
            'success',
            'unknown',
            'dryrun',
            'cancel',
            'stale',
            'filtered_stale',
            'scan_stale',
            'epoch',
            'filtered_epoch',
            'timeout',
            'pre_first_shift',
            'heading_bound',
            'distance_bound',
        ]:
            run_case(Path(sys.argv[1]) / name, name)
