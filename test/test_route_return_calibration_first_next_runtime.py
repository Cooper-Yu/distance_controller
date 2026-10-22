"""Actual ROS calibrated first action; synthetic geometry is not physical recovery evidence."""

import importlib.util
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time

from test_route_return import fixture
from test_route_return_calibration import reference_fixture

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from route_return_calibration import mapped_source, source_binding, verify_watch, watch_binding
from route_return_plan import load_source
from route_session_state import near

ROOT = Path(__file__).resolve().parents[1]


def sensor_fixture(folder):  # noqa: PLR0915 - one bounded ROS sensor lifecycle
    """Integrate real /cmd_vel messages, publish current odom, and retain command evidence."""
    import rclpy
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = rclpy.create_node('calibrated_return_dynamic_fixture')
    odom_pubs = [
        node.create_publisher(Odometry, topic, 10)
        for topic in ('/odometry/filtered', '/rosbot_xl_base_controller/odom')
    ]
    scan_pub = node.create_publisher(LaserScan, '/scan_filtered', 10)
    static = node.create_publisher(
        TFMessage, '/tf_static', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    )
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'base_link', 'laser'
    transform.transform.rotation.w = 1.0
    static.publish(TFMessage(transforms=[transform]))
    state = dict(pose=[0.12, -0.08, 0.4], velocity=[0.0, 0.0, 0.0], received=0.0)
    commands = (folder / 'commands.jsonl').open('w', buffering=1)

    def command(msg):
        state['velocity'] = [msg.linear.x, msg.linear.y, msg.angular.z]
        state['received'] = time.monotonic()
        commands.write(json.dumps(dict(time=time.time(), velocity=state['velocity'])) + '\n')

    node.create_subscription(Twist, '/cmd_vel', command, 10)
    previous, count = time.monotonic(), 0

    def tick():
        nonlocal previous, count
        now = time.monotonic()
        dt, previous = min(now - previous, 0.05), now
        velocity = state['velocity'] if now - state['received'] < 0.3 else [0.0, 0.0, 0.0]
        x, y, yaw = state['pose']
        vx, vy, wz = velocity
        state['pose'] = [
            x + (math.cos(yaw) * vx - math.sin(yaw) * vy) * dt,
            y + (math.sin(yaw) * vx + math.cos(yaw) * vy) * dt,
            yaw + wz * dt,
        ]
        odom = Odometry()
        odom.header.stamp = node.get_clock().now().to_msg()
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = state['pose'][:2]
        odom.pose.pose.orientation.z = math.sin(state['pose'][2] / 2)
        odom.pose.pose.orientation.w = math.cos(state['pose'][2] / 2)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = vx, vy
        odom.twist.twist.angular.z = wz
        for pub in odom_pubs:
            pub.publish(odom)
        if count % 4 == 0:
            scan = LaserScan(
                angle_min=-math.pi,
                angle_max=math.pi,
                angle_increment=2 * math.pi / 720,
                range_min=0.1,
                range_max=10.0,
                ranges=[1.0] * 720,
            )
            scan.header.stamp, scan.header.frame_id = odom.header.stamp, 'laser'
            scan_pub.publish(scan)
            temporary = folder / 'sensor.tmp'
            temporary.write_text(
                json.dumps(
                    dict(
                        time=time.time(),
                        pose=state['pose'],
                        velocity=velocity,
                        publishers=node.count_publishers('/cmd_vel'),
                    )
                )
            )
            temporary.replace(folder / 'sensor.json')
        count += 1

    node.create_timer(0.02, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        commands.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def rows(path):
    if not path.exists():
        return []
    lines = path.read_text().splitlines()
    return [json.loads(line) for line in lines if line.strip()]


def wait_event(path, name, child, seconds=20):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        found = [row for row in rows(path) if row.get('event') == name]
        if found:
            return found[-1]
        assert child.poll() is None, f'Runner exited before {name}'
        time.sleep(0.05)
    raise AssertionError(f'Timed out waiting for {name}')


def report_for(folder, epoch, watch):
    book, audit = fixture()
    audit[-1]['time'] = time.time() - 30
    path = folder / 'points.json'
    path.write_text(json.dumps(book))
    path.with_suffix('.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in audit))
    original = load_source(path)
    binding = source_binding(path, original)
    report = dict(
        schema=2,
        source_binding=binding,
        gates=[],
        ambiguous=False,
        same_local_geometry_supported=True,
        best=dict(x=0.0, y=0.0, yaw=0.0, rmse=0.001, overlap=1.0),
        reference=reference_fixture(binding, folder),
        epoch=epoch,
        watch=watch_binding(watch, epoch['capture_started_unix']),
    )
    report_path = folder / 'synthetic_geometry_report.json'
    report_path.write_text(json.dumps(report))
    return path, original, report_path, report


def test_actual_calibrated_one_next_then_quit(tmp_path, monkeypatch):  # noqa: PLR0915 - ordered evidence lifecycle
    monkeypatch.setenv('ROS_DOMAIN_ID', '224')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    ops = ROOT / 'tools' / 'task6_ops'
    spec = importlib.util.spec_from_file_location(
        'scan_compare_first_next', ops / 'scan_compare.py'
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    children, handles = [], []
    watch = tmp_path / 'watch.jsonl'
    try:
        for name, argv in (
            ('sensors', [__file__, '--sensors', str(tmp_path)]),
            (
                'watch',
                [
                    str(ops / 'odom_reset_watch.py'),
                    '--output',
                    str(watch),
                    '--seconds',
                    '90',
                    '--base-topic',
                    '/rosbot_xl_base_controller/odom',
                ],
            ),
        ):
            handle = (tmp_path / f'{name}.log').open('w')
            handles.append(handle)
            children.append(
                subprocess.Popen(
                    [sys.executable, *argv],
                    stdin=subprocess.DEVNULL,
                    stdout=handle,
                    stderr=subprocess.STDOUT,
                )
            )
        time.sleep(1)
        *_, epoch = module.current_capture()
        path, original, report_path, report = report_for(tmp_path, epoch, watch)
        time.sleep(2.1)
        verify_watch(report)
        before = path.read_bytes(), path.with_suffix('.jsonl').read_bytes()
        mapped, _ = mapped_source(original, report)
        handle = (tmp_path / 'runner.log').open('w')
        handles.append(handle)
        runner = subprocess.Popen(
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
                epoch['gid'],
                '--output',
                str(tmp_path / 'return.json'),
                '--laser-log-only',
                '--speed',
                '0.03',
                '--turn-speed',
                '0.10',
            ],
            stdin=subprocess.PIPE,
            stdout=handle,
            stderr=subprocess.STDOUT,
            text=True,
        )
        children.append(runner)
        audit = tmp_path / 'return.jsonl'
        wait_event(audit, 'return_session_started', runner)
        assert not any(
            any(abs(v) > 0 for v in row['velocity']) for row in rows(tmp_path / 'commands.jsonl')
        )
        sent = time.time()
        runner.stdin.write('next\n')
        runner.stdin.flush()
        completed = wait_event(audit, 'return_action_completed', runner)
        assert completed['cursor'] == 2
        assert near(Pose(**completed['actual']), mapped.actions[-1].start, 0.005, math.radians(1))
        time.sleep(0.3)
        assert len([r for r in rows(audit) if r['event'] == 'return_action_started']) == 1
        runner.stdin.write('quit\n')
        runner.stdin.flush()
        assert runner.wait(timeout=10) == 0
        wait_released(tmp_path)
        evidence = rows(audit)
        assert evidence[-1]['event'] == 'return_closed' and evidence[-1]['cursor'] == 2
        assert not evidence[-1]['complete']
        assert path.with_suffix('.return_claim.json').exists()
        assert before == (path.read_bytes(), path.with_suffix('.jsonl').read_bytes())
        commands = rows(tmp_path / 'commands.jsonl')
        nonzero = [row for row in commands if any(abs(v) > 0 for v in row['velocity'])]
        assert nonzero and nonzero[0]['time'] >= sent
        assert commands[-1]['velocity'] == [0.0, 0.0, 0.0]
        (tmp_path / 'verification.json').write_text(
            json.dumps(
                dict(
                    scope='isolated ROS; synthetic reference geometry; one reverse action only',
                    cursor=2,
                    exit_code=0,
                    first_nonzero_after_next=True,
                    final_zero=True,
                    publisher_count=0,
                    source_unchanged=True,
                    actual=completed['actual'],
                    expected=vars(mapped.actions[-1].start),
                ),
                indent=2,
            )
        )
        print(f'FIRST_NEXT_EVIDENCE={tmp_path}')
    finally:
        for child in reversed(children):
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=5)
        for handle in handles:
            handle.close()


def wait_released(folder):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        state = json.loads((folder / 'sensor.json').read_text())
        if state['publishers'] == 0 and state['velocity'] == [0.0, 0.0, 0.0]:
            return
        time.sleep(0.1)
    raise AssertionError('Runner did not release publisher with stopped feedback')


if __name__ == '__main__':
    assert sys.argv[1] == '--sensors'
    sensor_fixture(Path(sys.argv[2]))
