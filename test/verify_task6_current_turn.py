import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
from verify_precise_correction import Room, rclpy, ROOT
from nav_msgs.msg import Odometry

rclpy.init()
root = Path('/tmp/task6_current_turn_ros') / str(time.time_ns())
root.mkdir(parents=True)
for case in ('normal', 'scan_loss'):
    out = root / case
    out.mkdir()
    (out / 'runtime.log').touch()
    room = Room('current_turn', out)
    raw = room.node.create_publisher(Odometry, '/rosbot_xl_base_controller/odom', 10)
    room.node.create_subscription(Odometry, '/odometry/filtered', raw.publish, 10)
    assert not room.node.get_publishers_info_by_topic('/cmd_vel')
    original = room.publish_scan
    begun = time.monotonic()

    def scan(now):
        if case != 'scan_loss' or now - begun < 3:
            original(now)

    room.publish_scan = scan
    proc = None
    try:
        with (out / 'runtime.log').open('w') as log:
            proc = subprocess.Popen(
                [
                    sys.executable,
                    str(ROOT / 'tools/task6_current_turn.py'),
                    '--output',
                    str(out / 'turn.json'),
                    '--turn-speed',
                    '.20',
                    '--max-seconds',
                    '35',
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            end = time.monotonic() + 42
            while proc.poll() is None and time.monotonic() < end:
                room.tick()
        assert proc.poll() is not None, 'fixture timeout'
        for _ in range(30):
            room.tick()
        report = json.loads((out / 'turn.json').read_text())
        assert proc.returncode == (0 if case == 'normal' else 2), report
        if case == 'normal':
            assert abs(report['signed_rotation_rad'] - 3.141592653589793) < 0.01, report
        else:
            assert 'laser' in report['error'].lower(), report
        assert room.samples and room.samples[-1]['command'] == [0.0, 0.0, 0.0], room.samples[-1]
        assert not room.node.get_publishers_info_by_topic('/cmd_vel')
        print('PASS', case, out, json.dumps(report), flush=True)
    finally:
        if proc is not None and proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=5)
        room.node.destroy_node()
rclpy.shutdown()
print('EVIDENCE', root)
