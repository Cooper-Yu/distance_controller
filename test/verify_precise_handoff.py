"""Actual C++ /step handoff gates after laser verification; isolated localhost room."""

import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

from verify_precise_correction import Room, rclpy
from distance_controller.srv import ExecuteStep


class HandoffRoom(Room):
    def __init__(self, mode, output):
        super().__init__('handoff_' + mode, output)
        self.case = mode
        self.waiting_bias = 0.0
        self.client = self.node.create_client(ExecuteStep, '/distance_controller/step')

    def distances(self, now):
        right = 0.261149
        if self.case == 'unstable' and self.verify_start is not None:
            right += 0.008 if int(now * 14) % 2 else -0.008
        return right, 0.284

    def publish_odom(self, now):
        original = self.pose[2]
        # Simulate odom heading drift while physical wall alignment stays unchanged.
        bias = 0.012 if self.case != 'nonprecise' and self.verify_start is not None else 0.0
        self.pose[2] += bias + self.waiting_bias
        super().publish_odom(now)
        self.pose[2] = original

    def publish_scan(self, now):
        if self.case == 'stale' and self.verify_start is not None:
            return
        super().publish_scan(now)

    def until(self, predicate, seconds=20):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.tick()
            if predicate():
                return
        raise AssertionError(
            ('deadline', self.case, (self.output / 'runtime.log').read_text()[-3000:])
        )

    def call(self, action):
        self.until(self.client.service_is_ready, 5)
        request = ExecuteStep.Request()
        request.action = action
        future = self.client.call_async(request)
        self.until(future.done, 5)
        result = future.result()
        return {'accepted': result.accepted, 'message': result.message}


def exercise_handoff(room, mode, output, result, old_failure):
    if mode in ('unstable', 'stale'):
        room.until(lambda: room.verify_start is not None)
        deadline = time.monotonic() + (3.0 if mode == 'unstable' else 0.3)
        while time.monotonic() < deadline:
            room.tick()
        result['without_verified'] = room.call('finish')
        assert not result['without_verified']['accepted'], result
        text = (output / 'runtime.log').read_text()
        assert 'START_VERIFIED:' not in text and 'START_ORIGIN_HEADING_CAPTURE:' not in text
        room.call('cancel')
    else:
        room.until(lambda: 'Centered A recorded=' in (output / 'runtime.log').read_text())
        result['verified_status'] = room.call('status')
        status = result['verified_status']['message']
        observed, reference = re.search(
            r'pose=\([^,]+,[^,]+,([^\)]+)\) heading_reference=([^ ]+)', status
        ).groups()
        result['observed_yaw'], result['reference_yaw'] = float(observed), float(reference)
        if mode != 'nonprecise':
            assert abs(float(observed)) > 0.008726646, result
            if not old_failure:
                assert abs(float(observed) - float(reference)) < 0.00001, result
        if mode in ('waiting_drift', 'nonprecise'):
            room.waiting_bias = 0.02
            for _ in range(4):
                room.tick()
            result['after_waiting_drift'] = room.call('finish')
            assert not result['after_waiting_drift']['accepted'], result
            assert 'aligned WAITING' in result['after_waiting_drift']['message']
            room.call('cancel')
        else:
            result['verified_finish'] = room.call('finish')
            assert result['verified_finish']['accepted'] != old_failure, result
            if old_failure:
                assert abs(float(observed) - float(reference)) > 0.008726646, result
                room.call('cancel')


def run(mode, old_failure=False):
    output = Path('/tmp/precise_handoff_ros') / f'{time.time_ns()}_{mode}'
    output.mkdir(parents=True)
    rclpy.init()
    room = HandoffRoom(mode, output)
    command = [
        'ros2',
        'run',
        'distance_controller',
        'distance_controller',
        '2',
        '--ros-args',
        '-p',
        'manual_mode:=true',
        '-p',
        'start_paused:=true',
        '-p',
        'alignment_wall:=right',
        '-p',
        'robust_wall_heading:=true',
        '-p',
        'positioning_wall:=right',
        '-p',
        'right_target_distance:=0.26',
        '-p',
        'precise_start:=' + ('false' if mode == 'nonprecise' else 'true'),
    ]
    process = None
    result = {'mode': mode, 'old_failure_expected': old_failure}
    try:
        with (output / 'runtime.log').open('w') as log:
            process = subprocess.Popen(
                command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
            )
            result['before_verified'] = room.call('finish')
            assert not result['before_verified']['accepted'], result
            exercise_handoff(room, mode, output, result, old_failure)
            if result.get('verified_finish', {}).get('accepted'):
                room.until(lambda: process.poll() is not None, 5)
                assert process.returncode == 0, process.returncode
            elif process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=8)
            for _ in range(50):
                room.tick()
            assert room.samples and sum(abs(v) for v in room.velocity) < 1e-9
            assert not room.node.get_publishers_info_by_topic('/cmd_vel')
            result.update(last_zero=True, publishers=0, exit_code=process.returncode)
            (output / 'result.json').write_text(json.dumps(result, indent=2))
            (output / 'commands.json').write_text(json.dumps(room.samples))
            print('PASS', mode, output, json.dumps(result), flush=True)
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=8)
        room.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    run(sys.argv[1] if len(sys.argv) > 1 else 'verified', '--expect-old-failure' in sys.argv)
