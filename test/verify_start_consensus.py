"""Real C++ init with controlled moderate scan spikes; isolated localhost domain194."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from verify_precise_correction import Room
import rclpy


class NoisyRoom(Room):
    def __init__(self, case, output):
        super().__init__('joint_consensus', output)
        self.case, self.scan_index = case, 0

    def distances(self, now):
        if self.verify_start is None:
            return 0.26, 0.28
        self.scan_index += 1
        index = self.scan_index % 15
        spike = index in ((0, 5, 10) if self.case == 'three_spikes' else (0, 4, 8, 12))
        return (0.272, 0.293) if spike else (0.26, 0.28)


def main():
    assert os.environ['ROS_DOMAIN_ID'] == '194' and os.environ['ROS_LOCALHOST_ONLY'] == '1'
    case = sys.argv[1]
    assert case in ('three_spikes', 'four_spikes')
    root = Path('/tmp/start_consensus') / f'{time.time_ns()}_{case}'
    root.mkdir(parents=True)
    rclpy.init()
    room, process = NoisyRoom(case, root), None
    try:
        with (root / 'runtime.log').open('w') as log:
            process = subprocess.Popen(
                [
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
                    'precise_start:=true',
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            deadline = time.monotonic() + 18
            while time.monotonic() < deadline:
                room.tick()
                text = (root / 'runtime.log').read_text()
                if 'START_VERIFIED:' in text or 'Initial preparation fault:' in text:
                    break
            else:
                raise AssertionError('Bounded init did not finish')
        if case == 'three_spikes':
            assert 'START_VERIFIED:' in text and 'consensus=12/15' in text, text
            assert 'START_VERIFY_TIMEOUT' not in text, text
        else:
            assert 'START_VERIFY_TIMEOUT' in text and 'START_VERIFIED:' not in text, text
            assert 'consensus=0/15' in text, text
        assert room.samples and all(not any(s['command']) for s in room.samples), room.samples
        (root / 'commands.json').write_text(json.dumps(room.samples))
        print('CONSENSUS_ROS_PASS', case, root, flush=True)
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=5)
        room.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
