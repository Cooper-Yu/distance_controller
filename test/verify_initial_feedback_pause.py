"""Synthetic C++ initialization pause checks on localhost domain 194; no robot connection."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from verify_precise_correction import Room, Odometry
import rclpy


class FeedbackRoom(Room):
    def __init__(self, mode, output):
        super().__init__('feedback_pause', output)
        self.case = mode
        self.started = time.monotonic()
        self.outage = self.restored = self.pause = self.resumed = None
        self.log_offset = 0

    def log_state(self, now):
        text = super().log_state(now)
        if self.outage is None and self.correction_start is not None:
            if now - self.correction_start > 0.12:
                self.outage = now
                self.pause = self.resumed = None
                self.log_offset = len(text)
        tail = text[self.log_offset:] if self.outage is not None else text
        if self.pause is None and 'FEEDBACK_RECOVERING:' in tail:
            self.pause = now
        if self.resumed is None and 'FEEDBACK_RECOVERED:' in tail:
            self.resumed = now
        return text

    def blocked(self, now, stream):
        if self.case == 'startup':
            return now - self.started < 1.2
        if self.outage is None or self.case in ('odom_reset', 'odom_invalid', 'odom_frame'):
            return False
        if self.case == 'timeout':
            return stream == 'odom'
        if self.case == 'scan' and stream != 'scan':
            return False
        if self.case == 'odom' and stream != 'odom':
            return False
        if now - self.outage < 1.5:
            return True
        if self.restored is None:
            self.restored = now
        return False

    def publish_odom(self, now):
        if self.outage is not None and self.case == 'odom_invalid':
            message = Odometry()
            message.header.frame_id, message.child_frame_id = 'odom', 'base_link'
            message.pose.pose.orientation.w = 1.0
            self.odom.publish(message)
            return
        if self.outage is not None and self.case in ('odom_reset', 'odom_frame'):
            self.mode = 'odom_stamp' if self.case == 'odom_reset' else 'odom_frame'
        if not self.blocked(now, 'odom'):
            super().publish_odom(now)

    def publish_scan(self, now):
        if not self.blocked(now, 'scan'):
            super().publish_scan(now)

    def distances(self, now):
        # An initial residual triggers one real correction; after interruption, require a new goal.
        if self.case == 'startup':
            return 0.26, 0.28
        if self.verify_start is None:
            return 0.26, 0.284
        return 0.26, 0.294 if self.outage is None else 0.28


def main():
    mode = sys.argv[1]
    assert mode in ('startup', 'scan', 'odom', 'both', 'timeout', 'odom_reset', 'odom_invalid', 'odom_frame')
    output = Path(sys.argv[2])
    output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    room = FeedbackRoom(mode, output)
    process = None
    try:
        with (output / 'runtime.log').open('w') as log:
            process = subprocess.Popen(
                ['ros2', 'run', 'distance_controller', 'distance_controller', '2', '--ros-args',
                 '-p', 'manual_mode:=true', '-p', 'start_paused:=true',
                 '-p', 'alignment_wall:=right', '-p', 'robust_wall_heading:=true',
                 '-p', 'positioning_wall:=right', '-p', 'right_target_distance:=0.26',
                 '-p', 'precise_start:=true'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
            )
            until = time.monotonic() + 28
            while time.monotonic() < until:
                room.tick()
                text = (output / 'runtime.log').read_text()
                if 'Initial preparation fault:' in text or 'START_VERIFIED:' in text:
                    break
            else:
                raise AssertionError('Initialization did not reach a bounded result')
            if mode in ('odom_reset', 'odom_invalid', 'odom_frame'):
                reason = 'FEEDBACK_POSE_OR_FRAME_JUMP' if mode == 'odom_frame' else 'FEEDBACK_ODOM_STAMP_RESET'
                assert reason in text and 'START_VERIFIED:' not in text, text
                assert room.outage is not None
                assert time.monotonic() - room.outage < 0.5
                (output / 'commands.json').write_text(json.dumps(room.samples))
                print('PASS', mode, output, flush=True)
                return
            assert room.pause is not None, text
            if mode == 'timeout':
                assert 'FEEDBACK_RECOVERY_TIMEOUT' in text, text
                assert time.monotonic() - room.pause < 5.3
                assert 'FEEDBACK_RECOVERED:' not in text[room.log_offset:]
            else:
                assert 'START_VERIFIED:' in text, text
                assert room.resumed is not None
                if mode != 'startup':
                    assert room.restored is not None
                    assert room.resumed - room.restored >= 0.29
                    assert 'START_CORRECTION_CANCELLED:' in text
            # Every command after observed pause and before observed resume remains zero.
            # The final transition tick also publishes zero, allowing this receive-side check.
            commands = [s for s in room.samples
                        if s['t'] >= room.pause and
                        (room.resumed is None or s['t'] <= room.resumed)]
            assert commands and all(not any(s['command']) for s in commands), commands
            (output / 'commands.json').write_text(json.dumps(room.samples))
            (output / 'result.json').write_text(json.dumps(dict(
                mode=mode, outage=room.outage, pause=room.pause,
                restored=room.restored, resumed=room.resumed, pause_commands=len(commands))))
            print('PASS', mode, output, flush=True)
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=5)
        room.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()