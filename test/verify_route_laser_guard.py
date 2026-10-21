#!/usr/bin/env python3
"""Installed Console/guard real-node tests, isolated from production entrypoint and hardware.

Uses synthetic-room Fixture sensors. The child imports installed code and injects
the optional guard explicitly; no production CLI or default is changed.
"""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from unittest.mock import patch

from verify_task6_route_runner import Fixture, rclpy

os.environ['ROS_DOMAIN_ID'] = '193'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ.pop('CYCLONEDDS_URI', None)


class GuardFixture(Fixture):
    def events(self):
        """Ignore an unfinished final JSONL write while reading a live large audit."""
        audit = self.path.with_suffix('.jsonl')
        if not audit.exists():
            return []
        return [
            json.loads(line)
            for line in audit.read_text().splitlines(keepends=True)
            if line.endswith('\n')
        ]

    def launch(self, source):
        environment = dict(
            os.environ, LASER_FIXTURE_SHADOW='1' if self.mode.startswith('shadow_') else '0'
        )
        return subprocess.Popen(
            [
                sys.executable,
                str(Path(__file__).with_name('route_laser_fixture_child.py')),
                '--scene',
                '2',
                '--route-json',
                str(source),
                '--laser-log-only',
                '--output',
                str(self.path),
            ],
            stdin=subprocess.DEVNULL,
            stdout=self.output,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            env=environment,
        )

    def inject(self):
        if self.injected:
            return
        actions = [e for e in self.events() if e['event'] == 'route_action_started']
        if not actions:
            return
        if self.mode == 'cancel' and self.velocity[0] > 0.005:
            os.killpg(self.child.pid, signal.SIGINT)
        elif (
            self.mode == 'blind_turn'
            and actions[-1]['name'] == 'P02_after_turn'
            and self.velocity[2] > 0.005
        ):
            pass
        elif (
            self.mode in ('scan_stale', 'unknown', 'overlap', 'shadow_unknown', 'shadow_stale')
            and self.velocity[0] > 0.005
        ):
            pass
        else:
            return
        self.injected = True
        self.injection_time = time.monotonic()

    def publish_scan(self, stamp):
        if self.injected and self.mode in ('scan_stale', 'shadow_stale'):
            return
        original = self.scan.publish

        def publish(scan):
            if self.injected:
                if self.mode in ('unknown', 'shadow_unknown'):
                    scan.ranges[0] = float('nan')  # Relevant front ray; never converted to free.
                elif self.mode == 'blind_turn':
                    scan.ranges[90] = float('nan')  # One diagonal filtered ray during rotation.
                elif self.mode == 'overlap':
                    scan.ranges[180] = 0.15  # Valid left ray inside the wheel rectangle.
            original(scan)

        with patch.object(self.scan, 'publish', side_effect=publish):
            super().publish_scan(stamp)

    def verify(self):
        self.wait(lambda: self.child.poll() is not None, 105)
        self.wait(self.released, 10)
        self.wait(lambda: not self.node.get_publishers_info_by_topic('/cmd_vel'), 5)
        events = self.events()
        guards = [e for e in events if e['event'] == 'laser_guard']
        assert guards, self.log.read_text()[-5000:]
        if self.mode == 'success' or self.mode.startswith('shadow_'):
            assert self.child.returncode == 0, self.log.read_text()[-5000:]
            assert events[-1]['event'] == 'route_completed', events[-1]
            points = json.loads(self.path.read_text())['points']
            assert list(points) == ['P01', 'P02', 'P02_after_turn', 'P03']
        else:
            assert self.injected, self.mode
            assert self.child.returncode != 0, self.mode
            assert events[-1]['event'] == 'route_failed', events[-1]
            assert not any(e['event'] == 'route_completed' for e in events)
            reason = events[-1]['reason']
            expected = {
                'scan_stale': 'LASER_GUARD_UNKNOWN',
                'unknown': 'LASER_GUARD_UNKNOWN',
                'blind_turn': 'LASER_GUARD_UNKNOWN',
                'overlap': 'LASER_GUARD_BLOCKED',
                'cancel': 'Cancelled',
            }[self.mode]
            assert expected in reason, events[-1]
        if self.mode.startswith('shadow_'):
            assert self.injected
            assert all(e['mode'] == 'shadow' and not e['enforced'] for e in guards)
            assert any(e['decision']['state'] == 'unknown' for e in guards)
            assert all(e['applied_command'] == e['proposed_command'] for e in guards)
            assert 'SHADOW ONLY' in self.log.read_text()
        else:
            assert all(e['mode'] == 'enforced' and e['enforced'] for e in guards)
        assert self.commands and sum(abs(v) for v in self.commands[-1]) == 0
        print(
            self.mode,
            'PASS',
            'exit',
            self.child.returncode,
            'last_zero/pub0/lock_free',
            self.log,
            flush=True,
        )


def main():
    output = Path('/tmp/route_laser_ros') / str(time.time_ns())
    output.mkdir(parents=True)
    rclpy.init()
    try:
        modes = os.getenv(
            'LASER_GUARD_CASES',
            'success,scan_stale,unknown,overlap,cancel,blind_turn,shadow_unknown,shadow_stale',
        )
        for mode in modes.split(','):
            fixture = GuardFixture(output / mode, mode)
            try:
                fixture.verify()
            finally:
                fixture.close()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
