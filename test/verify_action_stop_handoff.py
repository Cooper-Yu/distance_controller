#!/usr/bin/env python3
"""Isolated actual C++ preparation fault/cancel -> Python zero-owner handoff.

Source Python backend plus installed C++ controller; localhost domain 198 only.
No timing shim, production parameters or real robot connection.
"""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from verify_task6_route_runner import Fixture, rclpy

os.environ['ROS_DOMAIN_ID'] = '198'
os.environ['ROS_LOCALHOST_ONLY'] = '1'


class HandoffFixture(Fixture):
    def __init__(self, directory, mode):
        self.maximum_publishers = 0
        self.endpoint_samples = []
        super().__init__(directory, mode)
        if mode == 'cancel_moving':
            self.pose[0] = 0.04
        self.graph_timer = self.node.create_timer(0.002, self.observe_graph)

    def launch(self, source):
        tools = Path(__file__).resolve().parents[1] / 'tools'
        entry = self.directory / 'source_backend.py'
        entry.write_text(
            'import sys\n'
            f'sys.path.insert(0, {str(tools)!r})\n'
            'import task6_route_runner as backend\n'
            'Original=backend.WallRunner\n'
            'def factory(*a,**k):\n'
            '    runner=Original(*a,**k)\n'
            + (
                "    runner.prepare_options += ['-p','positioning_timeout:=0.40']\n"
                if self.mode == 'initial_fault'
                else ''
            )
            + '    return runner\n'
            'backend.WallRunner=factory\n'
            'raise SystemExit(backend.main())\n'
        )
        return subprocess.Popen(
            [
                sys.executable,
                str(entry),
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
        )

    def observe_graph(self):
        endpoints = self.node.get_publishers_info_by_topic('/cmd_vel')
        self.maximum_publishers = max(self.maximum_publishers, len(endpoints))
        row = (time.monotonic(), [(e.node_name, bytes(e.endpoint_gid).hex()) for e in endpoints])
        self.endpoint_samples.append(row)

    def inject(self):
        if self.injected or self.mode != 'cancel_moving':
            return
        if any(abs(v) > 1e-4 for v in self.velocity):
            self.injected = True
            self.cancel_at = time.monotonic()
            os.killpg(self.child.pid, signal.SIGINT)

    def verify_handoff(self):
        self.wait(lambda: self.child.poll() is not None, 25)
        self.wait(self.released, 5)
        self.wait(lambda: not self.node.get_publishers_info_by_topic('/cmd_vel'), 4)
        text = self.log.read_text()
        events = self.events()
        assert self.child.returncode == 2, text
        assert events[-1]['event'] == 'route_failed', events
        assert not any(e['event'] == 'preparation_handoff' for e in events), events
        if self.mode == 'initial_fault':
            assert 'POSITIONING_TIMEOUT' in text, text
            assert 'state=FAULT' in events[-1]['reason'], events[-1]
        else:
            assert self.injected, 'Must cancel after actual nonzero command'
            assert 'Cancelled' in events[-1]['reason'], events[-1]
        assert 'Secondary owned cleanup failure' not in text, text
        assert self.commands and all(v == 0 for v in self.commands[-1]), self.commands[-5:]
        assert self.maximum_publishers == 1, self.endpoint_samples
        parents = [
            row for row in self.endpoint_samples if any(n == 'supervised_return' for n, _ in row[1])
        ]
        assert parents, 'Parent zero owner was not observed'
        report = dict(
            mode=self.mode,
            exit_code=self.child.returncode,
            max_cmd_publishers=self.maximum_publishers,
            commands=len(self.commands),
            nonzero=sum(any(v != 0 for v in c) for c in self.commands),
            last_command=self.commands[-1],
            final_publishers=[],
            lock_released=self.released(),
            event=events[-1],
            parent_zero_owner_observed=True,
        )
        (self.directory / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
        (self.directory / 'graph.json').write_text(json.dumps(self.endpoint_samples) + '\n')
        print(json.dumps(report), flush=True)


def main():
    assert os.environ['ROS_DOMAIN_ID'] == '198' and os.environ['ROS_LOCALHOST_ONLY'] == '1'
    root = Path('/tmp/action_stop_handoff_ros') / str(time.time_ns())
    root.mkdir(parents=True)
    print('EVIDENCE', root, flush=True)
    rclpy.init()
    fixture = None
    try:
        for mode in ('initial_fault', 'cancel_moving'):
            fixture = HandoffFixture(root / mode, mode)
            fixture.verify_handoff()
            fixture.close()
            fixture.node.destroy_node()
            fixture = None
    finally:
        if fixture is not None:
            try:
                if fixture.child.poll() is None:
                    os.killpg(fixture.child.pid, signal.SIGINT)
                    fixture.wait(lambda: fixture.child.poll() is not None, 5)
            finally:
                if fixture.child.poll() is None:
                    os.killpg(fixture.child.pid, signal.SIGKILL)
                    fixture.child.wait(timeout=2)
                fixture.output.close()
                fixture.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
