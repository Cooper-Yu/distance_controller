"""Stopped recovery keeps anchors, history and travel budgets; rejects epoch/pose changes."""

import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose, Step
from wall_session import WallSession
from session_checkpoint import write_checkpoint, read_checkpoint, restore_checkpoint


class Backend:
    def __init__(self):
        self.current = Pose(0, 0, 0)
        self.frame, self.stamp = 'odom', 100
        self.reference, self.max_speed = 0.1, 0.06
        self.continuous_yaw = 0.0
        self.latest = (self.current, 0, 0, 0)
        self.last_report = {}
        self.fail = False
        self.calls = []

    def pose(self):
        return self.current

    def execute(self, step, target):
        self.calls.append(
            (target, getattr(self, 'resume_origin', None), getattr(self, 'resume_path', 0))
        )
        if self.fail:
            self.current = Pose(0.4, 0, 0)
            self.last_report = {'path_m': 0.41}
            raise RuntimeError('test interruption')
        self.current = target
        self.last_report = {'path_m': 0.9}
        return target


class Recovery(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'state.json'
        self.b = Backend()
        self.s = WallSession(
            Pose(0, 0, 0),
            [
                Step('one', 'forward', 0.9, {'follow': 'left', 'max_travel': 1.2}),
                Step('two', 'turn', -math.pi / 2, {'capture': 'left'}),
            ],
            self.b,
            lambda *args: None,
        )

    def snapshot(self):
        write_checkpoint(self.path, self.s)
        return read_checkpoint(self.path)

    def test_completed_restore_has_no_motion_and_retains_plan(self):
        self.s.next()
        self.s.edit(1, -math.pi)
        data = self.snapshot()
        b = Backend()
        b.current = self.b.current
        b.latest = (b.current, 0, 0, 0)
        restored = restore_checkpoint(data, b, lambda *args: None)
        self.assertEqual(restored.cursor, 1)
        self.assertEqual(restored.steps[1].value, -math.pi)
        self.assertEqual(restored.active, json.loads(json.dumps(self.s.active)))
        self.assertEqual(b.calls, [])
        restored.back()
        self.assertEqual(restored.cursor, 0)
        self.assertEqual(b.current, Pose(0, 0, 0))

    def test_partial_restore_resumes_original_anchor_target_budget(self):
        self.b.fail = True
        with self.assertRaises(RuntimeError):
            self.s.next()
        data = self.snapshot()
        b = Backend()
        b.current = self.b.current
        restored = restore_checkpoint(data, b, lambda *args: None)
        restored.resume()
        self.assertEqual(b.calls[0], (Pose(0.9, 0, 0), Pose(0, 0, 0), 0.41))
        self.assertEqual(restored.cursor, 1)
        self.assertIsNone(restored.partial)
        self.assertIsNone(b.resume_origin)

    def test_changed_pose_frame_or_time_refused(self):
        data = self.snapshot()
        for field, value in [
            ('current', Pose(0.1, 0, 0)),
            ('current', Pose(0, 0, 0.1)),
            ('frame', 'map'),
            ('stamp', 99),
        ]:
            b = Backend()
            setattr(b, field, value)
            with self.assertRaises(RuntimeError):
                restore_checkpoint(data, b, lambda *args: None)
            self.assertEqual(b.calls, [])

    def test_busy_and_corrupt_snapshots_refused(self):
        write_checkpoint(self.path, self.s, usable=False)
        with self.assertRaises(ValueError):
            read_checkpoint(self.path)
        data = self.snapshot()
        data['cursor'] = 1
        self.path.write_text(json.dumps(data))
        with self.assertRaises(ValueError):
            read_checkpoint(self.path)

    def test_continuous_yaw_branch_restored(self):
        self.s.last_actual = Pose(0, 0, 2 * math.pi + 0.1)
        data = self.snapshot()
        b = Backend()
        b.current = Pose(0, 0, 0.1)
        b.latest = (b.current, 0, 0, 0)
        b.continuous_yaw = 0.1
        restore_checkpoint(data, b, lambda *args: None)
        self.assertAlmostEqual(b.latest[0].yaw, 2 * math.pi + 0.1)

    def test_failed_return_cannot_resume_outward(self):
        self.s.next()
        self.b.fail = True
        with self.assertRaises(RuntimeError):
            self.s.back()
        data = self.snapshot()
        b = Backend()
        b.current = self.b.current
        restored = restore_checkpoint(data, b, lambda *args: None)
        with self.assertRaisesRegex(RuntimeError, 'No resumable'):
            restored.resume()

    def test_legacy_stopped_audit_migrates_for_return_only(self):
        from dataclasses import asdict

        self.b.fail = True
        with self.assertRaises(RuntimeError):
            self.s.next()
        rows = [
            {
                'event': 'origin',
                'data': {
                    'pose': asdict(self.s.origin),
                    'actions': [asdict(s) for s in self.s.steps],
                },
            },
            {
                'event': 'initial_reference',
                'data': {'body_clearance_m': 0.1, 'speed_cap_m_s': 0.06},
            },
            {'event': 'started', 'data': self.s.partial},
            {
                'event': 'incomplete',
                'data': {**self.s.partial, 'last_stop': asdict(self.b.current)},
            },
        ]
        audit = self.path.with_suffix('.jsonl')
        audit.write_text('\n'.join(json.dumps(r) for r in rows))
        data = read_checkpoint(audit)
        restored = restore_checkpoint(data, self.b, lambda *args: None)
        with self.assertRaises(RuntimeError):
            restored.resume()
        self.b.fail = False
        restored.back()
        self.assertEqual(self.b.current, Pose(0, 0, 0))
        audit.write_text('\n'.join(json.dumps(r) for r in rows[:-1]))
        with self.assertRaises(ValueError):
            read_checkpoint(audit)


if __name__ == '__main__':
    unittest.main()
