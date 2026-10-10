"""Stationary acceptance is robust to bounded noise, never stale or unsafe samples."""

import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from front_settling import StoppedWindow


class Settling(unittest.TestCase):
    def window(self, values, interval=0.1, stopped=True):
        w = StoppedWindow()
        for i, value in enumerate(values):
            gaps = None if value is None else {'front': value, 'left': 0.115}
            w.add(i, i * interval, gaps, stopped)
        return w.result(0.08, 'left', 0.118)

    def test_bounded_noise_and_two_rejected_fits(self):
        self.assertIsNotNone(self.window([0.071, None, 0.077, 0.081, None, 0.074, 0.075]))

    def test_reject_bias_spread_low_floor_motion_and_age(self):
        for values in (
            [0.06] * 7,
            [0.10] * 7,
            [0.064] + [0.08] * 6,
            [0.068, 0.091] + [0.08] * 5,
            [None] * 3 + [0.08] * 4,
        ):
            self.assertIsNone(self.window(values))
        self.assertIsNone(self.window([0.08] * 7, interval=0.3))
        self.assertIsNone(self.window([0.08] * 7, stopped=False))

    def test_duplicate_stamp_cannot_confirm(self):
        w = StoppedWindow()
        for i in range(20):
            w.add(1, i * 0.1, {'front': 0.08}, True)
        self.assertIsNone(w.result(0.08, 'none', None))

    def test_raw_guard_and_stale_feedback_still_abort_window(self):
        from types import SimpleNamespace
        from action_plan import Pose
        from wall_policy import Policy
        from front_settling import verify_stopped

        for failure in ('obstacle', 'stale'):
            sent = []

            def reject():
                raise RuntimeError(failure)

            runner = SimpleNamespace(
                reference=0.12,
                send=sent.append,
                pump=reject if failure == 'stale' else lambda: None,
                moving_pose=lambda: Pose(0, 0, 0),
                guard_translation=lambda *args: reject(),
            )
            with self.assertRaisesRegex(RuntimeError, failure):
                verify_stopped(runner, Pose(0, 0, 0), Policy(stop='front', stop_clearance=0.08))
            self.assertEqual(sent, [(0.0, 0.0, 0.0)])
