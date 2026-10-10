"""Stopped reference sampling boundaries using the real capture method and a deterministic clock."""

import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from wall_runtime import WallRunner, WallFitError


class Fixture:
    def __init__(self, frames):
        self.frames = frames
        self.index = -1
        self.now = 0.0
        self.reference = 0.19
        self.latest = [None, None, 0, 0]
        self.cached_stamp = None

    def pump(self):
        self.now += 0.1
        self.index += 1
        self.frame = self.frames[min(self.index, len(self.frames) - 1)]
        self.cached_stamp = self.frame[0]

    def moving_pose(self):
        pass

    def walls(self, sides):
        if isinstance(self.frame[1], BaseException):
            raise self.frame[1]
        return {sides[0]: SimpleNamespace(gap=self.frame[1])}

    def run(self):
        with patch('wall_runtime.time.monotonic', lambda: self.now):
            return WallRunner.capture_reference(self, 'left')


class Capture(unittest.TestCase):
    def test_bad_frame_resets_and_five_new_frames_recover(self):
        f = Fixture(
            [(i, 0.1) for i in range(4)]
            + [(4, WallFitError('bad fit'))]
            + [(i, 0.12) for i in range(5, 10)]
        )
        self.assertAlmostEqual(f.run(), 0.12)
        self.assertEqual(f.index, 9)

    def test_repeated_stamp_cannot_finish(self):
        f = Fixture([(1, 0.1)])
        with self.assertRaisesRegex(RuntimeError, 'within 4 s'):
            f.run()
        self.assertEqual(f.reference, 0.19)
        self.assertLessEqual(f.now, 4.1)

    def test_permanent_bad_fit_times_out_without_replacing_reference(self):
        f = Fixture([(1, WallFitError('bad fit'))])
        with self.assertRaisesRegex(RuntimeError, 'last_fit_error=bad fit'):
            f.run()
        self.assertEqual(f.reference, 0.19)
        self.assertLessEqual(f.now, 4.1)

    def test_feedback_failure_and_cancellation_not_retried(self):
        for error in (
            RuntimeError('stale scan'),
            RuntimeError('TF unavailable'),
            KeyboardInterrupt(),
        ):
            f = Fixture([(1, error)])
            with self.assertRaises(type(error)):
                f.run()
            self.assertEqual(f.index, 0)
        f = Fixture([(1, 0.1)])
        with patch.object(f, 'moving_pose', side_effect=RuntimeError('odom stale')):
            with self.assertRaisesRegex(RuntimeError, 'odom stale'):
                f.run()
        self.assertEqual(f.reference, 0.19)


if __name__ == '__main__':
    unittest.main()
