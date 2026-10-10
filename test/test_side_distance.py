"""Recorded side scans, ambiguous surfaces and bounded stationary recovery."""

import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from wall_geometry import side_distance, swept_clearance
from wall_runtime import WallRunner, WallFitError


def points(gap=0.1, rotation=0):
    c, s = math.cos(rotation), math.sin(rotation)
    return [
        (
            c * x - s * (gap + 0.16),
            s * x + c * (gap + 0.16),
            math.atan2(s * x + c * (gap + 0.16), c * x - s * (gap + 0.16)),
        )
        for x in [i * 0.005 for i in range(-28, 29)]
    ]


class SideDistance(unittest.TestCase):
    def test_three_recorded_scans(self):
        frames = json.loads((Path(__file__).parent / 'side_distance_scans.json').read_text())
        for name, f in frames.items():
            w = side_distance(f['points'], f['counts'], 'left', 0)
            self.assertGreaterEqual(w.count / w.raw_count, 0.8, name)
            self.assertLess(w.rms, 0.012, name)
        w = side_distance(
            frames['task6_scan_p01_partial']['points'],
            frames['task6_scan_p01_partial']['counts'],
            'left',
            0,
        )
        self.assertAlmostEqual(w.gap, 0.10154, places=4)

    def test_heading_compensation_and_mirrored_side(self):
        for angle in [-0.06, 0, 0.06]:
            p = points(rotation=angle)
            self.assertAlmostEqual(
                side_distance(p, {'left': len(p)}, 'left', angle).gap,
                0.26 - (abs(math.sin(angle)) * 0.135 + abs(math.cos(angle)) * 0.16),
            )
            mirrored = [(x, -y, -a) for x, y, a in p]
            self.assertAlmostEqual(
                side_distance(mirrored, {'right': len(p)}, 'right', -angle).gap,
                side_distance(p, {'left': len(p)}, 'left', angle).gap,
            )

    def test_opening_competing_walls_sparse_and_wrong_heading(self):
        a = points()
        b = points(0.3)
        for p, count in [(a[::4], len(a)), (a[:28] + b[28:], 57), (points(rotation=0.35), 57)]:
            with self.assertRaises(ValueError):
                side_distance(p, {'left': count}, 'left', 0)

    def test_near_obstacle_not_removed_from_guard(self):
        p = points() + [(0, 0.17, math.pi / 2)]
        side_distance(p, {'left': len(p)}, 'left', 0)
        self.assertLess(swept_clearance(p), 0.02)


class RecoveryFixture:
    def __init__(self, frames):
        self.frames = frames
        self.i = -1
        self.now = 0.0
        self.latest = [None, None, 0.0, 0.0]
        self.recovery_seconds = 0.0
        self.sent = []

    def send(self, v):
        self.sent.append(v)

    def pump(self):
        self.now += 0.1
        self.i += 1
        self.frame = self.frames[min(self.i, len(self.frames) - 1)]
        self.cached_stamp = self.frame[0]

    def moving_pose(self):
        pass

    def guard_translation(self, *args):
        pass

    def motion_walls(self, *args):
        if isinstance(self.frame[1], BaseException):
            raise self.frame[1]
        return {'left': SimpleNamespace(gap=self.frame[1])}

    def run(self, prior=None):
        with patch('wall_runtime.time.monotonic', lambda: self.now):
            return WallRunner.recover_walls(self, {'left'}, 0, 'front', prior or {})


class Recovery(unittest.TestCase):
    def test_three_distinct_frames_and_reset_on_bad_fit(self):
        f = RecoveryFixture(
            [(1, 0.1), (2, WallFitError('bad')), (3, 0.1), (3, 0.1), (4, 0.101), (5, 0.102)]
        )
        f.run({'left': 0.1})
        self.assertEqual(f.i, 5)
        self.assertTrue(all(v == (0.0, 0.0, 0.0) for v in f.sent))

    def test_persistent_loss_duplicate_frames_and_new_wall_timeout(self):
        for frames, prior in [
            ([(1, WallFitError('bad'))], {}),
            ([(1, 0.1)], {}),
            ([(i, 0.3) for i in range(30)], {'left': 0.1}),
        ]:
            f = RecoveryFixture(frames)
            with self.assertRaisesRegex(RuntimeError, 'WALL_RECOVERY_TIMEOUT'):
                f.run(prior)
            self.assertLess(f.now, 2.2)

    def test_stale_and_cancel_propagate(self):
        for error in [RuntimeError('scan stale'), KeyboardInterrupt()]:
            f = RecoveryFixture([(1, error)])
            with self.assertRaises(type(error)):
                f.run()
            self.assertEqual(f.i, 0)

    def test_moving_feedback_and_cumulative_budget(self):
        f = RecoveryFixture([(i, 0.1) for i in range(30)])
        f.latest[2] = 0.02
        with self.assertRaisesRegex(RuntimeError, 'TIMEOUT'):
            f.run()
        f = RecoveryFixture([(i, 0.1) for i in range(30)])
        f.recovery_seconds = 3.9
        with self.assertRaisesRegex(RuntimeError, 'BUDGET'):
            f.run()


if __name__ == '__main__':
    unittest.main()
