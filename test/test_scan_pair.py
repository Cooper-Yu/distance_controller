"""Scan-time pairing rejects gaps, but does not reject a retained scan as odom advances."""

import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from wall_geometry import pose_at_scan
from wall_runtime import WallRunner, WallFitError


class ScanPair(unittest.TestCase):
    def test_old_scan_uses_history_not_newest(self):
        history = [
            (100_000_000, Pose(1, 0, 0)),
            (200_000_000, Pose(2, 0, 0)),
            (400_000_000, Pose(4, 0, 0)),
        ]
        pose, offset = pose_at_scan(history, 150_000_000)
        self.assertEqual(pose.x, 1.5)
        self.assertEqual(offset, 50_000_000)

    def test_continuous_yaw_interpolates_across_pi(self):
        p, _ = pose_at_scan([(0, Pose(0, 0, 3.1)), (100, Pose(0, 0, 3.2))], 50)
        self.assertAlmostEqual(p.yaw, 3.15)

    def test_missing_large_gap_and_outside_bounds_rejected(self):
        for history, stamp in [
            ([], 0),
            ([(0, Pose(0, 0, 0))], 160_000_000),
            ([(0, Pose(0, 0, 0)), (400_000_000, Pose(0, 0, 0))], 200_000_000),
        ]:
            with self.assertRaises(ValueError):
                pose_at_scan(history, stamp)

    def test_nearest_fallback_is_bounded(self):
        p, offset = pose_at_scan([(100_000_000, Pose(1, 2, 3))], 130_000_000)
        self.assertEqual(p, Pose(1, 2, 3))
        self.assertEqual(offset, 30_000_000)

    def test_pair_is_frozen_while_latest_odom_advances(self):
        r = object.__new__(WallRunner)
        r.latest = [Pose(1, 0, 0), 0, 0, 0]
        r.stamp = 100_000_000
        r.cached_stamp = 100_000_000
        r.odom_history = [(r.stamp, r.latest[0])]
        r.moving_pose = lambda: r.latest[0]
        self.assertEqual(r.scan_pose().x, 1)
        r.stamp = 400_000_000
        r.latest[0] = Pose(4, 0, 0)
        self.assertEqual(r.scan_pose().x, 1)
        r.cached_stamp = 600_000_000
        with self.assertRaises(WallFitError):
            r.scan_pose()


if __name__ == '__main__':
    unittest.main()
