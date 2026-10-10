"""Fixed-wall recovery must distinguish braking motion from surface replacement."""

import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from wall_geometry import Wall, transported_gaps, front_distance, support, swept_clearance
from wall_runtime import WallRunner


class WallTransport(unittest.TestCase):
    def test_cloud_failure_frames_keep_quality_limits(self):
        frames = json.loads((Path(__file__).parent / 'front_recovery_returns.json').read_text())
        self.assertEqual(len(frames), 16)
        for f in frames:
            w = front_distance(f['points'], f['counts'])
            self.assertLessEqual(w.rms, 0.012)
            self.assertLessEqual(abs(w.angle), math.radians(12))
            self.assertGreaterEqual(w.count / w.raw_count, 0.8)
            self.assertGreaterEqual(w.span, 0.08)
            # Nearby obstacle remains in the independent all-point guard.
            self.assertLess(swept_clearance(f['points'] + [(0.18, 0, 0)]), 0.02)

    def test_braking_travel_preserves_same_front_wall(self):
        w = {'front': Wall(0.89688, 0, 0, 0.4, 40, 40)}
        expected = transported_gaps(w, Pose(0, 0, 0), Pose(0.032, 0, 0))
        self.assertAlmostEqual(expected['front'], 0.86488)
        self.assertLess(abs(0.865 - expected['front']), 0.03)
        self.assertGreater(abs(0.80 - expected['front']), 0.03)

    def test_rotated_frame_and_support(self):
        w = {'front': Wall(0.9, 0, 0, 0.4, 40, 40), 'left': Wall(0.1, 0, 0, 0.2, 40, 40)}
        p = Pose(1, 2, math.pi / 2)
        q = Pose(1.01, 2.04, math.pi / 2 + 0.05)
        result = transported_gaps(w, p, q)
        self.assertAlmostEqual(
            result['front'], 0.9 - 0.04 + 0.17 - support(math.cos(-0.05), math.sin(-0.05))
        )
        self.assertAlmostEqual(
            result['left'],
            0.1 + 0.01 + 0.16 - support(math.cos(math.pi / 2 - 0.05), math.sin(math.pi / 2 - 0.05)),
        )

    def test_large_motion_rejected_and_wrap_allowed(self):
        w = {'front': Wall(0.9, 0, 0, 0.4, 40, 40)}
        for p in [Pose(0.09, 0, 0), Pose(0, 0, 0.16)]:
            with self.assertRaises(RuntimeError):
                transported_gaps(w, Pose(0, 0, 0), p)
        self.assertAlmostEqual(
            transported_gaps(w, Pose(0, 0, math.pi), Pose(0, 0, -math.pi))['front'], 0.9
        )

    def test_stale_pair_cannot_change_reference(self):
        r = object.__new__(WallRunner)
        r.cached_stamp = 200_000_000
        r.stamp = 0
        r.wall_snapshot = ({'front': Wall(0.9, 0, 0, 0.4, 40, 40)}, Pose(0, 0, 0))
        with self.assertRaisesRegex(RuntimeError, 'skew'):
            r.expected_gaps({'front': 0.9})
        with self.assertRaisesRegex(RuntimeError, 'skew'):
            r.remember_walls({})

    def recovery_fixture(self, observed_gap):
        from unittest.mock import Mock

        runner = object.__new__(WallRunner)
        runner.wall_snapshot = ({'front': Wall(0.9, 0, 0, 0.4, 40, 40)}, Pose(0, 0, 0))
        runner.latest = [Pose(0.04, 0, 0), 0, 0, 0]
        runner.stamp = runner.cached_stamp = 0
        runner.recovery_seconds = 0
        runner.observations = []
        runner.send = Mock()
        runner.guard_translation = Mock()
        runner.moving_pose = lambda: runner.latest[0]

        def pump():
            runner.stamp += 100_000_000
            runner.cached_stamp = runner.stamp

        runner.pump = pump
        runner.motion_walls = lambda *args: {'front': Wall(observed_gap, 0, 0, 0.4, 40, 40)}
        return runner

    def test_runtime_recovery_accepts_braking_but_not_replaced_wall(self):
        from unittest.mock import patch
        from contextlib import redirect_stdout
        from io import StringIO

        for gap, succeeds in [(0.86, True), (0.78, False)]:
            runner = self.recovery_fixture(gap)
            ticks = iter(i * 0.05 for i in range(200))
            with (
                patch('wall_runtime.time.monotonic', side_effect=lambda: next(ticks)),
                redirect_stdout(StringIO()),
            ):
                if succeeds:
                    runner.recover_walls({'front'}, 0, 'front', {'front': 0.9})
                    self.assertEqual(len(runner.observations), 3)
                    self.assertEqual(runner.wall_snapshot[1].x, 0.04)
                else:
                    with self.assertRaisesRegex(RuntimeError, 'WALL_RECOVERY_TIMEOUT'):
                        runner.recover_walls({'front'}, 0, 'front', {'front': 0.9})
                    self.assertEqual(runner.wall_snapshot[1].x, 0)
            self.assertTrue(
                all(call.args == ((0.0, 0.0, 0.0),) for call in runner.send.call_args_list)
            )

    def test_duplicate_scan_does_not_reanchor_at_new_pose(self):
        r = self.recovery_fixture(0.86)
        r.remember_walls({'front': Wall(0.9, 0, 0, 0.4, 40, 40)})
        first = r.wall_snapshot
        r.latest[0] = Pose(0.06, 0, 0)
        r.remember_walls({'front': Wall(0.9, 0, 0, 0.4, 40, 40)})
        self.assertEqual(r.wall_snapshot, first)


if __name__ == '__main__':
    unittest.main()
