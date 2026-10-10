"""Guard failure diagnostics preserve actual geometry and unchanged decision boundaries."""

import sys
from pathlib import Path
import unittest
from unittest.mock import patch
from contextlib import redirect_stdout
from io import StringIO
import math

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from action_runtime import PlannedRunner
from wall_runtime import WallRunner


class GuardDiagnostics(unittest.TestCase):
    def fixture(self, speed=0.0, wz=0.0, sign=0):
        runner = object.__new__(WallRunner)
        runner.latest = [Pose(0, 0, 0), 0, speed, wz]
        runner.moving_pose = lambda: runner.latest[0]
        runner.turn_guard = True
        runner.turn_sign = sign
        runner.cached_stamp = 123
        runner.observations = []
        runner.geometry = lambda: ([(-0.22837, -0.04262, -2.957)], {})
        return runner

    def test_static_safe_frame_does_not_reject(self):
        runner = self.fixture()
        with patch.object(PlannedRunner, 'pump'):
            runner.pump()
        self.assertEqual(runner.observations, [])

    def test_return_allowances_trigger_and_are_recorded(self):
        runner = self.fixture(speed=0.03, wz=0.3)
        with (
            patch.object(PlannedRunner, 'pump'),
            redirect_stdout(StringIO()),
            self.assertRaisesRegex(RuntimeError, 'RETURN_CLEARANCE'),
        ):
            runner.pump()
        d = runner.observations[0]['guard_failure']
        self.assertAlmostEqual(d['static_gap_m'], 0.05837)
        self.assertAlmostEqual(d['linear_allowance_m'], 0.015)
        self.assertAlmostEqual(d['angular_allowance_m'], 0.033)
        self.assertAlmostEqual(d['protected_gap_m'], 0.01037)
        self.assertEqual(d['scan_stamp_ns'], 123)
        self.assertEqual(len(d['points_base_xy_bearing']), 1)

    def test_turn_records_sweep_without_return_allowances(self):
        runner = self.fixture(sign=1)
        runner.geometry = lambda: ([(0.18, 0.0, 0.0)], {})
        with (
            patch.object(PlannedRunner, 'pump'),
            redirect_stdout(StringIO()),
            self.assertRaisesRegex(RuntimeError, 'TURN_CLEARANCE'),
        ):
            runner.pump()
        d = runner.observations[0]['guard_failure']
        self.assertEqual(d['mode'], 'turn')
        self.assertEqual(d['linear_allowance_m'], 0)
        self.assertTrue(math.isfinite(d['sweep_gap_m']))

    def translation_fixture(self, gap):
        runner = self.fixture()
        points = [(1.0, i * 0.005, 0.0) for i in range(10)]
        points.append((0.076, -0.16 - gap, -1.19))
        runner.geometry = lambda: (points, {'front': 10, 'right': 10})
        return runner

    def test_zero_command_preserves_safe_close_returns(self):
        runner = self.translation_fixture(0.0278)
        runner.guard_translation((0, 0, 0), 'front')
        self.assertEqual(runner.observations, [])

    def test_recovery_zero_command_records_static_obstacle(self):
        runner = self.translation_fixture(0.015)
        with redirect_stdout(StringIO()), self.assertRaisesRegex(RuntimeError, 'OBSTACLE'):
            runner.guard_translation((0, 0, 0), 'front')
        detail = runner.observations[0]['guard_failure']
        self.assertTrue(detail['zero_command'])
        self.assertAlmostEqual(detail['static_gap_m'], 0.015)
        self.assertAlmostEqual(detail['protected_gap_m'], 0.015)
        self.assertEqual(len(detail['points_base_xy_bearing']), 11)
        self.assertLess(detail['limiting_bearing_deg'], 0)

    def test_moving_sweep_records_command_and_predictive_gap(self):
        runner = self.fixture()
        points = [(0.21, i * 0.001, 0) for i in range(10)]
        runner.geometry = lambda: (points, {'front': 10})
        with redirect_stdout(StringIO()), self.assertRaisesRegex(RuntimeError, 'OBSTACLE'):
            runner.guard_translation((0.06, 0, 0), 'front')
        detail = runner.observations[0]['guard_failure']
        self.assertFalse(detail['zero_command'])
        self.assertEqual(detail['command_vx_vy_wz'], (0.06, 0, 0))
        self.assertAlmostEqual(detail['static_gap_m'], 0.04)
        self.assertAlmostEqual(detail['protected_gap_m'], 0.01)


if __name__ == '__main__':
    unittest.main()
