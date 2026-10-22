"""Current-pose turn failures never enter motion or bypass cleanup."""

import io
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
import task6_current_turn as turn


class CurrentTurnTest(unittest.TestCase):
    def runner(self):
        runner = Mock()
        runner.pose.return_value = Pose(1, 2, 0.2)
        runner.geometry.return_value = ([(1.0, 0.0, 0.0)] * 20, {})
        runner.node.get_publishers_info_by_topic.return_value = [
            SimpleNamespace(endpoint_gid=b'original')
        ]
        return runner

    def run_case(self, runner, effect=None):
        console = Mock()
        console.finish_motion.return_value = Pose(1, 2, 0.2 + math.pi)
        if effect:
            console.finish_motion.side_effect = lambda *_a, **_k: effect(console, runner)
        with (
            patch.object(turn, 'WallRunner', return_value=runner),
            patch.object(turn, 'Console', return_value=console),
            patch.object(turn.signal, 'signal'),
            patch.object(turn, 'close_runner') as close,
        ):
            result = turn.execute(SimpleNamespace(turn_speed=0.2, max_seconds=45), io.StringIO())
        close.assert_called_once_with(runner)
        return result, console

    def test_fresh_current_anchor_and_signed_pi_success(self):
        result, console = self.run_case(self.runner())
        self.assertTrue(result['success'])
        self.assertAlmostEqual(result['signed_rotation_rad'], math.pi)
        self.assertEqual(result['scope'], 'standalone_current_turn_not_full_route_acceptance')
        args = console.finish_motion.call_args.args
        self.assertEqual(args[0], Pose(1, 2, 0.2))
        self.assertEqual(args[1].angle, math.pi)

    def test_invalid_laser_never_starts(self):
        runner = self.runner()
        runner.geometry.return_value = ([], {})
        result, console = self.run_case(runner)
        self.assertFalse(result['success'])
        self.assertIn('Insufficient valid', result['error'])
        console.finish_motion.assert_not_called()

    def test_publisher_change_during_motion_rejected(self):
        def changed(console, runner):
            runner.node.get_publishers_info_by_topic.return_value = [
                SimpleNamespace(endpoint_gid=b'replacement')
            ]
            return console.command_guard((0.0, 0.0, 0.1))

        result, _ = self.run_case(self.runner(), changed)
        self.assertIn('identity changed', result['error'])

    def test_scan_loss_during_motion_rejected(self):
        def stale(console, runner):
            runner.geometry.side_effect = RuntimeError('Required laser scan stale')
            return console.command_guard((0.0, 0.0, 0.1))

        result, _ = self.run_case(self.runner(), stale)
        self.assertIn('scan stale', result['error'])

    def test_cancellation_propagates_to_failed_result(self):
        def cancelled(_console, _runner):
            raise KeyboardInterrupt

        result, _ = self.run_case(self.runner(), cancelled)
        self.assertEqual(result['error'], 'Cancelled')


if __name__ == '__main__':
    unittest.main()
