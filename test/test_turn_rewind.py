"""Yaw-only rollback must keep route targets and fixed recovery/checkpoint state."""

from dataclasses import asdict
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose, Step
from wall_session import WallSession
from session_checkpoint import validate_records


class Rewind(unittest.TestCase):
    def setUp(self):
        self.actual = Pose(1, 2, -0.89)
        self.backend = SimpleNamespace(pose=lambda: self.actual, check_turn=lambda t: None)
        step = Step('turn', 'turn', -1.5708, {'capture': 'left'})
        self.s = WallSession(self.actual, [step], self.backend, lambda *a: None)
        self.s.partial = dict(
            index=0,
            revision=0,
            step=asdict(step),
            start=asdict(Pose(1, 2.02, -0.01)),
            target=asdict(Pose(1, 2, -1.5808)),
        )

    def finish(self, backend, target):
        self.actual = target

    def test_fixed_yaw_only_and_additional_adjustment(self):
        self.s.partial['turn_adjustment'] = {'done': True}
        with patch('turn_adjustment.run_rewind', side_effect=self.finish) as turn:
            self.s.rewind_turn()
            self.s.rewind_turn()
            turn.assert_called_once()
            self.assertEqual(turn.call_args.args[1], Pose(1, 2, -0.01))
        self.assertEqual(self.s.cursor, 0)
        self.assertAlmostEqual(self.s.partial['target']['yaw'], -1.5808)
        self.assertIn('turn_adjustment', self.s.partial['rewind_previous_adjustments'])
        with patch(
            'turn_adjustment.run_adjustment',
            side_effect=lambda b, state: (
                setattr(self, 'actual', Pose(**state['target'])),
                state.update(done=True),
            ),
        ):
            self.s.adjust_turn(0.02)
            first = self.actual
            self.s.adjust_turn(0.02)
        self.assertEqual(self.actual, first)
        self.assertLess(self.actual.y, 1.981)
        validate_records({'active': [], 'partial': self.s.partial}, self.s.steps)

    def test_failed_rewind_blocks_other_motion_and_retains_target(self):
        def fail(b, target):
            self.actual = Pose(1.001, 2, -0.5)
            raise RuntimeError('blocked')

        with patch('turn_adjustment.run_rewind', side_effect=fail):
            with self.assertRaisesRegex(RuntimeError, 'blocked'):
                self.s.rewind_turn()
        target = self.s.partial['turn_rewind']['target'].copy()
        for action in (self.s.resume, self.s.back, lambda: self.s.adjust_turn(0.02)):
            with self.assertRaises((RuntimeError, ValueError)):
                action()
        with patch('turn_adjustment.run_rewind', side_effect=self.finish) as turn:
            self.s.rewind_turn()
        self.assertEqual(asdict(turn.call_args.args[1]), target)
        self.assertEqual(self.s.last_actual, self.actual)

    def test_rejected_escape_may_rewind_but_moved_escape_cannot(self):
        self.s.partial['turn_escape'] = dict(done=False, path_m=0, origin=asdict(self.actual))
        with patch('turn_adjustment.run_rewind', side_effect=self.finish):
            self.s.rewind_turn()
        self.assertNotIn('turn_escape', self.s.partial)
        self.setUp()
        self.s.partial['turn_escape'] = dict(done=False, path_m=0.001, origin=asdict(self.actual))
        with self.assertRaisesRegex(ValueError, 'Escape already moved'):
            self.s.rewind_turn()

    def test_checkpoint_rejects_translation_or_different_yaw(self):
        with patch('turn_adjustment.run_rewind', side_effect=RuntimeError('blocked')):
            with self.assertRaises(RuntimeError):
                self.s.rewind_turn()
        data = {'active': [], 'partial': self.s.partial}
        validate_records(data, self.s.steps)
        state = self.s.partial['turn_rewind']
        state['target']['x'] += 0.01
        with self.assertRaisesRegex(ValueError, 'Invalid fixed rewind'):
            validate_records(data, self.s.steps)
        state['target']['x'] -= 0.01
        state['target']['yaw'] = 0.5
        with self.assertRaisesRegex(ValueError, 'Invalid fixed rewind'):
            validate_records(data, self.s.steps)

    def test_preflight_failure_does_not_launch_turn_and_cleans_up(self):
        from turn_adjustment import run_rewind
        from unittest.mock import Mock

        runner = Mock()
        runner.check_turn.side_effect = RuntimeError('sweep blocked')
        with patch('turn_recovery.supervised_turn') as turn:
            with self.assertRaisesRegex(RuntimeError, 'sweep blocked'):
                run_rewind(runner, Pose(0, 0, 0))
        turn.assert_not_called()
        runner.stop_owned.assert_called_once()
        self.assertFalse(runner.turn_guard)
