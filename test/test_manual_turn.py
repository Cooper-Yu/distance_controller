"""One-shot visual supervision must not leak into returns or later actions."""

import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose, Step
from wall_session import WallSession
from wall_runtime import WallRunner, TurnClearanceError, ReturnClearanceError
from action_runtime import PlannedRunner


class ManualTurn(unittest.TestCase):
    def session(self):
        backend = SimpleNamespace(pose=lambda: Pose(0, 0, 0))
        return WallSession(Pose(0, 0, 0), [Step('turn', 'turn', -0.5)], backend, Mock())

    def test_scope_cleared_after_success_failure_and_interrupt(self):
        for failure in (None, RuntimeError('stale'), KeyboardInterrupt()):
            s = self.session()

            def execute():
                self.assertTrue(s.backend.manual_turn_active)
                if failure is not None:
                    raise failure
                return Pose(0, 0, -0.5)

            with patch.object(s, 'next', side_effect=execute):
                if failure is None:
                    s.manual_turn()
                else:
                    with self.assertRaises(type(failure)):
                        s.manual_turn()
            self.assertFalse(s.backend.manual_turn_active)

    def test_translation_and_return_rejected(self):
        s = self.session()
        s.steps = [Step('forward', 'forward', 0.1)]
        with self.assertRaises(ValueError):
            s.manual_turn()
        s = self.session()
        s.partial = {'step': {'kind': 'turn'}, 'return_started': True}
        with self.assertRaises(ValueError):
            s.manual_turn()

    def test_partial_uses_resume_and_pending_adjustment_still_blocks(self):
        s = self.session()
        s.partial = {'step': {'kind': 'turn'}, 'turn_adjustment': {'done': False}}
        with self.assertRaisesRegex(RuntimeError, 'adjust_turn'):
            s.manual_turn()
        self.assertFalse(s.backend.manual_turn_active)
        s.partial = {'step': {'kind': 'turn'}}
        with patch.object(s, 'resume', return_value=Pose(0, 0, -0.5)) as resume:
            s.manual_turn()
            resume.assert_called_once()

    def runner(self, manual, sign=1):
        r = object.__new__(WallRunner)
        r.refresh_callbacks = Mock()
        r.moving_pose = Mock(return_value=Pose(0, 0, 0))
        r.geometry = Mock(return_value=([(0.18, 0, 0)], {}))
        r.return_scenarios = Mock(return_value=[(0, 0, 0.08)])
        r.turn_guard, r.turn_sign, r.turn_speed = True, sign, 0.08
        r.manual_turn_active = manual
        r.cached_stamp, r.observations = 1, []
        r.latest = (Pose(0, 0, 0), 0, 0, 0)
        return r

    def test_near_points_log_only_but_default_and_return_still_stop(self):
        with patch.object(PlannedRunner, 'pump'):
            r = self.runner(True)
            r.pump()
            self.assertEqual(len(r.observations), 1)
            self.assertFalse(r.observations[0]['manual_turn_clearance']['enforced'])
            with self.assertRaises(TurnClearanceError):
                self.runner(False).pump()
            with self.assertRaises(ReturnClearanceError):
                self.runner(True, 0).pump()

    def test_stale_feedback_still_fails(self):
        with patch.object(PlannedRunner, 'pump'):
            for source in ('geometry', 'moving_pose'):
                r = self.runner(True)
                getattr(r, source).side_effect = RuntimeError('stale')
                with self.assertRaisesRegex(RuntimeError, 'stale'):
                    r.pump()
