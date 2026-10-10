"""Micro-adjustments keep fixed targets, reject turn-in-progress and preserve checkpoints."""

from dataclasses import asdict
import math
import json
import tempfile
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose, Step
from wall_session import WallSession
from session_checkpoint import validate_records
from turn_adjustment import adjustment_command
from session_audit import import_audit


class Adjustment(unittest.TestCase):
    def test_audit_requires_checkpoint_after_adjustment(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'audit.jsonl'
            rows = [
                {'event': 'origin', 'data': {'pose': {'x': 0, 'y': 0, 'yaw': 0}, 'actions': []}},
                {'event': 'turn_adjustment_started', 'data': {}},
            ]
            path.write_text('\n'.join(json.dumps(row) for row in rows))
            with self.assertRaisesRegex(ValueError, 'requires its checkpoint'):
                import_audit(path)

    def setup_session(self):
        self.pose = Pose(0, 0, 0)
        backend = SimpleNamespace(pose=lambda: self.pose, check_turn=lambda target: None)
        step = Step('turn', 'turn', -math.pi / 2, {'capture': 'left'})
        self.session = WallSession(self.pose, [step], backend, lambda *args: None)
        self.session.partial = {
            'index': 0,
            'revision': 0,
            'step': asdict(step),
            'start': asdict(self.pose),
            'target': asdict(Pose(0, 0, -math.pi / 2)),
        }
        return self.session

    def test_failure_retry_is_same_target_and_blocks_rotation(self):
        s = self.setup_session()
        seen = []

        def fail(backend, state):
            seen.append(state['target'].copy())
            self.pose = Pose(0, -0.008, 0)
            state['path_m'] = 0.008
            raise RuntimeError('obstacle')

        with patch('turn_adjustment.run_adjustment', side_effect=fail):
            with self.assertRaises(RuntimeError):
                s.adjust_turn(0.02)
        with self.assertRaises(RuntimeError):
            s.resume()
        with self.assertRaises(ValueError):
            s.adjust_turn(0.03)

        def succeed(backend, state):
            seen.append(state['target'].copy())
            self.assertEqual(state['path_m'], 0.008)
            self.pose = Pose(**state['target'])
            state.update(done=True, path_m=0.02)

        with patch('turn_adjustment.run_adjustment', side_effect=succeed) as move:
            s.adjust_turn(0.02)
            s.adjust_turn(0.02)
            self.assertEqual(move.call_count, 1)
        self.assertEqual(seen[0], seen[1])
        self.assertEqual(s.cursor, 0)
        self.assertEqual(s.partial['start'], asdict(Pose(0, 0, 0)))
        self.assertEqual(s.partial['target'], asdict(Pose(0, -0.02, -math.pi / 2)))
        validate_records({'active': [], 'partial': s.partial}, s.steps)
        s.partial['turn_adjustment']['target']['y'] = -0.03
        with self.assertRaises(ValueError):
            validate_records({'active': [], 'partial': s.partial}, s.steps)

    def test_final_check_failure_does_not_repeat_translation(self):
        s = self.setup_session()

        def move(backend, state):
            self.pose = Pose(**state['target'])
            state['done'] = True

        s.backend.check_turn = lambda t: (_ for _ in ()).throw(RuntimeError('sweep rejected'))
        with patch('turn_adjustment.run_adjustment', side_effect=move) as motion:
            for _ in range(2):
                with self.assertRaises(RuntimeError):
                    s.adjust_turn(0.02)
            self.assertEqual(motion.call_count, 1)
        self.assertTrue(s.partial['turn_adjustment']['done'])
        self.assertEqual(s.last_actual, self.pose)

    def test_bounds_and_started_turn(self):
        s = self.setup_session()
        for value in (0, 0.041, float('nan')):
            with self.assertRaises(ValueError):
                s.adjust_turn(value)
        self.pose = Pose(0, 0, 0.04)
        s.last_actual = self.pose
        with self.assertRaises(ValueError):
            s.adjust_turn(0.02)
        self.assertNotIn('turn_adjustment', s.partial)

    def test_velocity_cap_and_heading(self):
        velocity, ready = adjustment_command(Pose(0, 0, 0), Pose(0, -0.02, 0))
        self.assertEqual(velocity, (0, -0.01, 0))
        self.assertFalse(ready)
        velocity, _ = adjustment_command(Pose(0, 0, 0), Pose(0.02, -0.02, 0.1))
        self.assertEqual(velocity, (0, 0, 0.08))
        self.assertTrue(adjustment_command(Pose(0, -0.019, 0), Pose(0, -0.02, 0))[1])

    def test_four_cm_before_pending_turn_and_repeat_after_restore(self):
        s = self.setup_session()
        s.partial = None
        s.backend.reference = 0.10

        def move(backend, state):
            self.pose = Pose(**state['target'])
            state.update(done=True, path_m=0.04)

        with patch('turn_adjustment.run_adjustment', side_effect=move) as motion:
            s.adjust_turn(0.04)
            # JSON round-trip mirrors persistence of the fixed target.
            s.partial = json.loads(json.dumps(s.partial))
            s.adjust_turn(0.04)
            self.assertEqual(motion.call_count, 1)
        self.assertEqual(self.pose, Pose(0, -0.04, 0))
        self.assertEqual(s.cursor, 0)
        self.assertAlmostEqual(s.partial['target']['yaw'], -math.pi / 2)
        validate_records({'active': [], 'partial': s.partial}, s.steps)
        with self.assertRaises(ValueError):
            s.adjust_turn(0.02)

    def test_escape_limit_remains_three_cm(self):
        s = self.setup_session()
        with self.assertRaisesRegex(ValueError, 'exactly 0.03'):
            s.escape_turn(0.04)

    def test_existing_over_budget_path_can_settle_but_escape_cannot(self):
        from turn_adjustment import guarded_adjustment
        from unittest.mock import Mock
        import copy

        state = dict(
            origin=asdict(Pose(0, 0, 0)),
            target=asdict(Pose(0, -0.04, 0)),
            path_m=0.066304,
            done=False,
        )
        runner = Mock()
        runner.pose.return_value = runner.moving_pose.return_value = Pose(0, -0.04, 0)
        runner.latest = (None, None, 0, 0)
        stamps = iter(range(50))
        runner.pump.side_effect = lambda: setattr(runner, 'cached_stamp', next(stamps))
        clock = iter(i * 0.1 for i in range(100))
        with (
            patch('turn_adjustment.time.monotonic', side_effect=lambda: next(clock)),
            patch('turn_adjustment.guard_normal'),
        ):
            guarded_adjustment(runner, state)
        self.assertGreater(state['path_m'], 0.06)
        escape = copy.deepcopy(state)
        escape['escape'] = True
        with self.assertRaisesRegex(RuntimeError, 'escape exceeded'):
            guarded_adjustment(runner, escape)

    def test_over_budget_path_does_not_fail_final_endpoint_check(self):
        from turn_adjustment import run_adjustment
        from unittest.mock import Mock

        state = dict(
            origin=asdict(Pose(0, 0, 0)),
            target=asdict(Pose(0, -0.04, 0)),
            path_m=0.066304,
            done=False,
        )
        runner = Mock()
        runner.pose.return_value = Pose(0, -0.04, 0)
        with patch('turn_adjustment.guarded_adjustment'):
            run_adjustment(runner, state)
        self.assertTrue(state['done'])
        self.assertEqual(state['path_m'], 0.066304)
