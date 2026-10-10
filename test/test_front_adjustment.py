"""Endpoint corrections survive retry without reindexing or repeating a displacement."""

from dataclasses import asdict
import copy
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose, Step
from wall_session import WallSession
from session_checkpoint import validate_records


class FrontAdjustment(unittest.TestCase):
    def setUp(self):
        self.actual = Pose(0.4, 0, 0)
        backend = SimpleNamespace(pose=lambda: self.actual, reference=0.12)
        steps = [
            Step('forward', 'forward', 0.4, {'stop': 'front', 'follow': 'left'}),
            Step('right', 'right', 0.2, {'stop': 'right'}),
        ]
        self.session = WallSession(Pose(0, 0, 0), steps, backend, lambda *args: None)
        self.session.cursor = 1
        self.session.last_actual = self.actual
        self.session.active = [
            {
                'index': 0,
                'step': asdict(steps[0]),
                'start': asdict(Pose(0, 0, 0)),
                'target': asdict(self.actual),
                'end': asdict(self.actual),
            }
        ]

    def test_interruption_blocks_other_motion_and_preserves_target(self):
        s = self.session
        original = copy.deepcopy(s.active[0])

        def fail(runner, step, state):
            self.actual = Pose(0.41, 0, 0)
            state['path_m'] = 0.01
            raise RuntimeError('obstacle')

        with patch('front_adjustment.run_adjustment', side_effect=fail):
            with self.assertRaisesRegex(RuntimeError, 'obstacle'):
                s.adjust_front(0.08)
        for operation in (
            s.next,
            s.resume,
            s.back,
            lambda: s.adjust_turn(0.02),
            lambda: s.escape_turn(0.03),
            s.rewind_turn,
        ):
            with self.assertRaisesRegex(RuntimeError, 'Finish adjust_front'):
                operation()
        with self.assertRaisesRegex(ValueError, 'fixed'):
            s.adjust_front(0.09)
        state = s.active[0]['front_adjustment']
        self.assertEqual(state['origin'], original['end'])
        self.assertEqual(state['path_m'], 0.01)
        validate_records({'active': s.active, 'partial': None}, s.steps)

        def finish(runner, step, state):
            self.assertEqual(state['path_m'], 0.01)
            self.actual = Pose(0.44, 0, 0)

        with patch('front_adjustment.run_adjustment', side_effect=finish) as run:
            s.adjust_front(0.08)
            s.adjust_front(0.08)
            self.assertEqual(run.call_count, 1)
        self.assertEqual(s.cursor, 1)
        self.assertEqual(s.goals[1].x, 0.44)
        self.assertEqual(s.active[0]['start'], original['start'])
        self.assertEqual(state['original_end'], original['end'])
        self.assertEqual(s.backend.reference, 0.12)

    def test_invalid_context_and_checkpoint(self):
        s = self.session
        for value in (float('nan'), 0.01, 0.2):
            with self.assertRaises(ValueError):
                s.adjust_front(value)
        with patch('front_adjustment.run_adjustment'):
            s.adjust_front(0.08)
        data = {'active': copy.deepcopy(s.active), 'partial': None}
        data['active'][0]['front_adjustment']['clearance'] = 0.01
        with self.assertRaisesRegex(ValueError, 'Invalid front'):
            validate_records(data, s.steps)
        s.partial = {'index': 1}
        with self.assertRaisesRegex(ValueError, 'no partial'):
            s.adjust_front(0.08)

    def test_runtime_failure_restores_speed_and_retains_budget(self):
        from front_adjustment import run_adjustment

        def fail(step, target):
            backend.last_report = {'path_m': 0.03}
            backend.path_last_pose = Pose(0.425, 0, 0)
            raise RuntimeError('guard rejected')

        backend = SimpleNamespace(
            max_speed=0.06,
            execute=fail,
            pose=lambda: Pose(0.43, 0, 0),
            path_last_pose=None,
            last_report=None,
        )
        state = {
            'origin': asdict(Pose(0.4, 0, 0)),
            'original_target': asdict(Pose(0.4, 0, 0)),
            'clearance': 0.08,
            'path_m': 0.01,
        }
        with self.assertRaisesRegex(RuntimeError, 'guard rejected'):
            run_adjustment(backend, self.session.steps[0], state)
        self.assertAlmostEqual(state['path_m'], 0.035)
        self.assertEqual(backend.max_speed, 0.06)
        self.assertIsNone(backend.resume_origin)
        self.assertIsNone(backend.endpoint_adjustment_origin)
