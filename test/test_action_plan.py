"""Plan/history regression tests with an imperfect deterministic backend."""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose, Session, Step, load_steps, reverse_steps, targets


class Backend:
    def __init__(self):
        self.actual = Pose(1, 2, 0)
        self.fail = False
        self.offset = 0.005
        self.commands = []

    def pose(self):
        return self.actual

    def execute(self, step, target):
        self.commands.append((step, target))
        if self.fail:
            self.actual = Pose((self.actual.x + target.x) / 2, target.y, target.yaw)
            raise RuntimeError('partial test')
        self.actual = Pose(target.x - self.offset, target.y, target.yaw)


class PlanTest(unittest.TestCase):
    def setUp(self):
        self.backend = Backend()
        self.events = []
        self.steps = [
            Step('a', 'forward', 0.1),
            Step('b', 'turn', math.pi / 2),
            Step('c', 'left', 0.1),
        ]
        self.session = Session(
            self.backend.pose(), self.steps, self.backend, lambda *event: self.events.append(event)
        )

    def test_reverse_uses_endpoint_frame(self):
        origin = Pose(1, 2, 0.4)
        end = targets(origin, self.steps)[-1]
        actual = targets(end, reverse_steps(self.steps))[-1]
        self.assertAlmostEqual(actual.x, origin.x)
        self.assertAlmostEqual(actual.y, origin.y)
        self.assertAlmostEqual(actual.yaw, origin.yaw)

    def test_targets_use_planned_not_stopping_error(self):
        self.session.next()
        self.session.next()
        self.assertAlmostEqual(self.backend.commands[1][1].x, 1.1)
        self.assertAlmostEqual(self.backend.actual.x, 1.095)

    def test_edit_moves_all_downstream_targets(self):
        self.session.edit(0, 0.2)
        self.assertAlmostEqual(self.session.goals[1].x, 1.2)
        self.assertAlmostEqual(self.session.goals[2].x, 1.1)

    def test_executed_edit_rejected_until_return(self):
        self.session.next()
        with self.assertRaises(ValueError):
            self.session.edit(0, 0.2)
        self.session.back()
        self.session.edit(0, 0.2)
        self.session.next()
        self.assertAlmostEqual(self.backend.commands[-1][1].x, 1.2)
        self.assertEqual(sum(e[0] == 'completed' for e in self.events), 2)

    def test_return_uses_actual_start_and_preserves_nominal_origin(self):
        self.session.next()
        recorded = self.backend.actual
        self.session.next()
        self.session.back()
        self.assertEqual(self.backend.commands[-1][1], recorded)
        self.assertEqual(self.session.origin, Pose(1, 2, 0))
        self.assertEqual(self.session.cursor, 1)

    def test_partial_requires_recovery_before_retry(self):
        self.backend.fail = True
        with self.assertRaises(RuntimeError):
            self.session.next()
        self.assertEqual(self.session.cursor, 0)
        with self.assertRaises(RuntimeError):
            self.session.next()
        with self.assertRaises(ValueError):
            self.session.edit(0, 0.2)
        self.backend.fail = False
        self.session.back()
        self.assertEqual(self.backend.commands[-1][1], Pose(1, 2, 0))
        self.session.edit(0, 0.2)

    def test_failed_return_can_retry_same_anchor(self):
        self.session.next()
        self.backend.fail = True
        with self.assertRaises(RuntimeError):
            self.session.back()
        self.backend.fail = False
        self.session.back()
        self.assertEqual(self.session.cursor, 0)
        self.assertEqual(self.session.active, [])
        self.assertEqual(self.backend.commands[-1][1], Pose(1, 2, 0))

    def test_waiting_relocation_rejected_without_motion(self):
        self.backend.actual = Pose(0, 0, 0)
        with self.assertRaises(RuntimeError):
            self.session.next()
        self.assertFalse(self.backend.commands)

    def test_reload_keeps_executed_prefix(self):
        self.session.next()
        with self.assertRaises(ValueError):
            self.session.reload([Step('a', 'forward', 0.2)])
        self.session.reload([self.steps[0], Step('new', 'right', 0.3)])
        self.assertAlmostEqual(self.session.goals[-1].y, 1.7)

    def test_explicit_units_and_invalid_numbers(self):
        result = load_steps({'actions': [dict(name='turn', kind='turn', value=-90, unit='deg')]})
        self.assertAlmostEqual(result[0].value, -math.pi / 2)
        for value in [0, -1, math.nan, math.inf]:
            with self.assertRaises(ValueError):
                Step('bad', 'forward', value).validate()

    def test_turn_and_translation_are_independent(self):
        p = targets(Pose(1, 2, 0.3), [Step('turn', 'turn', math.pi), Step('go', 'forward', 0.2)])
        self.assertEqual((p[0].x, p[0].y), (1, 2))
        self.assertEqual(p[1].yaw, p[0].yaw)


if __name__ == '__main__':
    unittest.main()
