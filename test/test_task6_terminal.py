"""Directed terminal turn, fail-before-move clearance, and manifest contracts."""

from copy import deepcopy
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from route_laser_guard import RADIUS
from task6_route_model import RouteAction, validate_manifest
from task6_route_runner import run_actions
from task6_terminal import TERMINAL_NAME, observed_turn_gap, terminal_guard, turn_motion


def manifest():
    return dict(
        schema=1,
        scene=2,
        frame='P01',
        actions=[
            dict(name='P15', kind='path', heading_rad=0.0, points=[[0.0, 0.0], [0.04, 0.0]]),
            dict(name=TERMINAL_NAME, kind='turn', heading_rad=math.pi),
        ],
    )


class TerminalTests(unittest.TestCase):
    def action(self):
        return RouteAction(TERMINAL_NAME, 'turn', 0.0)

    def console(self):
        console = Mock()
        console.runner.cached_stamp = 123
        console.book.origin = Pose(0, 0, 0)
        console.runner.pose.return_value = Pose(0.04, 0, -math.pi)
        console.command_guard = Mock(name='prior_guard')
        return console

    def guard(self, gap):
        guard = Mock()
        guard.policy.hard_m = 0.02
        guard.snapshot.return_value = SimpleNamespace(points=((RADIUS + gap, 0.0),))
        return guard

    def test_terminal_keeps_positive_pi_for_both_sides_of_wrap(self):
        for yaw in (-math.pi - 0.001, -math.pi + 0.001, math.pi - 0.001, math.pi + 0.001):
            with self.subTest(yaw=yaw):
                motion = turn_motion(self.action(), Pose(0, 0, 0.3), Pose(0.04, 0, yaw))
                self.assertEqual((motion.kind, motion.angle), ('turn', math.pi))

    def test_normal_turn_preserves_shortest_signed_correction(self):
        action = RouteAction('P14_after_turn', 'turn', -math.pi / 2)
        self.assertAlmostEqual(
            turn_motion(action, Pose(0, 0, 0), Pose(0, 0, -math.pi)).angle, math.pi / 2
        )
        self.assertAlmostEqual(
            turn_motion(action, Pose(0, 0, 0), Pose(0, 0, 0)).angle, -math.pi / 2
        )

    def test_observed_gap_is_conservative_disk_distance(self):
        self.assertAlmostEqual(observed_turn_gap(((RADIUS + 0.029, 0), (1.0, 1.0))), 0.029)
        for points in ((), ((float('nan'), 0),), ((0, float('inf')),)):
            with self.subTest(points=points), self.assertRaises(RuntimeError):
                observed_turn_gap(points)

    def test_static_gap_rejects_before_console_move(self):
        console = self.console()
        previous = console.command_guard
        guard = self.guard(0.019)
        with patch('task6_terminal.RouteLaserGuard', return_value=guard):
            with self.assertRaisesRegex(RuntimeError, '2 cm'):
                run_actions(console, (self.action(),), Mock())
        guard.snapshot.assert_called_once_with()
        console.move.assert_not_called()
        console.runner.node.create_publisher.assert_not_called()
        console.record.assert_not_called()
        self.assertIs(console.command_guard, previous)

    def test_exact_twenty_mm_boundary_accepts_and_restores_on_cancel(self):
        console = self.console()
        previous = console.command_guard
        guard = self.guard(0.02)
        # Isolate the exact comparison from float subtraction in point geometry.
        with (
            patch('task6_terminal.RouteLaserGuard', return_value=guard),
            patch('task6_terminal.observed_turn_gap', return_value=0.02),
        ):
            with self.assertRaises(KeyboardInterrupt):
                with terminal_guard(console, self.action()):
                    self.assertIs(console.command_guard, guard)
                    console.command_guard((0.0, 0.0, 0.1))
                    raise KeyboardInterrupt
        guard.assert_called_once_with((0.0, 0.0, 0.1))
        self.assertIs(console.command_guard, previous)

    def test_terminal_run_records_only_after_guarded_move(self):
        console = self.console()
        previous = console.command_guard
        guard = self.guard(0.03)

        def move(motion):
            self.assertIs(console.command_guard, guard)
            self.assertEqual(motion.angle, math.pi)

        console.move.side_effect = move
        with patch('task6_terminal.RouteLaserGuard', return_value=guard):
            run_actions(console, (self.action(),), Mock())
        console.record.assert_called_once_with(TERMINAL_NAME)
        self.assertIs(console.command_guard, previous)

    def test_nonterminal_does_not_construct_guard(self):
        console = self.console()
        previous = console.command_guard
        with patch('task6_terminal.RouteLaserGuard') as factory:
            with terminal_guard(console, RouteAction('P14_after_turn', 'turn', 0)):
                self.assertIs(console.command_guard, previous)
        factory.assert_not_called()

    def test_valid_terminal_manifest(self):
        self.assertEqual(validate_manifest(manifest())[-1].name, TERMINAL_NAME)

    def test_malformed_terminal_manifest_is_rejected(self):
        mutations = [
            lambda d: d['actions'][0].update(name='P14'),
            lambda d: d['actions'][1].update(heading_rad=math.pi / 2),
            lambda d: d['actions'].append(dict(name='extra', kind='turn', heading_rad=0)),
            lambda d: d['actions'].insert(1, dict(name='interposed', kind='turn', heading_rad=0)),
            lambda d: d['actions'][1].update(
                kind='path', heading_rad=0, points=[[0.04, 0], [0.08, 0]]
            ),
        ]
        for mutate in mutations:
            data = deepcopy(manifest())
            mutate(data)
            with self.subTest(data=data), self.assertRaises(ValueError):
                validate_manifest(data)


if __name__ == '__main__':
    unittest.main()
