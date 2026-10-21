"""Operator command semantics without ROS or old-session loading."""

from dataclasses import asdict
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from motion_model import target_pose
from route_session_runtime import run_supervised
from task6_route_model import RouteAction


class Input:
    def __init__(self, commands, events):
        self.commands, self.events = list(commands), events

    def poll(self):
        command = self.commands.pop(0) if self.commands else None
        if command == 'confirm':
            plan = next(
                e['plan'] for e in reversed(self.events) if e['event'] == 'supervised_return_plan'
            )
            command = 'back ' + plan['token']
        return True, command


class RuntimeTest(unittest.TestCase):
    def setup_console(self):
        origin = Pose(0, 0, 0)
        runner = SimpleNamespace(pump=Mock(), invalid=None, current=origin)
        runner.pose = lambda: runner.current
        book = SimpleNamespace(origin=origin, data={'session_id': 'live', 'points': {}})
        events = []
        console = SimpleNamespace(
            runner=runner, book=book, command_guard=SimpleNamespace(enforced=True)
        )
        console.emit = lambda event, **data: events.append(dict(event=event, **data))
        console.move = lambda motion, **_kwargs: setattr(
            runner, 'current', target_pose(origin, motion)
        )
        console.record = lambda name: book.data['points'].update({name: asdict(runner.current)})
        actions = (RouteAction('P02', 'path', 0, ((0, 0), (0.04, 0))),)
        return console, actions, events

    def test_roundtrip_reexecute_and_prefix_gate(self):
        console, actions, events = self.setup_console()
        inputs = Input(['next', 'back_plan', 'confirm', 'next', 'next', 'quit'], events)
        state = run_supervised(console, actions, Mock(), inputs)
        self.assertEqual(state.cursor, 1)
        self.assertEqual(list(console.book.data['points']), ['P02_visit_1', 'P02_visit_2'])
        self.assertTrue(any(e['event'] == 'supervised_command_rejected' for e in events))
        self.assertFalse(any(e['event'] == 'route_completed' for e in events))
        self.assertEqual(state.origin, Pose(0, 0, 0))

    def test_shadow_rejected_before_any_motion(self):
        console, actions, events = self.setup_console()
        console.command_guard.enforced = False
        with self.assertRaisesRegex(RuntimeError, 'enforced'):
            run_supervised(console, actions, Mock(), Input(['next'], events))
        console.runner.pump.assert_not_called()

    def test_motion_fault_exits_without_next_or_return(self):
        console, actions, events = self.setup_console()
        console.move = Mock(side_effect=RuntimeError('Odom stale'))
        with self.assertRaisesRegex(RuntimeError, 'Odom stale'):
            run_supervised(console, actions, Mock(), Input(['next', 'next'], events))
        console.move.assert_called_once()
        self.assertEqual(events[-1]['event'], 'supervised_session_failed')
        self.assertEqual(events[-1]['cursor'], 0)

    def test_return_budget_uses_length_not_zero_endpoint(self):
        import io
        from motion_console import Console
        from motion_model import Motion

        runner = SimpleNamespace(
            exclusive=Mock(),
            pose=lambda: Pose(0.8, 0, 0),
            frame='odom',
            twist_frame='base_link',
            max_speed=0.03,
        )
        console = Console(runner, SimpleNamespace(origin=Pose(0, 0, 0)), Mock(), io.StringIO(), 0.1)
        console.cancel_input, console.stop_motion, console.measure = Mock(), Mock(), Mock()
        console.finish_motion = Mock(return_value=Pose(0, 0, 0))
        motion = Motion('path', points=((0.8, 0), (0, 0)))
        console.move(motion, use_path_length=True)
        self.assertAlmostEqual(console.finish_motion.call_args.args[2], 110.0)
        console.move(motion)
        self.assertAlmostEqual(console.finish_motion.call_args.args[2], 30.0)

    def test_eof_closes_without_route_success(self):
        console, actions, events = self.setup_console()
        state = run_supervised(console, actions, Mock(), Input([], events))
        self.assertEqual(state.cursor, 0)
        self.assertFalse(events[-1]['prefix_complete'])


if __name__ == '__main__':
    unittest.main()
