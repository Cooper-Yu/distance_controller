"""Regression for paused preparation with four pending default segments."""

import sys
from pathlib import Path
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_runtime import PlannedRunner


class PreparationTest(unittest.TestCase):
    def runner(self, messages):
        runner = object.__new__(PlannedRunner)
        runner.child = Mock()
        runner.child.poll.return_value = None
        runner.client = Mock()
        runner.client.service_is_ready.return_value = True
        runner.pump = Mock()
        runner.call = Mock(side_effect=messages)
        return runner

    def test_preparation_accepts_paused_default_route(self):
        message = 'state=WAITING waypoint=A pending=4 return_remaining=0'
        runner = self.runner([message])
        self.assertEqual(runner.wait_idle(1, preparation=True), message)

    def test_normal_action_still_requires_empty_queue(self):
        busy = 'state=WAITING waypoint=A pending=4 return_remaining=0'
        done = 'state=WAITING waypoint=session_target pending=0 return_remaining=0'
        runner = self.runner([busy, done])
        self.assertEqual(runner.wait_idle(1, 'session_target'), done)
        self.assertEqual(runner.call.call_count, 2)

    def test_preparation_fault_is_not_accepted(self):
        runner = self.runner(['state=FAULT waypoint=A pending=4 return_remaining=0'])
        with self.assertRaises(RuntimeError):
            runner.wait_idle(1, preparation=True)

    def test_prepare_handoff_finishes_without_starting_route(self):
        runner = self.runner(
            ['state=WAITING waypoint=A pending=4 return_remaining=0', 'Shutdown requested']
        )
        runner.start = Mock()
        runner.wait_exit = Mock()
        runner.prepare_start()
        self.assertEqual([c.args[0] for c in runner.call.call_args_list], ['status', 'finish'])
        runner.wait_exit.assert_called_once_with(10)
        self.assertIsNone(runner.child)


if __name__ == '__main__':
    unittest.main()
