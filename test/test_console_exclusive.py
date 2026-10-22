"""A completed entry gate removes only the first redundant discovery wait."""

from types import SimpleNamespace
from unittest.mock import Mock
import unittest

import test_console_settling as settling
from action_plan import Pose
from motion_model import Motion
from survey_return import Runner


class ExclusiveTest(unittest.TestCase):
    def console(self, poses=None):
        return settling.SettlingTests().make_console(poses or [Pose(0.1, 0, 0)])

    def test_direct_finish_retains_wait_and_final_check(self):
        console = self.console()
        console.finish_motion(Pose(0, 0, 0), Motion('move', x=0.1), 30)
        console.runner.exclusive.assert_called_once()
        console.runner.assert_exclusive.assert_called_once()
        self.assertIn('publisher_created', console.audit.getvalue())

    def test_entry_gate_reused_only_for_attempt_zero(self):
        console = self.console([Pose(0.1, 0, 0.012), Pose(0.1, 0, 0)])
        console.finish_motion(Pose(0, 0, 0), Motion('move', x=0.1), 30, initial_exclusive_done=True)
        console.runner.exclusive.assert_called_once()
        self.assertEqual(console.runner.assert_exclusive.call_count, 2)

    def test_move_keeps_initial_wait_before_frozen_pose(self):
        console = self.console([Pose(0, 0, 0), Pose(0.1, 0, 0)])
        console.runner.frame, console.runner.twist_frame = 'odom', 'base_link'
        console.measure = Mock()
        calls = []
        console.runner.exclusive.side_effect = lambda: calls.append('wait')
        poses = iter([Pose(0, 0, 0), Pose(0.1, 0, 0)])
        console.runner.pose.side_effect = lambda: calls.append('pose') or next(poses)
        console.move(Motion('move', x=0.1))
        self.assertEqual(calls, ['wait', 'pose', 'pose'])
        console.runner.assert_exclusive.assert_called_once()

    def test_late_competitor_prevents_publisher_and_follow(self):
        for kind in ('publisher', 'service'):
            with self.subTest(kind=kind):
                console = self.console()
                node = console.runner.node
                node.get_publishers_info_by_topic = Mock(
                    return_value=[object()] if kind == 'publisher' else []
                )
                console.runner.client = SimpleNamespace(service_is_ready=lambda: kind == 'service')
                console.runner.assert_exclusive = lambda: Runner.assert_exclusive(console.runner)
                with self.assertRaisesRegex(RuntimeError, 'Existing'):
                    console.finish_motion(
                        Pose(0, 0, 0), Motion('move', x=0.1), 30, initial_exclusive_done=True
                    )
                node.create_publisher.assert_not_called()
                console.follow.assert_not_called()

    def test_cancel_and_audit_failure_do_not_leak_publisher(self):
        console = self.console()
        console.cancel_input.side_effect = [None, KeyboardInterrupt()]
        with self.assertRaises(KeyboardInterrupt):
            console.finish_motion(
                Pose(0, 0, 0), Motion('move', x=0.1), 30, initial_exclusive_done=True
            )
        console.runner.node.create_publisher.assert_not_called()
        console = self.console()
        console.emit = Mock(side_effect=OSError('audit failed'))
        with self.assertRaisesRegex(OSError, 'audit failed'):
            console.finish_motion(
                Pose(0, 0, 0), Motion('move', x=0.1), 30, initial_exclusive_done=True
            )
        console.stop_motion.assert_called_once()
        console.follow.assert_not_called()


if __name__ == '__main__':
    unittest.main()
