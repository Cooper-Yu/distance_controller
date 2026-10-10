"""Preparation wall selection is independent from per-action following and capture."""

import contextlib
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import patch, Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import action_session
from action_runtime import PlannedRunner
from wall_runtime import WallRunner

ROOT = Path(__file__).resolve().parents[1]


class AlignmentSelection(unittest.TestCase):
    def invoke(self, options, route='task6_wall_actions.json'):
        with (
            patch.object(
                sys, 'argv', ['action_session', '--route', str(ROOT / 'config' / route), *options]
            ),
            patch.object(action_session, 'run_session', return_value=0) as run,
        ):
            result = action_session.main()
            return result, run.call_args

    def test_default_and_explicit_walls_do_not_change_follow_policy(self):
        for options, side in [
            ([], 'right'),
            (['--alignment-wall', 'left'], 'left'),
            (['--alignment-wall', 'right'], 'right'),
        ]:
            result, call = self.invoke(['--start', 'prepare', *options])
            self.assertEqual(result, 0)
            self.assertEqual(call.args[3], side)
            self.assertEqual(call.args[0][0].wall['follow'], 'left')

    def test_current_and_legacy_reject_preparation_only_option(self):
        for start, route in [
            ('current', 'task6_wall_actions.json'),
            ('prepare', 'task6_actions.json'),
        ]:
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                self.invoke(['--start', start, '--alignment-wall', 'right'], route)
            self.assertEqual(error.exception.code, 2)

    def test_selected_wall_reaches_child_parameter(self):
        with (
            patch.object(PlannedRunner, '__init__', return_value=None),
            patch.object(WallRunner, 'node', Mock(), create=True),
            patch.object(WallRunner, 'twist', Mock(), create=True),
        ):
            for side in ('left', 'right'):
                runner = WallRunner(alignment_wall=side)
                self.assertIn('alignment_wall:=' + side, runner.prepare_options)
            with self.assertRaises(ValueError):
                WallRunner(alignment_wall='front')


if __name__ == '__main__':
    unittest.main()
