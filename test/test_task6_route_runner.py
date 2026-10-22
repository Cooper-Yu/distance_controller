"""Non-ROS manifest, execution, cancellation and exit-status contracts."""

from copy import deepcopy
import io
import json
import math
from pathlib import Path
import signal
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from task6_route_model import reconnect_route, validate_manifest
from teaching_path import PathTracker
import task6_route_runner as route


def manifest():
    return dict(
        schema=1,
        scene=2,
        frame='P01',
        actions=[
            dict(name='P02', kind='path', heading_rad=0.0, points=[[0.0, 0.0], [0.04, 0.0]]),
            dict(name='P02_after_turn', kind='turn', heading_rad=math.pi / 2),
            dict(
                name='P03',
                kind='path',
                heading_rad=math.pi / 2,
                points=[[0.04, 0.0], [0.045, 0.03], [0.04, 0.06]],
            ),
        ],
    )


class ManifestTest(unittest.TestCase):
    def test_retains_every_supplied_point(self):
        data = manifest()
        actions = validate_manifest(data)
        self.assertEqual(actions[2].points, tuple(map(tuple, data['actions'][2]['points'])))
        self.assertEqual(actions[-1].path_motion().angle, math.pi / 2)

    def test_invalid_manifest_boundaries(self):
        mutations = [
            lambda d: d.update(schema=True),
            lambda d: d.update(scene=1),
            lambda d: d.update(frame='odom'),
            lambda d: d.update(extra='ignored'),
            lambda d: d.update(actions=[]),
            lambda d: d['actions'][0].update(name='P01'),
            lambda d: d['actions'][1].update(name='P02'),
            lambda d: d['actions'][0].update(heading_rad=float('nan')),
            lambda d: d['actions'][0].update(heading_rad=True),
            lambda d: d['actions'][0].update(points=[[0, 0], [0.06, 0]]),
            lambda d: d['actions'][0].update(points=[[0, 0], [0, 0]]),
            lambda d: d['actions'][0].update(points=[[0.01, 0], [0.04, 0]]),
            lambda d: d['actions'][2].update(heading_rad=0),
            lambda d: d['actions'][2].update(points=[[0.041, 0], [0.04, 0.03]]),
            lambda d: d.update(actions=d['actions'][1:]),
        ]
        for mutation in mutations:
            data = deepcopy(manifest())
            mutation(data)
            with self.subTest(data=data), self.assertRaises(ValueError):
                validate_manifest(data)

    def test_reconnect_preserves_bend_and_endpoint(self):
        motion = validate_manifest(manifest())[2].path_motion()
        result = reconnect_route(motion, Pose(0, 0, 0), Pose(0.041, 0.002, math.pi / 2))
        self.assertEqual(result.points[1:], motion.points[1:])
        self.assertEqual(result.points[0], (0.041, 0.002))
        self.assertEqual((result.x, result.y, result.angle), (motion.x, motion.y, motion.angle))

    def test_reconnect_rejects_position_and_heading_drift(self):
        motion = validate_manifest(manifest())[2].path_motion()
        for actual in (Pose(0.061, 0, math.pi / 2), Pose(0.04, 0, math.pi / 2 + 0.02)):
            with self.subTest(actual=actual), self.assertRaises(RuntimeError):
                reconnect_route(motion, Pose(0, 0, 0), actual)

    def test_reconnect_resampling_retains_suffix_and_same_polyline_projection(self):
        data = manifest()
        data['actions'] = [
            dict(
                name='P02',
                kind='path',
                heading_rad=0.0,
                points=[[0.0, 0.0], [0.05, 0.0], [0.05, 0.04]],
            )
        ]
        motion = validate_manifest(data)[0].path_motion()
        origin, actual = Pose(0, 0, 0), Pose(-0.019, 0, 0)
        result = reconnect_route(motion, origin, actual)
        self.assertEqual(result.points[-len(motion.points[1:]) :], motion.points[1:])
        self.assertEqual(result.points[0], (actual.x, actual.y))
        self.assertGreater(len(result.points), len(motion.points))
        self.assertTrue(
            all(
                0 < math.dist(a, b) <= 0.05 + 1e-12
                for a, b in zip(result.points, result.points[1:])
            )
        )
        coarse = ((actual.x, actual.y), *motion.points[1:])
        for position in (
            (-0.03, 0.01),
            (-0.001, 0.012),
            (0.04, 0.008),
            (0.055, 0.025),
            (0.06, 0.06),
        ):
            before, after = PathTracker(origin, coarse), PathTracker(origin, result.points)
            with self.subTest(position=position):
                self.assertAlmostEqual(before.project(position), after.project(position), places=12)
                self.assertAlmostEqual(before.progress, after.progress, places=12)
                self.assertAlmostEqual(before.arc[-1], after.arc[-1], places=12)
                self.assertAlmostEqual(before.ahead()[0], after.ahead()[0], places=12)
                self.assertAlmostEqual(before.ahead()[1], after.ahead()[1], places=12)


class ExecutionTest(unittest.TestCase):
    def console(self):
        console = Mock()
        console.book.origin = Pose(1, 2, 0.3)
        console.runner.pose.return_value = Pose(1, 2, 0.3)
        return console

    def test_supplied_actions_and_absolute_heading_used(self):
        console = self.console()
        actions = validate_manifest(manifest())[:2]
        route.run_actions(console, actions, Mock())
        calls = console.move.call_args_list
        self.assertEqual(calls[0].args[0].points, actions[0].points)
        self.assertAlmostEqual(calls[1].args[0].angle, math.pi / 2)
        self.assertEqual(
            [c.args[0] for c in console.record.call_args_list], ['P02', 'P02_after_turn']
        )

    def test_failure_does_not_record_or_run_next_action(self):
        console = self.console()
        console.move.side_effect = RuntimeError('Odom stale')
        with self.assertRaisesRegex(RuntimeError, 'Odom stale'):
            route.run_actions(console, validate_manifest(manifest()), Mock())
        self.assertEqual(console.move.call_count, 1)
        console.record.assert_not_called()

    def test_cancel_survives_preparation_transient_reset(self):
        runner = Mock()
        cancellation = route.Cancellation(runner)
        cancellation.request(signal.SIGTERM, None)
        runner.cancel_requested = False
        with self.assertRaises(KeyboardInterrupt):
            cancellation.poll()
        runner.request_cancel.assert_called_once()

    def test_cleanup_continues_after_stop_failure(self):
        runner = Mock()
        runner.stop_translation.side_effect = RuntimeError('zero publication failed')
        with self.assertRaisesRegex(RuntimeError, 'Cleanup failed'):
            route.close_runner(runner)
        runner.stop_owned.assert_called_once()
        runner.close.assert_called_once_with(False)

    def test_prepare_is_required_with_precise_right_wall(self):
        runner = Mock(prepare_options=['-p', 'alignment_wall:=right'])
        runner.pose.return_value = Pose(0, 0, 0)
        runner.frame, runner.twist_frame = 'odom', 'base_link'
        with (
            patch.object(route, 'TeachingBook'),
            patch.object(route, 'Console'),
            patch.object(route, 'run_actions'),
            patch('route_feedback_pause.RouteFeedbackPause'),
        ):
            route.prepare_and_run(runner, Path('unused'), io.StringIO(), (), 0.1, Mock())
        runner.prepare_start.assert_called_once()
        self.assertIn('precise_start:=true', runner.prepare_options)
        self.assertIn('positioning_wall:=right', runner.prepare_options)
        self.assertIn('right_target_distance:=0.26', runner.prepare_options)

    def test_no_success_event_on_execution_or_cleanup_failure(self):
        for failure in ('execution', 'cleanup'):
            audit, runner = io.StringIO(), Mock()
            options = SimpleNamespace(speed=0.04, turn_speed=0.1)
            execute_error = RuntimeError('bad action') if failure == 'execution' else None
            cleanup_error = RuntimeError('bad cleanup') if failure == 'cleanup' else None
            with (
                patch.object(route, 'WallRunner', return_value=runner),
                patch.object(route, 'prepare_and_run', side_effect=execute_error),
                patch.object(route, 'close_runner', side_effect=cleanup_error),
            ):
                with self.assertRaises(RuntimeError):
                    route.session(options, Path('unused'), audit, validate_manifest(manifest()))
            self.assertNotIn('route_completed', audit.getvalue())

    def test_main_propagates_failure_as_nonzero_and_audits(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            source = path / 'route.json'
            source.write_text(json.dumps(manifest()))
            argv = [
                '--scene',
                '2',
                '--route-json',
                str(source),
                '--laser-log-only',
                '--output',
                str(path / 'result.json'),
            ]
            with (
                patch.object(route.fcntl, 'flock'),
                patch.object(route, 'session', side_effect=RuntimeError('failed')),
            ):
                self.assertEqual(route.main(argv), 2)
            self.assertIn('route_failed', (path / 'result.jsonl').read_text())

    def test_invalid_later_action_never_creates_runner(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'route.json'
            data = manifest()
            data['actions'][-1]['heading_rad'] = float('inf')
            path.write_text(json.dumps(data))
            with patch.object(route, 'WallRunner') as runner:
                self.assertEqual(
                    route.main(['--scene', '2', '--route-json', str(path), '--laser-log-only']), 2
                )
            runner.assert_not_called()


if __name__ == '__main__':
    unittest.main()
