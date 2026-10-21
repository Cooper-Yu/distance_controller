"""Completed audit provenance, absolute reverse actions, and continuity rejection."""

from copy import deepcopy
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from motion_model import Motion, target_pose
from route_return_plan import ReturnSequence, ReturnSource, source_actions, validate_source
from route_return_runner import Continuity, attach, claim_source, normalize_gid
from task6_route_model import RouteAction


def fixture():
    origin = Pose(2, 3, 0.2)
    actions = (
        RouteAction('P02', 'path', 0, ((0, 0), (0.04, 0))),
        RouteAction('P02_turn', 'turn', 0.3),
        RouteAction('P03', 'path', 0.3, ((0.04, 0), (0.045, 0.02), (0.04, 0.04))),
    )
    book = dict(
        schema=1, frame='odom', session_id='test-live-session', origin=asdict(origin), points={}
    )

    def point(actual, stamp):
        return dict(pose=asdict(actual), odom_stamp_ns=stamp)

    book['points']['P01'] = point(origin, 100)
    rows = [
        dict(event='route_started', scene=2, frame='P01', actions=[asdict(a) for a in actions]),
        dict(event='record', name='P01', point=book['points']['P01']),
    ]
    previous = origin
    for i, action in enumerate(actions):
        motion = action.path_motion() if action.kind == 'path' else Motion('turn', angle=0.3)
        anchor = origin if action.kind == 'path' else previous
        end = target_pose(anchor, motion)
        book['points'][action.name] = point(end, 101 + i)
        rows += [
            dict(event='route_action_started', index=i, name=action.name),
            dict(event='started', start=asdict(anchor), motion=asdict(motion), target=asdict(end)),
            dict(event='final_check', ready=True),
            dict(event='completed', actual=asdict(end)),
            dict(event='record', name=action.name, point=book['points'][action.name]),
        ]
        previous = end
    rows.append(dict(event='route_completed', destination='P03', exit_code=0))
    return json.loads(json.dumps(book)), json.loads(json.dumps(rows))


def source():
    book, rows = fixture()
    return ReturnSource(*validate_source(book, rows), 'hash', tuple(source_actions(book, rows)))


class ReturnPlanTest(unittest.TestCase):
    def test_reverse_translation_turn_translation_original_frame(self):
        original = source()
        state, actual = ReturnSequence(original), original.last_stop
        kinds = []
        for action in reversed(original.actions):
            motion = state.begin(actual)
            kinds.append(motion.kind)
            if motion.kind == 'turn':
                self.assertAlmostEqual(motion.angle, -0.3)
                self.assertAlmostEqual(actual.yaw + motion.angle, action.start.yaw)
            else:
                self.assertTrue(
                    all(any(math.dist(p, q) < 1e-9 for q in motion.points) for p in action.points)
                )
            actual = action.start
            state.finish(actual)
        self.assertEqual(kinds, ['path', 'turn', 'path'])
        self.assertEqual(state.cursor, 0)
        self.assertEqual(actual, original.origin)

    def test_start_tolerance_and_partial_fault_cannot_continue(self):
        state = ReturnSequence(source())
        actual = state.last_stop
        with self.assertRaises(RuntimeError):
            state.begin(Pose(actual.x + 0.021, actual.y, actual.yaw))
        with self.assertRaises(RuntimeError):
            state.begin(actual)
        state = ReturnSequence(source())
        state.begin(state.last_stop)
        with self.assertRaises(RuntimeError):
            state.begin(state.last_stop)
        state.fail('Stopped')
        with self.assertRaises(RuntimeError):
            state.finish(state.source.actions[-1].start)

    def test_original_four_cm_turn_limit_retained(self):
        state = ReturnSequence(source())
        state.begin(state.last_stop)
        state.finish(state.source.actions[-1].start)
        state.begin(state.last_stop)
        target = state.source.actions[1].start
        with self.assertRaises(RuntimeError):
            state.finish(Pose(target.x + 0.045, target.y, target.yaw))
        self.assertEqual(state.cursor, 2)

    def test_incomplete_failed_or_wrong_destination_refused(self):
        book, rows = fixture()
        for bad in (
            rows[:-1],
            [*rows[:-1], dict(event='route_failed'), rows[-1]],
            [*rows[:-1], dict(event='route_completed', destination='P15', exit_code=0)],
        ):
            with self.assertRaises(ValueError):
                validate_source(book, bad)

    def test_record_mismatch_reordered_actions_and_false_ready_refused(self):
        book, rows = fixture()
        mutations = [
            lambda r: r[2].update(index=5),
            lambda r: r[4].update(ready=False),
            lambda r: r[6]['point']['pose'].update(x=99),
        ]
        for mutate in mutations:
            bad = deepcopy(rows)
            mutate(bad)
            with self.assertRaises(ValueError):
                validate_source(book, bad)

    def test_changed_fixed_bend_and_nonfinite_pose_refused(self):
        book, rows = fixture()
        bad = deepcopy(rows)
        bad[13]['motion']['points'][1][0] += 0.001
        with self.assertRaises(ValueError):
            validate_source(book, bad)
        book['origin']['x'] = float('nan')
        with self.assertRaises(ValueError):
            validate_source(book, rows)

    def test_turn_uses_second_stopped_pose_as_actual_anchor(self):
        from teaching_path import turn_to_heading

        book, rows = fixture()
        origin = Pose(**book['origin'])
        planned_start = Pose(**rows[8]['start'])
        measured_start = Pose(planned_start.x, planned_start.y, planned_start.yaw + 0.0002)
        motion = turn_to_heading(origin, planned_start, 0.3)
        actual_target = target_pose(measured_start, motion)
        self.assertAlmostEqual(actual_target.yaw - origin.yaw - 0.3, 0.0002)
        rows[8]['start'], rows[8]['target'] = asdict(measured_start), asdict(actual_target)
        rows[10]['actual'] = asdict(actual_target)
        rows[11]['point']['pose'] = asdict(actual_target)
        book['points']['P02_turn']['pose'] = asdict(actual_target)
        validate_source(book, rows)
        restored = tuple(source_actions(book, rows))[1]
        self.assertEqual(restored.start, measured_start)
        self.assertEqual(restored.end, actual_target)
        # A forged target still fails the exact recorded motion/anchor relationship.
        rows[8]['target']['yaw'] += 0.0001
        with self.assertRaisesRegex(ValueError, 'Motion target mismatch'):
            validate_source(book, rows)

    def test_turn_nominal_heading_bound_and_geometry_retained(self):
        for delta in (0.0101, -0.0101):
            with self.subTest(delta=delta):
                book, rows = fixture()
                rows[8]['start']['yaw'] += delta
                rows[8]['target']['yaw'] += delta
                rows[10]['actual']['yaw'] += delta
                rows[11]['point']['pose']['yaw'] += delta
                book['points']['P02_turn']['pose']['yaw'] += delta
                with self.assertRaisesRegex(ValueError, 'heading differs from fixed route'):
                    validate_source(book, rows)
        for key, value in (('x', 0.001), ('y', 0.001), ('angle', 2 * math.pi + 0.3)):
            with self.subTest(key=key):
                book, rows = fixture()
                rows[8]['motion'][key] = value
                with self.assertRaisesRegex(ValueError, 'Unsupported source turn geometry'):
                    validate_source(book, rows)

    def test_path_nominal_heading_still_exact(self):
        book, rows = fixture()
        rows[3]['motion']['angle'] += 0.0001
        with self.assertRaisesRegex(ValueError, 'heading differs from fixed route'):
            validate_source(book, rows)

    def test_source_claim_is_permanent_single_attempt(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'attempt.json'
            claim_source(path, source())
            with self.assertRaises(FileExistsError):
                claim_source(path, source())


class ContinuityTest(unittest.TestCase):
    def runner(self):
        data = source()
        p = data.last_stop
        gid = bytes(range(24))
        runner = SimpleNamespace(
            exclusive=Mock(),
            pose=lambda: p,
            frame='odom',
            twist_frame='base_link',
            stamp=data.stamp_ns + 1,
            continuous_yaw=p.yaw,
            latest=(p, 1, 0, 0),
            node=SimpleNamespace(
                get_publishers_info_by_topic=lambda _: [SimpleNamespace(endpoint_gid=gid)]
            ),
        )
        return runner, data, gid.hex()

    def test_current_identity_time_and_pose_required(self):
        runner, data, gid = self.runner()
        attach(runner, data, Continuity(runner, gid))
        for key, value in [('frame', 'map'), ('twist_frame', 'laser'), ('stamp', 1)]:
            runner, data, gid = self.runner()
            setattr(runner, key, value)
            with self.assertRaises(RuntimeError):
                attach(runner, data, Continuity(runner, gid))
        runner, data, gid = self.runner()
        runner.pose = lambda: Pose(data.last_stop.x, data.last_stop.y, data.last_stop.yaw + 0.02)
        with self.assertRaises(RuntimeError):
            attach(runner, data, Continuity(runner, gid))

    def test_publisher_change_rejected_during_command(self):
        runner, _data, gid = self.runner()
        guard = Continuity(runner, gid)
        self.assertEqual(guard((0.01, 0, 0)), (0.01, 0, 0))
        runner.node.get_publishers_info_by_topic = lambda _: []
        with self.assertRaises(RuntimeError):
            guard((0.01, 0, 0))
        with self.assertRaises(ValueError):
            normalize_gid('unknown')


if __name__ == '__main__':
    unittest.main()
