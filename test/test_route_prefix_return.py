"""Boundary recovery admission must not turn a failed/partial motion into success."""

from copy import deepcopy
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from motion_model import target_pose
from route_prefix_return_plan import ODOM, audit_prefix, verify_bag, verify_snapshot, FAILURE
from route_prefix_return_runtime import PrefixContinuity
from route_return_plan import ReturnSource, ReturnSequence, validate_source
from task6_route_model import RouteAction


def fixture():
    origin = Pose(0, 0, 0.2)
    actions = [
        RouteAction('P02', 'path', 0, ((0, 0), (0.02, 0.01), (0.04, 0))),
        RouteAction('P03', 'path', 0, ((0.04, 0), (0.06, -0.01), (0.08, 0))),
        RouteAction('P03_after_turn', 'turn', -1.57),
    ]
    book = dict(schema=1, frame='odom', session_id='prefix-test', origin=asdict(origin), points={})
    book['points']['P01'] = dict(pose=asdict(origin), odom_stamp_ns=100)
    rows = [
        dict(event='route_started', scene=2, frame='P01', actions=[asdict(a) for a in actions]),
        dict(event='record', name='P01', point=book['points']['P01'], time=1),
    ]
    for i, action in enumerate(actions[:2]):
        motion = action.path_motion()
        end = target_pose(origin, motion)
        book['points'][action.name] = dict(pose=asdict(end), odom_stamp_ns=101 + i)
        rows += [
            dict(event='route_action_started', index=i, name=action.name),
            dict(event='started', start=asdict(origin), motion=asdict(motion), target=asdict(end)),
            dict(event='final_check', ready=True),
            dict(event='completed', actual=asdict(end)),
            dict(event='record', name=action.name, point=book['points'][action.name], time=2 + i),
        ]
    rows += [
        dict(event='route_action_started', index=2, name=actions[2].name),
        dict(event='route_failed', exit_code=2, reason='Cancelled'),
    ]
    return json.loads(json.dumps(book)), json.loads(json.dumps(rows))


def bag_fixture():
    end = Pose(0.08, 0, 0)
    sample = dict(pose=asdict(end), velocity=(0, 0, 0), frames=('odom', 'base_link'))
    streams = {
        t: [
            dict(sample, receipt=3 + i / 10, stamp_ns=3_000_000_000 + i * 100_000_000)
            for i in range(8)
        ]
        for t in ODOM
    }
    snapshot = dict(
        unix_time=3.1, probe=dict(topics={ODOM[1]: dict(sample=dict(pose=[0.08, 0, 0]))})
    )
    return streams, [(2.9, (0, 0, 0))], 3, end, snapshot


class PrefixPlanTest(unittest.TestCase):
    def test_failed_prefix_two_reverse_paths_preserve_all_vertices(self):
        book, rows = fixture()
        actions, boundary, start = audit_prefix(book, rows)
        self.assertEqual((boundary, start), (3, 1))
        with self.assertRaises(ValueError):
            validate_source(book, rows)
        source = ReturnSource(
            'prefix-test', Pose(**book['origin']), actions[-1].end, 102, 'sha', actions
        )
        state = ReturnSequence(source)
        for action in reversed(actions):
            motion = state.begin(state.last_stop)
            self.assertEqual(motion.kind, 'path')
            self.assertTrue(
                all(
                    any(math.dist(p, q) < 1e-12 for q in motion.points)
                    for p in reversed(action.points)
                )
            )
            state.finish(action.start)
        self.assertEqual(state.cursor, 0)
        self.assertEqual(state.last_stop, source.origin)

    def test_next_started_or_partial_turn_rejected(self):
        book, rows = fixture()
        for event in ('started', 'completed', 'final_check'):
            bad = deepcopy(rows)
            bad.insert(-1, dict(event=event))
            with self.assertRaises(ValueError):
                audit_prefix(book, bad)

    def test_wrong_order_false_completion_and_changed_record_rejected(self):
        book, rows = fixture()
        for mutate in (
            lambda rs: rs[2].update(index=1),
            lambda rs: rs[4].update(ready=False),
            lambda rs: rs[-3]['point']['pose'].update(x=10),
        ):
            bad = deepcopy(rows)
            mutate(bad)
            with self.assertRaises(ValueError):
                audit_prefix(book, bad)

    def test_closed_final_stop_and_no_post_boundary_motion(self):
        args = bag_fixture()
        verify_bag(*args)
        for command in ((0.001, 0, 0), (0, 0, 0.001)):
            with self.assertRaises(ValueError):
                verify_bag(args[0], [*args[1], (3.2, command), (3.3, (0, 0, 0))], *args[2:])
        for field, value in (('velocity', (0.004, 0, 0)), ('pose', dict(x=0.1, y=0, yaw=0))):
            bad = deepcopy(args)
            bad[0][ODOM[1]][-1][field] = value
            with self.assertRaises(ValueError):
                verify_bag(*bad)

    def test_source_gap_backward_and_missing_final_observation(self):
        for kind in ('backward', 'gap', 'missing'):
            args = deepcopy(bag_fixture())
            if kind == 'missing':
                args[0][ODOM[0]] = []
            else:
                args[0][ODOM[0]][-1]['stamp_ns'] += (
                    -1_000_000_000 if kind == 'backward' else 600_000_000
                )
            with self.assertRaises(ValueError):
                verify_bag(*args)

    def test_snapshot_rejects_epoch_moving_stale_publishers(self):
        gids = {t: ['aa'] for t in ODOM}
        snapshot = dict(
            error=FAILURE,
            phase='official_runner',
            probe=dict(
                cmd_endpoints=[],
                topics={
                    t: dict(
                        publishers=[dict(gid='aa')],
                        sample=dict(
                            frame='odom', child='base_link', velocity=[0, 0, 0], pose=[0, 0, 0]
                        ),
                        receipt_age_s=0.01,
                        source_age_s=0.01,
                    )
                    for t in ODOM
                },
            ),
        )
        verify_snapshot(snapshot, gids, Pose(0, 0, 0))
        for change in (
            lambda s: s['probe'].update(cmd_endpoints=['publisher']),
            lambda s: s['probe']['topics'][ODOM[0]].update(source_age_s=0.51),
            lambda s: s['probe']['topics'][ODOM[1]].update(publishers=[dict(gid='bb')]),
            lambda s: s['probe']['topics'][ODOM[1]]['sample'].update(velocity=[0.01, 0, 0]),
        ):
            bad = deepcopy(snapshot)
            change(bad)
            with self.assertRaises(ValueError):
                verify_snapshot(bad, gids, Pose(0, 0, 0))


class PrefixLiveTest(unittest.TestCase):
    def guard(self):
        obj = PrefixContinuity.__new__(PrefixContinuity)
        obj.raw = dict(
            stamp_ns=10_000_000_000,
            receipt=10,
            pose=dict(x=0, y=0, yaw=0),
            velocity=(0, 0, 0),
            frames=('odom', 'base_link'),
        )
        obj.raw_invalid = None
        obj.metadata = dict(raw_stamp_ns=9_000_000_000, raw_last_stop=dict(x=0, y=0, yaw=0))
        obj.source = SimpleNamespace(last_stop=Pose(0, 0, 0))
        obj.runner = Mock(body_velocity=(0, 0, 0))
        obj.runner.node.get_clock().now.return_value.nanoseconds = 10_100_000_000
        obj.runner.moving_pose.return_value = Pose(0, 0, 0)
        return obj

    def test_raw_duplicate_does_not_refresh_receipt_and_mutation_rejected(self):
        guard = self.guard()
        current = dict(guard.raw, receipt=10.3)
        guard.compare_raw(current, Pose(0, 0, 0))
        self.assertEqual(current['receipt'], 10)
        current['velocity'] = (0.01, 0, 0)
        with self.assertRaises(ValueError):
            guard.compare_raw(current, Pose(0, 0, 0))
        current['stamp_ns'] -= 1
        with self.assertRaises(ValueError):
            guard.compare_raw(current, Pose(0, 0, 0))

    def test_freshness_and_stopped_pose_before_first_command(self):
        guard = self.guard()
        with patch('route_prefix_return_runtime.time.monotonic', return_value=10.1):
            guard.check_raw_fresh()
            guard.check_initial_stop()
        with patch('route_prefix_return_runtime.time.monotonic', return_value=10.6):
            with self.assertRaises(RuntimeError):
                guard.check_raw_fresh()
        guard.raw['pose']['x'] = 0.006
        with self.assertRaises(RuntimeError):
            guard.check_initial_stop()

    def test_immutable_evidence_changed_or_replaced_rejected(self):
        from route_prefix_return_runtime import fingerprint
        from route_prefix_return_plan import digest

        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory) / 'original.json'
            p.write_text('failed')
            obj = self.guard()
            obj.files = {p: digest(p)}
            obj.stats = {p: fingerprint(p)}
            obj.verify_bytes()
            p.write_text('passed')
            with self.assertRaises(RuntimeError):
                obj.verify_bytes()
            with self.assertRaises(RuntimeError):
                obj.check_files()


class PrefixRequestTest(unittest.TestCase):
    def test_watcher_continuity_faults_and_missing_coverage_refused(self):
        from route_prefix_return_plan import verify_watch

        gids = {t: ['aa'] for t in ODOM}
        rows = [dict(event='publisher_graph', topic=t, current=[['aa']], wall_s=0) for t in ODOM]
        rows += [dict(event='sample', wall_s=0), dict(event='sample', wall_s=3)]
        verify_watch(rows, gids, 1, 2)
        for event in ('feedback_gap', 'odom_change', 'nonfinite_odom', 'invalid_odom'):
            with self.assertRaises(ValueError):
                verify_watch([*rows, dict(event=event, wall_s=1.5)], gids, 1, 2)
        with self.assertRaises(ValueError):
            verify_watch(rows[:-1], gids, 1, 2)
        with self.assertRaises(ValueError):
            verify_watch(
                [*rows, dict(event='publisher_graph', topic=ODOM[1], current=[['bb']], wall_s=1.5)],
                gids,
                1,
                2,
            )

    def test_wrong_uuid_either_gid_and_output_inside_source_refused(self):
        from route_prefix_return_runner import verify_request

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            path.joinpath('source').mkdir()
            args = SimpleNamespace(
                confirm_same_odom='session',
                odom_publisher_gid='aa' * 24,
                raw_odom_publisher_gid='bb' * 24,
                source_run_dir=path / 'source',
                output=path / 'return.json',
            )
            source = SimpleNamespace(session_id='session')
            metadata = dict(gids={ODOM[0]: ['aa' * 24], ODOM[1]: ['bb' * 24]})
            self.assertEqual(
                verify_request(args, source, metadata), path / 'source/points.return_claim.json'
            )
            for key, value in (
                ('confirm_same_odom', 'other'),
                ('odom_publisher_gid', 'cc' * 24),
                ('raw_odom_publisher_gid', 'cc' * 24),
                ('output', path / 'source/return.json'),
            ):
                bad = deepcopy(args)
                setattr(bad, key, value)
                with self.assertRaises(ValueError):
                    verify_request(bad, source, metadata)
            (path / 'source/points.return_claim.json').write_text('{}')
            with self.assertRaises(ValueError):
                verify_request(args, source, metadata)


class PrefixCleanupTest(unittest.TestCase):
    def test_subscription_teardown_failure_still_stops_runner(self):
        import route_prefix_return_runner as entry

        runner = Mock()
        continuity = Mock()
        continuity.close.side_effect = RuntimeError('subscription teardown failed')
        with (
            patch.object(entry, 'WallRunner', return_value=runner),
            patch.object(entry, 'Cancellation'),
            patch.object(entry, 'PrefixContinuity', return_value=continuity),
            patch.object(entry, 'run_session', return_value=0),
            patch.object(entry, 'close_runner') as close,
            patch.object(entry.signal, 'signal'),
        ):
            with self.assertRaisesRegex(RuntimeError, 'subscription teardown'):
                entry.execute(SimpleNamespace(speed=0.03), Mock(), {}, Path('claim'), Mock())
            close.assert_called_once_with(runner)
