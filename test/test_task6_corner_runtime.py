"""Local fake-runner motion integration; never imports or starts ROS."""

import io
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose  # noqa: E402
from motion_console import Console  # noqa: E402
from motion_model import Motion  # noqa: E402
from route_laser_guard import ScanSnapshot  # noqa: E402
from task6_corner_geometry import CornerPolicy  # noqa: E402
import task6_corner_runtime as runtime  # noqa: E402


def policy():
    return CornerPolicy(0.09, 0.005, 0.02, 0.08, 0.02, 0.20, 0.005, 3, 0.3)


class FakeRunner:
    def __init__(self, pose=Pose(0, 0, 0), front=0.30):
        self.now = 1.0
        self.actual = pose
        self.front = front
        self.velocity_pub = None
        self.command = (0.0, 0.0, 0.0)
        self.sent = []
        self.latest = (pose, None, 0.0, 0.0)
        self.body_velocity = (0.0, 0.0, 0.0)
        self.frame, self.twist_frame = 'odom', 'base_link'
        self.max_speed = 0.04
        self.stamp = 1_000_000_000
        self.cached_stamp = self.stamp
        self.freeze = False
        self.cancel = False
        self.bad_identity = False
        self.feedback_pause = SimpleNamespace(generation=0, check=lambda: None)
        self.node = SimpleNamespace(
            create_publisher=self.create,
            destroy_publisher=self.destroy,
            get_publishers_info_by_topic=lambda _: [1] if self.velocity_pub is not None else [],
        )
        self.ros = SimpleNamespace(spin_once=lambda _node, timeout_sec: self.tick(timeout_sec))
        self.twist = object

    def create(self, *_args):
        assert self.velocity_pub is None
        return object()

    def destroy(self, _publisher):
        self.velocity_pub = None

    def exclusive(self):
        assert self.velocity_pub is None

    def assert_exclusive(self):
        pass

    def pose(self):
        return self.actual

    def tick(self, dt):
        self.now += max(dt, 1e-4)
        vx, vy, wz = self.command
        c, s = math.cos(self.actual.yaw), math.sin(self.actual.yaw)
        self.actual = Pose(
            self.actual.x + (c * vx - s * vy) * dt,
            self.actual.y + (s * vx + c * vy) * dt,
            self.actual.yaw + wz * dt,
        )
        self.latest = (self.actual, None, math.hypot(vx, vy), abs(wz))
        self.body_velocity = self.command
        if not self.freeze:
            self.stamp = int(self.now * 1e9)

    def pump(self):
        self.tick(0.05)

    def send(self, command):
        self.command = tuple(command)
        self.sent.append(tuple(command))

    def motion_walls(self, _sides, _heading):
        return {'front': SimpleNamespace(gap=self.front - self.actual.x - 0.17)}

    def read_frame(self):
        points, rays = [], []
        for i in range(720):
            a = -math.pi + i * math.pi / 360
            c, s = math.cos(a), math.sin(a)
            distances = []
            if abs(c) > 1e-10:
                distances.append(
                    ((self.front - self.actual.x) if c > 0 else (-1 - self.actual.x)) / c
                )
            if abs(s) > 1e-10:
                distances.append(((0.5 - self.actual.y) if s > 0 else (-0.5 - self.actual.y)) / s)
            d = min(x for x in distances if x > 0)
            points.append((d * c, d * s))
            rays.append((a, True, d))
        identity = ('changed' if self.bad_identity and self.now > 1.3 else 'scan',)
        self.cached_stamp = self.stamp
        return runtime._Frame(
            ScanSnapshot(tuple(points), tuple((a, v) for a, v, _ in rays)),
            tuple(rays),
            self.actual,
            self.stamp,
            self.stamp,
            identity,
            (0.0, 0.0),
            'same-raw',
        )


def console(runner):
    inputs = SimpleNamespace(poll=lambda: (False, None))
    return Console(runner, None, inputs, io.StringIO(), 0.10)


class RuntimeTests(unittest.TestCase):
    def run_corner(self, runner, name, nominal, budget=12):
        c = console(runner)
        with (
            patch.object(runtime, '_read_frame', lambda r: r.read_frame()),
            patch.object(runtime.time, 'monotonic', lambda: runner.now),
        ):
            return runtime.CornerPlacement(c, policy(), enabled=True).place(
                name, nominal, budget_s=budget
            )

    def test_front_moves_and_accepts_by_scan_target_before_old_odom_goal(self):
        r = FakeRunner()
        endpoint = self.run_corner(r, 'P10', Pose(0.08, 0, 0))
        self.assertGreater(endpoint.actual.x, 0.02)
        self.assertLess(endpoint.actual.x, 0.05)
        self.assertLess(endpoint.actual.x, endpoint.original_nominal.x)
        self.assertGreaterEqual(len(endpoint.scans), 3)
        self.assertTrue(any(v[0] > 0 for v in r.sent))
        self.assertIsNone(r.velocity_pub)
        self.assertEqual(r.sent[-1], (0.0, 0.0, 0.0))

    def test_lane_moves_to_geometric_center_and_requires_distinct_post_stop_scans(self):
        r = FakeRunner(Pose(0, -0.025, 0), front=1.0)
        endpoint = self.run_corner(r, 'P09', Pose(0, 0.02, 0))
        self.assertGreater(endpoint.actual.y, -0.01)
        self.assertTrue(any(v[1] > 0 for v in r.sent))
        self.assertGreaterEqual(len(set(row['scan_stamp_ns'] for row in endpoint.scans)), 3)

    def test_single_repeated_scan_cannot_accept_and_deadline_stops(self):
        r = FakeRunner(front=0.26)
        r.freeze = True
        with self.assertRaisesRegex(RuntimeError, 'deadline'):
            self.run_corner(r, 'P10', Pose(0, 0, 0), budget=1.0)
        self.assertIsNone(r.velocity_pub)
        self.assertEqual(r.sent[-1], (0.0, 0.0, 0.0))

    def test_identity_change_and_stale_failure_zero_and_release(self):
        r = FakeRunner()
        r.bad_identity = True
        with self.assertRaisesRegex(RuntimeError, 'identity'):
            self.run_corner(r, 'P10', Pose(0.08, 0, 0))
        self.assertIsNone(r.velocity_pub)
        self.assertEqual(r.sent[-1], (0.0, 0.0, 0.0))
        r = FakeRunner()
        c = console(r)
        with (
            patch.object(runtime, '_read_frame', side_effect=RuntimeError('stale source')),
            patch.object(runtime.time, 'monotonic', lambda: r.now),
        ):
            with self.assertRaisesRegex(RuntimeError, 'stale'):
                runtime.CornerPlacement(c, policy(), enabled=True).place('P10', Pose(0.08, 0, 0))
        self.assertIsNone(r.velocity_pub)
        self.assertEqual(r.sent[-1], (0.0, 0.0, 0.0))

    def test_cancel_does_not_return_accepted_endpoint(self):
        r = FakeRunner()
        c = console(r)
        checks = [0]

        def cancel():
            checks[0] += 1
            if checks[0] > 4:
                raise KeyboardInterrupt

        c.cancel_input = cancel
        with (
            patch.object(runtime, '_read_frame', lambda r: r.read_frame()),
            patch.object(runtime.time, 'monotonic', lambda: r.now),
        ):
            with self.assertRaises(KeyboardInterrupt):
                runtime.CornerPlacement(c, policy(), enabled=True).place('P10', Pose(0.08, 0, 0))
        self.assertIsNone(r.velocity_pub)
        self.assertEqual(r.sent[-1], (0.0, 0.0, 0.0))

    def test_feedback_recovery_resets_pid_time_without_restarting_target(self):
        r = FakeRunner(Pose(0, -0.025, 0), front=1.0)
        original = r.pump
        fired = [False]

        def pump():
            if not fired[0] and r.now > 1.25:
                fired[0] = True
                r.send((0.0, 0.0, 0.0))
                r.tick(0.7)
                r.feedback_pause.generation += 1
            original()

        r.pump = pump
        endpoint = self.run_corner(r, 'P09', Pose(0, 0.02, 0))
        self.assertTrue(fired[0])
        self.assertLess(abs(endpoint.actual.y), 0.01)
        self.assertIsNone(r.velocity_pub)

    def test_short_wall_fit_loss_waits_zero_then_recovers_same_action(self):
        from wall_runtime import WallFitError

        r = FakeRunner()
        original = r.motion_walls
        count = [0]

        def walls(*args):
            count[0] += 1
            if count[0] <= 3:
                raise WallFitError('temporary test fit loss')
            return original(*args)

        r.motion_walls = walls
        endpoint = self.run_corner(r, 'P10', Pose(0.08, 0, 0))
        self.assertGreater(endpoint.actual.x, 0.02)
        self.assertEqual(r.sent[:3], [(0.0, 0.0, 0.0)] * 3)
        self.assertIsNone(r.velocity_pub)

    def test_opt_in_and_connector_keep_fixed_suffix_and_old_turn_gate(self):
        with self.assertRaises(ValueError):
            runtime.CornerPlacement(console(FakeRunner()), policy())
        accepted = runtime.AcceptedEndpoint(
            'P10', Pose(0, 0, 0), Pose(0.03, 0, 0), ({}, {}, {}), ()
        )
        motion = Motion(
            'path', x=0.12, y=0.02, angle=0, points=((0, 0), (0.04, 0), (0.08, 0), (0.12, 0.02))
        )
        result = runtime.connector(motion, Pose(0, 0, 0), accepted)
        self.assertEqual(result.points[-3:], motion.points[-3:])
        self.assertTrue(
            all(math.dist(a, b) <= 0.05 + 1e-12 for a, b in zip(result.points, result.points[1:]))
        )
        with self.assertRaisesRegex(RuntimeError, '2 cm'):
            runtime.connector(motion, Pose(0, 0, 0), accepted, Pose(0.051, 0, 0))
        self.assertEqual(motion.points[0], (0, 0))


if __name__ == '__main__':
    unittest.main()
