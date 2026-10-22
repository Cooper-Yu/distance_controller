"""No ROS initialization: execute actual stop methods against a deterministic callback clock."""

from pathlib import Path
import sys
from types import SimpleNamespace as N
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from motion_console import Console
from wall_runtime import WallRunner


class FakeRunner:
    def __init__(self, mode='ready', cancel=False, failure=None):
        self.mode, self.failure = mode, failure
        self.now, self.sent, self.waits = 0.0, [], []
        self.destroyed, self.cancel_requested = 0, cancel
        self.velocity_pub = object()
        self.node = N(destroy_publisher=self.release)
        self.ros = N(spin_once=self.spin)

    def send(self, velocity):
        self.sent.append((self.now, velocity))
        if self.failure == 'send':
            raise RuntimeError('send failed')

    def spin(self, _node, timeout_sec):
        self.waits.append(timeout_sec)
        if self.failure == 'spin':
            raise RuntimeError('callback failed')
        if self.mode == 'idle':
            self.now += timeout_sec
        elif self.mode == 'slow':
            self.now += 0.27
        else:
            self.now += min(timeout_sec, 0.0002)
        if self.mode == 'cancel':
            self.cancel_requested = True

    def release(self, _publisher):
        self.destroyed += 1


def stop(runner, kind):
    with patch('time.monotonic', side_effect=lambda: runner.now):
        if kind == 'console':
            Console.stop_motion(N(runner=runner))
        else:
            WallRunner.stop_translation(runner)


class ZeroCommandPacingTest(unittest.TestCase):
    def test_ready_self_echo_cannot_multiply_publish_frequency(self):
        for kind in ('console', 'wall'):
            runner = FakeRunner()
            stop(runner, kind)
            times = [t for t, _ in runner.sent]
            with self.subTest(kind=kind):
                self.assertEqual(times[0], 0)
                self.assertTrue(19 <= len(times) <= 21)
                self.assertTrue(all(b - a >= 0.05 - 1e-12 for a, b in zip(times, times[1:])))
                self.assertGreater(len(runner.waits), 1000)
                self.assertTrue(all(0 <= t <= 0.05 + 1e-12 for t in runner.waits))
                self.assertTrue(all(v == (0, 0, 0) for _, v in runner.sent))
                self.assertGreaterEqual(runner.now, 1.0)
                self.assertLessEqual(runner.now, 1.000001)
                self.assertIsNone(runner.velocity_pub)
                self.assertEqual(runner.destroyed, 1)

    def test_no_ready_callbacks_keep_twenty_hz_and_one_second_lifetime(self):
        for kind in ('console', 'wall'):
            runner = FakeRunner('idle')
            stop(runner, kind)
            self.assertTrue(20 <= len(runner.sent) <= 21)
            self.assertAlmostEqual(runner.now, 1.0)
            self.assertEqual(runner.destroyed, 1)

    def test_slow_callback_does_not_catch_up_with_burst_or_extend_deadline(self):
        for kind in ('console', 'wall'):
            runner = FakeRunner('slow')
            stop(runner, kind)
            self.assertEqual([round(t, 2) for t, _ in runner.sent], [0.0, 0.27, 0.54, 0.81])
            self.assertAlmostEqual(runner.now, 1.08)
            self.assertEqual(runner.destroyed, 1)

    def test_existing_cancel_is_preserved_by_console_and_wall_contract_unchanged(self):
        for kind, expected in (('console', True), ('wall', False)):
            runner = FakeRunner('idle', cancel=True)
            stop(runner, kind)
            self.assertEqual(runner.cancel_requested, expected)
            self.assertGreaterEqual(len(runner.sent), 20)

    def test_cancel_arriving_during_drain_does_not_skip_zeros_or_publisher_release(self):
        for kind, expected in (('console', True), ('wall', False)):
            runner = FakeRunner('cancel')
            stop(runner, kind)
            self.assertEqual(runner.cancel_requested, expected)
            self.assertEqual(runner.destroyed, 1)
            self.assertGreaterEqual(len(runner.sent), 19)

    def test_send_and_callback_errors_propagate_with_release(self):
        for kind in ('console', 'wall'):
            for failure in ('send', 'spin'):
                runner = FakeRunner(failure=failure, cancel=True)
                with self.subTest(kind=kind, failure=failure), self.assertRaises(RuntimeError):
                    stop(runner, kind)
                self.assertEqual(runner.sent, [(0.0, (0, 0, 0))])
                self.assertEqual(runner.destroyed, 1)
                self.assertIsNone(runner.velocity_pub)
                if kind == 'console':
                    self.assertTrue(runner.cancel_requested)

    def test_no_publisher_is_noop(self):
        for kind in ('console', 'wall'):
            runner = FakeRunner(cancel=True)
            runner.velocity_pub = None
            stop(runner, kind)
            self.assertEqual((runner.sent, runner.waits, runner.destroyed), ([], [], 0))
            self.assertTrue(runner.cancel_requested)


if __name__ == '__main__':
    unittest.main()
