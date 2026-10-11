"""Console contracts: parsing before motion, rotated axes, correction flags and persistence."""

import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from action_plan import Pose
from motion_model import Motion, parse_batch, feedback, local_delta, target_pose
from motion_console import TeachingBook, Console


class MotionConsoleTest(unittest.TestCase):
    def test_batch_is_fully_validated(self):
        with self.assertRaises(ValueError):
            parse_batch('move forward 0.1; move left nan')
        for text in (
            'move forward 0.1 move left 0.03',
            'move xy 0 0',
            'turn right inf',
            'move right -0.1',
            'turn left 0',
            'move xy 0.1 0.2 --heading yes',
            'move left 0.1 --lateral off --lateral on',
        ):
            with self.subTest(text=text), self.assertRaises(ValueError):
                parse_batch(text)
        self.assertEqual(len(parse_batch('move forward .1; move left .03')), 2)

    def test_start_frame_rotation(self):
        start = Pose(10, 20, math.pi / 2)
        end = target_pose(start, Motion('move', x=0.1, y=0.03))
        self.assertAlmostEqual(end.x, 9.97)
        self.assertAlmostEqual(end.y, 20.1)
        delta = local_delta(start, end)
        self.assertAlmostEqual(delta['x_m'], 0.1)
        self.assertAlmostEqual(delta['y_m'], 0.03)

    def test_diagonal_cross_track_correction(self):
        motion = Motion('move', x=0.1, y=0.03)
        velocity, _ = feedback(Pose(0, 0, 0), Pose(0.05, 0.01, 0), motion, 0.08, 0.1)
        # Left of the requested velocity ratio means correcting towards the line.
        self.assertGreater(velocity[1] / velocity[0], 0.3)
        off = Motion('move', x=0.1, y=0.03, lateral=False)
        velocity, _ = feedback(Pose(0, 0, 0), Pose(0.05, 0.01, 0), off, 0.08, 0.1)
        self.assertAlmostEqual(velocity[1] / velocity[0], 0.3)

    def test_disabled_corrections_do_not_block_arrival(self):
        start, actual = Pose(0, 0, 0), Pose(0.1, 0.03, 0.2)
        off = Motion('move', x=0.1, heading=False, lateral=False)
        velocity, ready = feedback(start, actual, off, 0.04, 0.1)
        self.assertTrue(ready)
        self.assertEqual(velocity[2], 0)
        self.assertFalse(feedback(start, actual, Motion('move', x=0.1), 0.04, 0.1)[1])

    def test_heading_off_still_uses_current_body_frame(self):
        velocity, _ = feedback(
            Pose(0, 0, 0), Pose(0, 0, math.pi / 2), Motion('move', x=0.1, heading=False), 0.04, 0.1
        )
        self.assertAlmostEqual(velocity[0], 0)
        self.assertLess(velocity[1], 0)

    def test_signed_turn_preserves_270_degrees(self):
        motion = parse_batch('turn right 270')[0]
        velocity, ready = feedback(Pose(0, 0, 3), Pose(0, 0, 3), motion, 0.04, 0.1)
        self.assertLess(velocity[2], 0)
        self.assertFalse(ready)
        self.assertTrue(
            feedback(Pose(0, 0, 3), target_pose(Pose(0, 0, 3), motion), motion, 0.04, 0.1)[1]
        )

    def test_turn_drift_fault(self):
        with self.assertRaises(RuntimeError):
            feedback(Pose(0, 0, 0), Pose(0.05, 0, 0), Motion('turn', angle=1), 0.04, 0.1)

    def test_stop_cleanup_preserves_new_cancel(self):
        from types import SimpleNamespace
        from unittest.mock import patch

        commands = []
        runner = SimpleNamespace(velocity_pub=object(), cancel_requested=False)
        runner.send = commands.append
        runner.node = SimpleNamespace(destroy_publisher=lambda _publisher: None)
        runner.ros = SimpleNamespace(
            spin_once=lambda *_args, **_kwargs: setattr(runner, 'cancel_requested', True)
        )
        console = Console(runner, None, None, None, 0.1)
        with patch('motion_console.time.monotonic', side_effect=[0, 0, 2]):
            console.stop_motion()
        self.assertTrue(runner.cancel_requested)
        self.assertIsNone(runner.velocity_pub)
        self.assertEqual(commands, [(0.0, 0.0, 0.0)])

    def test_shared_pid_for_move_turn_and_disabled_heading(self):
        from motion_model import HeadingPid

        start, actual = Pose(0, 0, 0), Pose(0, 0, 0.02)
        pid = HeadingPid()
        move, _ = feedback(start, actual, Motion('move', x=0.02), 0.12, 0.6, pid, 0.05, 0.04)
        turn, _ = feedback(
            start, actual, Motion('turn', angle=0), 0.12, 0.6, HeadingPid(), 0.05, 0.04
        )
        self.assertAlmostEqual(move[2], turn[2])
        self.assertAlmostEqual(turn[2], -0.036 - 0.00003 - 0.014)
        self.assertAlmostEqual(math.hypot(*move[:2]), 0.03)
        disabled, _ = feedback(
            start, actual, Motion('move', x=0.02, heading=False), 0.12, 0.6, pid, 0.05, 0.04
        )
        self.assertEqual(disabled[2], 0)
        self.assertEqual(pid.integral, 0)

    def test_book_persistence_and_delta(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'points.json'
            book = TeachingBook(path, Pose(1, 2, math.pi / 2), 'odom')
            book.record('P01', book.origin, 1, {'status': 'no_scan'})
            book.record('P02', Pose(0.97, 2.1, math.pi / 2), 2, {'status': 'no_scan'})
            delta = book.delta('P01', 'P02')['start_body']
            self.assertAlmostEqual(delta['x_m'], 0.1)
            self.assertAlmostEqual(delta['y_m'], 0.03)
            self.assertEqual(len(json.loads(path.read_text())['points']), 2)
            with self.assertRaises(ValueError):
                book.record('P02', book.origin, 3, {})
            with self.assertRaises(FileExistsError):
                TeachingBook(path, book.origin, 'odom')


if __name__ == '__main__':
    unittest.main()
