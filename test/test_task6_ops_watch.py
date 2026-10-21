import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools' / 'task6_ops'))
from odom_reset_watch import Watch


class ProbeTest(unittest.TestCase):
    def test_events_identity_and_idle_bounds(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'probe.jsonl'
            w = Watch(path, 65536)
            self.assertTrue(w.idle()['idle_is_lower_bound'])
            msg = Odometry()
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'base_link'
            msg.pose.pose.orientation.w = 1.0
            msg.header.stamp.sec = 100
            w.odom('/odometry/filtered', msg)
            w.odom('/odom', msg)
            msg.header.stamp.sec = 99
            msg.pose.pose.position.x = 0.5
            w.odom('/odometry/filtered', msg)
            cmd = Twist()
            cmd.linear.x = 0.1
            w.command(cmd)
            cmd.linear.x = 0.0
            w.command(cmd)
            self.assertFalse(w.idle()['idle_is_lower_bound'])
            event = Log()
            event.name = 'ekf_filter_node'
            event.msg = 'reset detected'
            w.rosout(event)
            for _ in range(20):
                w.rosout(event)
            gid = bytes(range(24))
            node = SimpleNamespace(
                get_publishers_info_by_topic=lambda _: [
                    SimpleNamespace(endpoint_gid=gid, node_name='fake', node_namespace='/')
                ]
            )
            w.snapshot(node, ['/odometry/filtered'])
            node.get_publishers_info_by_topic = lambda _: []
            w.snapshot(node, ['/odometry/filtered'])
            summary = w.close('test')
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            change = next(r for r in rows if r['event'] == 'odom_change')
            self.assertEqual(change['reasons'], ['pose_jump', 'stamp_jump'])
            self.assertEqual(summary['counts']['publisher_graph'], 2)
            self.assertEqual(summary['counts']['command_state'], 2)
            self.assertGreater(summary['counts']['rosout_suppressed'], 0)
            self.assertEqual(summary['max_step_m']['/odometry/filtered'], 0.5)
            self.assertLessEqual(path.stat().st_size, 65536)

    def test_file_cap_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'probe.jsonl'
            w = Watch(path, 300)
            for _ in range(8):
                w.emit('test', text='x' * 100)
            self.assertTrue(w.full)
            w.close('file_limit')
            self.assertLessEqual(path.stat().st_size, 300)
            with self.assertRaises(FileExistsError):
                Watch(path, 300)


if __name__ == '__main__':
    unittest.main()
