#!/usr/bin/env python3
"""Bounded raw ROS evidence capture using a single SequentialWriter callback thread.

No velocity publisher. Header stamps and serialized message bytes are unchanged;
bag timestamps are local wall-clock receipt nanoseconds. This is a recorder
alternative, not a diagnosis or proven repair of a ros2 CLI abort.
"""

import argparse
import datetime
import gc
import json
import math
import os
from pathlib import Path
import shutil
import signal
import time
import traceback
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from common import singleton, write_json

TOPICS = {
    '/scan_filtered': 'sensor_msgs/msg/LaserScan',
    '/scan': 'sensor_msgs/msg/LaserScan',
    '/tf': 'tf2_msgs/msg/TFMessage',
    '/tf_static': 'tf2_msgs/msg/TFMessage',
    '/odometry/filtered': 'nav_msgs/msg/Odometry',
    '/rosbot_xl_base_controller/odom': 'nav_msgs/msg/Odometry',
    '/cmd_vel': 'geometry_msgs/msg/Twist',
    '/rosout': 'rcl_interfaces/msg/Log',
    '/image_raw/compressed': 'sensor_msgs/msg/CompressedImage',
    '/imu_broadcaster/imu': 'sensor_msgs/msg/Imu',
}
STOP_SIGNAL = None


def stop(signum, frame):
    global STOP_SIGNAL
    STOP_SIGNAL = signum


def duration(value):
    ns = value.nanoseconds
    return {'sec': ns // 1000000000, 'nsec': ns % 1000000000}


def qos_dict(profile):
    return {
        'history': int(profile.history),
        'depth': profile.depth,
        'reliability': int(profile.reliability),
        'durability': int(profile.durability),
        'deadline': duration(profile.deadline),
        'lifespan': duration(profile.lifespan),
        'liveliness': int(profile.liveliness),
        'liveliness_lease_duration': duration(profile.liveliness_lease_duration),
        'avoid_ros_namespace_conventions': profile.avoid_ros_namespace_conventions,
    }


def main():  # noqa: PLR0912, PLR0915 - bounded lifecycle kept together for audit
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=int, default=900)
    parser.add_argument(
        '--image-hz',
        type=float,
        default=0,
        help='0 keeps all images; otherwise maximum retained image Hz',
    )
    parser.add_argument(
        '--max-mib',
        type=float,
        default=0,
        help='0 disables; stop recording near this bag file MiB limit',
    )
    parser.add_argument('--root', type=Path, default=Path.home() / 'task6_writer_evidence')
    args = parser.parse_args()
    if not 1 <= args.seconds <= 1800:
        parser.error('--seconds must be 1..1800')
    if not math.isfinite(args.image_hz) or args.image_hz < 0:
        parser.error('--image-hz must be finite and nonnegative')
    if not math.isfinite(args.max_mib) or args.max_mib < 0:
        parser.error('--max-mib must be finite and nonnegative')
    owner_lock = singleton('evidence_recorder')
    args.root.mkdir(parents=True, exist_ok=True)
    floor = 300 * 1024 * 1024
    if shutil.disk_usage(args.root).free < floor:
        raise SystemExit('REFUSED: less than 300 MiB free')
    session = args.root / (
        datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + str(os.getpid())
    )
    session.mkdir()
    bag = session / 'bag'
    manifest = {
        'session': str(session),
        'pid': os.getpid(),
        'started_unix': time.time(),
        'seconds_limit': args.seconds,
        'image_hz_limit': args.image_hz,
        'bag_mib_limit': args.max_mib,
        'topics': TOPICS,
        'implementation': 'SequentialWriter_raw_single_thread',
        'timestamp_basis': 'time.time_ns at callback receipt; source headers and CDR bytes unchanged',
        'initial_free_bytes': shutil.disk_usage(session).free,
        'disk_floor_bytes': floor,
        'ros_environment': {
            k: os.environ.get(k)
            for k in ['ROS_DISTRO', 'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY']
        },
    }
    (session / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    (session / 'supervisor.pid').write_text(str(os.getpid()))
    (args.root / 'latest_session.txt').write_text(str(session))
    print('SESSION=' + str(session), flush=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)
    node = rclpy.create_node('task6_evidence_writer_' + str(os.getpid()))
    writer = rosbag2_py.SequentialWriter()
    counts = {topic: 0 for topic in TOPICS}
    data_bytes = {topic: 0 for topic in TOPICS}
    metadata = {}
    image_received, image_dropped, image_last_kept = 0, 0, None
    bag_bytes, last_stat = 0, 0
    reason, failure, ready = 'duration_limit', None, False
    begin = time.monotonic()
    try:
        writer.open(
            rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3', max_cache_size=0),
            rosbag2_py.ConverterOptions(
                input_serialization_format='cdr', output_serialization_format='cdr'
            ),
        )
        ready = True

        def callback(topic, raw):
            nonlocal image_received, image_dropped, image_last_kept
            stamp = time.time_ns()
            if topic == '/image_raw/compressed':
                image_received += 1
                now = time.monotonic()
                if (
                    args.image_hz > 0
                    and image_last_kept is not None
                    and now - image_last_kept < 1.0 / args.image_hz
                ):
                    image_dropped += 1
                    return
                image_last_kept = now
            if topic not in metadata:
                peers = node.get_publishers_info_by_topic(topic)
                profiles = [qos_dict(peer.qos_profile) for peer in peers]
                fallback = not profiles
                if fallback:
                    profiles = [
                        qos_dict(
                            QoSProfile(
                                depth=100,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL
                                if topic == '/tf_static'
                                else DurabilityPolicy.VOLATILE,
                            )
                        )
                    ]
                writer.create_topic(
                    rosbag2_py.TopicMetadata(
                        name=topic,
                        type=TOPICS[topic],
                        serialization_format='cdr',
                        offered_qos_profiles=json.dumps(profiles),
                    )
                )
                metadata[topic] = {
                    'profiles': profiles,
                    'fallback': fallback,
                    'first_receipt_ns': stamp,
                }
                (session / 'qos.json').write_text(json.dumps(metadata, indent=2))
            writer.write(topic, raw, stamp)
            counts[topic] += 1
            data_bytes[topic] += len(raw)

        for topic, msgtype in TOPICS.items():
            qos = QoSProfile(
                depth=100,
                reliability=ReliabilityPolicy.RELIABLE
                if topic == '/tf_static'
                else ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL
                if topic == '/tf_static'
                else DurabilityPolicy.VOLATILE,
            )
            node.create_subscription(
                get_message(msgtype),
                topic,
                lambda raw, name=topic: callback(name, raw),
                qos,
                raw=True,
            )
        print(
            'RECORDING: single callback thread; no keyboard handler; verify health counts before motion',
            flush=True,
        )
        last_health = 0
        while time.monotonic() - begin < args.seconds:
            if STOP_SIGNAL is not None or (session / 'STOP').exists():
                reason = 'requested_stop'
                break
            free = shutil.disk_usage(session).free
            if free < floor:
                reason = 'disk_floor'
                break
            if time.monotonic() - last_stat >= 1:
                bag_bytes = sum(path.stat().st_size for path in bag.glob('*.db3*'))
                last_stat = time.monotonic()
            if args.max_mib > 0 and bag_bytes >= args.max_mib * 1024 * 1024:
                reason = 'bag_size_limit'
                break
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - last_health >= 5:
                health = {
                    'unix_time': time.time(),
                    'elapsed_s': time.monotonic() - begin,
                    'free_bytes': free,
                    'message_counts': counts,
                    'data_bytes': data_bytes,
                    'bag_file_bytes': bag_bytes,
                    'image_received': image_received,
                    'image_dropped': image_dropped,
                    'missing_topics': [topic for topic, count in counts.items() if count == 0],
                }
                write_json(session / 'health.json', health)
                last_health = time.monotonic()
    except BaseException as error:
        reason = 'writer_exception'
        failure = {
            'type': type(error).__name__,
            'message': str(error),
            'traceback': traceback.format_exc(),
        }
        traceback.print_exc()
    finally:
        node.destroy_node()
        writer = None
        gc.collect()
        if rclpy.ok():
            rclpy.shutdown()
        final = {
            'stop_reason': reason,
            'exception': failure,
            'supervisor_signal': STOP_SIGNAL,
            'writer_opened': ready,
            'closed_metadata': (bag / 'metadata.yaml').exists(),
            'elapsed_s': time.monotonic() - begin,
            'message_counts': counts,
            'data_bytes': data_bytes,
            'bag_file_bytes': sum(path.stat().st_size for path in bag.glob('*.db3*')),
            'image_received': image_received,
            'image_dropped': image_dropped,
            'ended_unix': time.time(),
            'final_free_bytes': shutil.disk_usage(session).free,
        }
        write_json(session / 'result.json', final)
        print(json.dumps(final), flush=True)
    owner_lock.close()
    return 0 if final['closed_metadata'] and failure is None else 2


if __name__ == '__main__':
    raise SystemExit(main())
