import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan, CompressedImage, Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

rclpy.init()
e = SingleThreadedExecutor()
n = rclpy.create_node('recorder_fixture')
e.add_node(n)
for name in [
    'rosbot_xl_base_controller',
    'ekf_filter_node',
    'sllidar_node',
    'scan_to_scan_filter_chain',
]:
    node = rclpy.create_node(name)
    node.declare_parameter('fixture_only', True)
    e.add_node(node)
pubs = {}
for topic, kind in [
    ('/scan', LaserScan),
    ('/scan_filtered', LaserScan),
    ('/image_raw/compressed', CompressedImage),
    ('/odometry/filtered', Odometry),
    ('/rosbot_xl_base_controller/odom', Odometry),
    ('/imu_broadcaster/imu', Imu),
    ('/tf', TFMessage),
]:
    pubs[topic] = n.create_publisher(kind, topic, 10)
static = n.create_publisher(
    TFMessage, '/tf_static', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
)
t = TransformStamped()
t.header.frame_id = 'base_link'
t.child_frame_id = 'laser'
t.transform.rotation.w = 1.0
static.publish(TFMessage(transforms=[t]))
image = CompressedImage(format='fixture bytes only - not a decodable image', data=bytes(26000))
image.header.frame_id = 'fixture_camera'
scan = LaserScan(
    angle_min=-3.14,
    angle_max=3.14,
    angle_increment=6.28 / 720,
    range_min=0.1,
    range_max=10.0,
    ranges=[1.0] * 720,
)
scan.header.frame_id = 'laser'
odom = Odometry()
odom.header.frame_id = 'odom'
odom.child_frame_id = 'base_link'
odom.pose.pose.orientation.w = 1.0
imu = Imu()
imu.header.frame_id = 'base_link'
count = 0


def tick():
    global count
    stamp = n.get_clock().now().to_msg()
    odom.header.stamp = stamp
    imu.header.stamp = stamp
    pubs['/odometry/filtered'].publish(odom)
    pubs['/rosbot_xl_base_controller/odom'].publish(odom)
    pubs['/imu_broadcaster/imu'].publish(imu)
    if count % 2 == 0:
        image.header.stamp = stamp
        scan.header.stamp = stamp
        pubs['/image_raw/compressed'].publish(image)
        pubs['/scan'].publish(scan)
        pubs['/scan_filtered'].publish(scan)
    t.header.stamp = stamp
    pubs['/tf'].publish(TFMessage(transforms=[t]))
    count += 1


n.create_timer(0.05, tick)
try:
    e.spin()
except KeyboardInterrupt:
    pass
finally:
    e.shutdown()
    rclpy.shutdown()
