"""Local Gazebo-only fixture: known flat wall and independent contact sensor."""

import os
from pathlib import Path
import xml.etree.ElementTree as ET
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    if (
        os.environ.get('ROS_DOMAIN_ID') != '184'
        or os.environ.get('IGN_PARTITION') != 'cp18_turn_audit'
    ):
        raise RuntimeError('Dedicated local simulation domain/partition required')
    out = Path(os.environ['TURN_AUDIT_OUT'])
    gap = float(os.environ['TURN_AUDIT_WALL_Y'])
    base = Path.home() / 'checkpoint18_assets/local_support/route_test/test_world.sdf'
    root = ET.parse(base).getroot()
    world = root.find('world')
    for model in list(world.findall('model')):
        if model.get('name') != 'ground_plane':
            world.remove(model)
    wall = ET.fromstring(f"""<model name="audit_wall"><static>true</static>
    <pose>0 {gap + 0.025} .25 0 0 0</pose><link name="wall">
    <collision name="solid"><geometry><box><size>3 .05 .5</size></box></geometry></collision>
    <visual name="visible"><geometry><box><size>3 .05 .5</size></box></geometry></visual>
    <sensor name="wall_contacts" type="contact"><always_on>true</always_on><update_rate>100</update_rate>
    <contact><collision>solid</collision><topic>/turn_audit/contacts</topic></contact></sensor>
    </link></model>""")
    world.append(wall)
    ground_link = world.find("model[@name='ground_plane']/link")
    ground_link.append(
        ET.fromstring("""<sensor name="heartbeat" type="contact">
    <always_on>true</always_on><update_rate>100</update_rate><contact>
    <collision>collision</collision><topic>/turn_audit/contacts</topic>
    </contact></sensor>""")
    )
    path = out / 'world.sdf'
    ET.ElementTree(root).write(path)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('ros_gz_sim')) / 'launch/gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'--headless-rendering -s -v 2 -r {path}',
            'on_exit_shutdown': 'True',
        }.items(),
    )
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('rosbot_xl_gazebo')) / 'launch/spawn.launch.py')
        ),
        launch_arguments={
            'namespace': '',
            'mecanum': 'True',
            'camera_model': 'None',
            'lidar_model': 'slamtec_rplidar_s1',
            'x': '0',
            'y': '0',
            'z': '.2',
            'yaw': '0',
        }.items(),
    )
    return LaunchDescription([gazebo, robot])
