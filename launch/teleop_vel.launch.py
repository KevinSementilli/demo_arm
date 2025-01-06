import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    package_name='demo_arm'

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

    teleop_vel_node = Node(
        package='robot_teleop',
        executable='teleop_vel',
        name='teleop_vel',
        output='screen',
        parameters=[os.path.join(get_package_share_directory(package_name), 'config', 'joy_config.yaml')]
    )

    return LaunchDescription([
        joy_node,
        teleop_vel_node
    ])