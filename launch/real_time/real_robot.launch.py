import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    package_name='demo_arm'

    # launch robot_state_publisher with sim_time set to true
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), 'launch/resources' ,'rsp.launch.py')]), 
            launch_arguments={'use_sim_time': 'false'}.items()
    )

    teleop_vel_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), 'launch/resources' ,'teleop_vel.launch.py')]), 
            launch_arguments={'use_sim_time': 'true'}.items()
    )

    # spawn the velocity controller
    vel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["vel_controller"],
    )

    # spawn the joint broadcaster
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    return LaunchDescription([
        rsp,
        teleop_vel_node,
        vel_controller_spawner,
        joint_broad_spawner
    ])
