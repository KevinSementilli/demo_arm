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
            [os.path.join(get_package_share_directory(package_name), 'launch', 'resources' ,'rsp.launch.py')]), 
            launch_arguments={'use_sim_time': 'true'}.items()
    )

    # launch the gazebo launch file provided with the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'robotic_arm'],
        output='screen'
    )

    # spawn the velocity controller
    joint_vel_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["vel_controller"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )


    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        joint_vel_spawner,
        joint_broad_spawner
    ])