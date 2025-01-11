import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnShutdown
from launch.actions import ExecuteProcess


def generate_launch_description():

    package_name = 'demo_arm'

    # Launch robot_state_publisher with sim_time set to true
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), 'launch/resources', 'rsp.launch.py')]), 
            launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Launch the Gazebo launch file provided with the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # Run the spawner node from the gazebo_ros package
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'robotic_arm'],
        output='screen'
    )

    # Spawn the velocity controller
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

    # Delay the controllers by 30 seconds
    delayed_controllers = TimerAction(
        period=30.0,  # 30-second delay
        actions=[
            joint_vel_spawner,
            joint_broad_spawner,
        ]
    )

    # Register shutdown event handler to clean up processes
    cleanup = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(cmd=['pkill', 'gzserver'], shell=False),
                ExecuteProcess(cmd=['pkill', 'gzclient'], shell=False),
                ExecuteProcess(cmd=['pkill', 'ros2'], shell=False),
            ]
        )
    )

    return LaunchDescription([
        rsp,

        gazebo,
        spawn_entity,

        delayed_controllers,  # Add delayed controllers
        cleanup,  # Add the cleanup event handler
    ])
