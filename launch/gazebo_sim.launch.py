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
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # launch the gazebo launch file provided with the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', 
                        executable='spawn_entity.py',
                        arguments=['-topic',
                                   'robot_description',
                                   '-entity',
                                   'my_bot'],
                        output='screen'
    )

    # launch the joy_node that outputs gamepad interpretation into the /joy topic
    joy_node_spawner = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

    # launch the custom node to convert gamepad interpretation into velocity commands
    gamepad_node = Node(
        package='robot_teleop',
        executable='teleop_jointjog_joy',
        name='teleop_jointjog_joy',
        output='screen',
        parameters=[os.path.join(get_package_share_directory, 'config', 'joy_config.yaml')]
    )

    # spawn the velocity controller
    vel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_vel_cont"],
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        vel_controller_spawner,
        joy_node_spawner,
        gamepad_node
    ])

