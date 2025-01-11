import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    package_name='demo_arm'

    # launch robot_state_publisher 
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), 'launch/resources' ,'rsp.launch.py')]), 
            launch_arguments={'use_sim_time': 'false'}.items()
    )

    teleop_vel_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), 'launch/resources' ,'teleop_vel.launch.py')]), 
    )

    controller_config = os.path.join(
        get_package_share_directory(package_name), 'config', 'vel_controller.yaml'
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        output="screen",
    )

    # spawn the velocity controller
    vel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["vel_controller"],
        remappings=[
            ("/vel_controller/commands", "/teleop_vel")
        ],
        output="screen",
    )

    # spawn the joint broadcaster
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output="screen",
    )

    delay_controllers = TimerAction(
        period=10.0,
        actions=[
            vel_controller_spawner,
            joint_broad_spawner,
        ]
    )

    return LaunchDescription([
        rsp,
        control_node,
        teleop_vel_node,
        delay_controllers,
    ])
