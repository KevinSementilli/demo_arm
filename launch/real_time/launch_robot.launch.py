import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

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

    robot_description = Command({'ros2 param get --hide-type /robot_state_publisher robot_description'})

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description' : robot_description},
                    controller_config],
        output="screen",
    )

    delayed_controller_manager = TimerAction(
        period=3.0, 
        actions={controller_manager}
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

    delayed_vel_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[vel_controller_spawner],
        )
    )

    # spawn the joint broadcaster
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output="screen",
    )

    return LaunchDescription([
        rsp,
        control_node,

        vel_controller_spawner,
        joint_broad_spawner,

        teleop_vel_node,
        delay_controllers,
    ])
