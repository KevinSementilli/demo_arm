import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'demo_arm'

    # Declare use_sim_time argument
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_real_time = LaunchConfiguration('use_real_time')

    
    # *******************************************
    # ****************** RVIZ *******************
    # *******************************************

    # Include rsp.launch.py
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package_name), 'launch/resources', 'rsp.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_real_time': use_real_time
        }.items()  # Pass use_sim_time as an argument
    )

    rviz_config_file = os.path.join(
        get_package_share_directory(package_name), 'config', 'config.rviz'
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )


    # *******************************************
    # ************** ROS2 Control ***************
    # *******************************************

    controller_config = os.path.join(
        get_package_share_directory(package_name), 'config', 'trajectory_controller.yaml'
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        remappings=[("/controller_manager/robot_description", "robot_description")],
        output="screen",
    )

    delayed_controller_manager = TimerAction(
        period=3.0, 
        actions={controller_manager}
    )

    # spawn the trajecotry controller
    trajecotry_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["trajectory_controller"],
        output="screen",
    )

    delayed_trajecotry_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[trajecotry_controller_spawner],
        )
    )

    # spawn the joint broadcaster
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broadcaster"],
        output="screen",
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )
    
    # **********************************************
    # ************** GAMEPAD Control ***************
    # **********************************************

    teleop_trajectory_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package_name), 'launch/resources', 'teleop_trajectory.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,}.items()  # Pass use_sim_time as an argument
    )    

    # Launch Description
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use sim time if true'
        ),

        DeclareLaunchArgument(
            'use_real_time',
            default_value='False',
            description="Choose control mode: 'sim', 'gazebo', or 'real'"
        ),

        rsp,
        node_rviz,
        delayed_controller_manager,
        delayed_trajecotry_controller_spawner,
        delayed_joint_broad_spawner,
        teleop_trajectory_node
    ])