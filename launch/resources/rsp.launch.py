import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_real_time = LaunchConfiguration('use_real_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('demo_arm'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')

    robot_description_config = Command(['xacro ', xacro_file, ' use_real_time:=', use_real_time, ' sim_mode:=', use_sim_time])
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use sim time if true'),
    
        DeclareLaunchArgument(
            'use_real_time',
            default_value='False',
            description="Control mode: 'sim', 'gazebo', or 'real'"
        ),

        node_robot_state_publisher
    ])