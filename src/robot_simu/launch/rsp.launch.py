import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('robot_simu'))
    pkg_desc = os.path.join(get_package_share_directory('robot_description'))

    # xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    xacro_file= os.path.join(pkg_desc,'robot','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                    'robot_description': robot_description_config.toxml(),
                    'use_sim_time': use_sim_time
                    }],
            arguments=[xacro_file],
        )
    ])