"""
from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    teleop_twist = Node(
        package='teleop_twist_joy',
        executable='teleop_twist',
        name='teleop_twist',
        parameters=[os.path.join(get_package_share_directory('joy_teleop'), 'config', 'joy_teleop_twist.yaml')]
    )
"""