import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
   
    joy_node = Node(
        package="joy",
        executable="joy_node",
    )
    
    joy_teleop_button_dir = get_package_share_directory('joy_teleop')
    launch_joy_teleop_button = os.path.join(joy_teleop_button_dir, 'launch')

    joy_teleop_button = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(launch_joy_teleop_button, 'joy_teleop_atlas.launch.py')]),
    )

    # teleop_twist = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(['teleop_launch.launch.py']),
    # )
    
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[os.path.join(get_package_share_directory('teleop_atlas'), 'config', 'teleop_twist_joy_atlas.yaml')]
    )

    return LaunchDescription([
        joy_node,
        joy_teleop_button
    ])