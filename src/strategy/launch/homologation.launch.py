import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node

def generate_launch_description():

    # Get the launch directory of robot_simu
    robot_simu_dir = get_package_share_directory('robot_simu')
    robot_simu_launch_dir = os.path.join(robot_simu_dir, 'launch')

    # Launch launch_real.launch.py
    launch_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(robot_simu_launch_dir, 'launch_real.launch.py')])
    )

    # Start homologation node
    homologation = Node(
        package='strategy',
        executable='homologation',
        name='homologation',
        output='screen'
    )

    return LaunchDescription([
        launch_real,
        homologation
    ])
