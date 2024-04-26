import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    # Get the launch directory
    robot_simu_dir = get_package_share_directory('robot_simu')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    rviz_config_file = LaunchConfiguration('rviz_config')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='navigation',
        description='Top-level namespace'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(robot_simu_dir, 'rviz', 'default_view2.rviz'),
        description='Full path to the RViz config file to use'
    )

    # Launch rviz
    start_rviz_cmd = Node(
        condition=UnlessCondition(use_namespace),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    namespaced_rviz_config_file = ReplaceString(
        source_file=rviz_config_file,
        replacements={'<robot_namespace>': ('/', namespace)})

    start_namespaced_rviz_cmd = Node(
        condition=IfCondition(use_namespace),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', namespaced_rviz_config_file],
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/goal_pose', 'goal_pose'),
                    ('/clicked_point', 'clicked_point'),
                    ('/initialpose', 'initialpose')])

    exit_event_handler = RegisterEventHandler(
        condition=UnlessCondition(use_namespace),
        event_handler=OnProcessExit(
            target_action=start_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))
        ))

    exit_event_handler_namespaced = RegisterEventHandler(
        condition=IfCondition(use_namespace),
        event_handler=OnProcessExit(
            target_action=start_namespaced_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))
        ))

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Add any conditioned actions
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_namespaced_rviz_cmd)

    # Add the exit event handlers
    ld.add_action(exit_event_handler)
    ld.add_action(exit_event_handler_namespaced)

    return ld
