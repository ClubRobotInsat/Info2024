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
    # Get the launch directory
    robot_simu_dir = get_package_share_directory('robot_simu')
    launch_dir = os.path.join(robot_simu_dir, 'launch')

    pkg_my_robot_control = get_package_share_directory('my_robot_control')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(launch_dir, 'rsp.launch.py')]),
        launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )



    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(robot_simu_dir, 'config', 'controllers.yaml')

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description},
                    controller_params_file],
    )

    can_rx = Node(
        package='can_robot',
        executable='can_rx',
        output='screen'
    )

    can_tx = Node(
        package='can_robot',
        executable='can_tx',
        output='screen'
    )

    can_tx_raw = Node(
        package='can_robot',
        executable='can_raw_tx',
        output='screen'
    )

    contoller_to_can = Node(
        package='my_robot_control',
        executable='control_can.py',
        output='screen'
    )
    delayed_controller_manager = TimerAction(period=1.5, actions=[controller_manager])

    my_robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_my_robot_control, 'launch', 'my_robot_control.launch.py')
        ),
    )

    forward_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'forward_velocity_controller'],
        output='screen'
    )

    delayed_forward_position_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=forward_position_controller,
            on_start=[forward_position_controller],
        )
    )


    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    delayed_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster,
            on_start=[joint_state_broadcaster],
        )
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        # can_rx,
        can_tx,
        can_tx_raw,
        contoller_to_can,
        delayed_controller_manager,
        my_robot_control,
        delayed_joint_state_broadcaster,
        delayed_forward_position_controller
    ])

