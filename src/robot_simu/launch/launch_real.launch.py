import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.actions import RegisterEventHandler
from launch.event_handlers.on_process_start import OnProcessStart
from launch_ros.substitutions import FindPackageShare
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

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("robot_description"),
                    "robot",
                    "robot.urdf.xacro",
                ]
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    controller_params_file = os.path.join(robot_simu_dir, 'config', 'controllers.yaml')

    # Lidar
    # Get the launch directory of urg_node2
    # urg_node_dir = get_package_share_directory('urg_node2')
    # urg_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(urg_node_dir, 'launch', 'urg_node2.launch.py')])
    # )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description},
                    controller_params_file],
    )

    can_rx_raw = Node(
        package='can_robot',
        executable='can_raw_rx',
        output='screen'
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

    controller_to_can = Node(
        package='my_robot_control',
        executable='control_can.py',
        output='screen'
    )

    arm_controller = Node(
        package='arm_controller',
        executable='arm_controller_server',
        output='screen'
    )

    my_robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_my_robot_control, 'launch', 'my_robot_control.launch.py')
        ),
    )

    forward_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'forward_velocity_controller'],
        output='screen'
    )

    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    enemy = Node(
        package='enemy_detection',
        executable='enemy_detection',
        output='screen'
    )

    already_started_nodes = set()
    already_started_nodes_index = 0

    def start_next_node(event, context):
        nonlocal already_started_nodes_index
        print(f'node {event.process_name} started.')
        already_started_nodes.update([event.process_name])
        if len(already_started_nodes) == 2:
            print(f'all required nodes are up, time to start node1')
        else:
            already_started_nodes_index += 1
            return [event.process_name]

    delayed_can_tx_raw = RegisterEventHandler(event_handler=OnProcessStart(target_action=can_rx_raw,
                                                                           on_start=can_tx_raw))
    delayed_can_tx = RegisterEventHandler(event_handler=OnProcessStart(target_action=can_tx_raw,
                                                                       on_start=can_tx))
    delayed_can_rx = RegisterEventHandler(event_handler=OnProcessStart(target_action=can_tx,
                                                                       on_start=can_rx))
    # delayed_urg_node = RegisterEventHandler(event_handler=OnProcessStart(target_action=can_rx,
    #                                                                      on_start=urg_node))

    delayed_controller_manager = TimerAction(period=0.5, actions=[controller_manager])

    delayed_forward_position_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=forward_position_controller,
            on_start=[forward_position_controller],
        )
    )

    delayed_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster,
            on_start=[joint_state_broadcaster],
        )
    )

    return LaunchDescription([
        rsp,
        can_rx_raw,
        delayed_can_tx_raw,
        delayed_can_tx,
        delayed_can_rx,
        # delayed_urg_node,
        controller_to_can,
        delayed_controller_manager,
        my_robot_control,
        delayed_joint_state_broadcaster,
        delayed_forward_position_controller,
        enemy,

    ])
    # Launch them all!
    # return LaunchDescription([
    #     rsp,
    #     can_rx_raw,
    #     can_rx,
    #     can_tx,
    #     can_tx_raw,
    #     urg_node,
    #     controller_to_can,
    #     delayed_controller_manager,
    #     my_robot_control,
    #     delayed_joint_state_broadcaster,
    #     delayed_forward_position_controller,
    #     enemy
    # ])

    already_started_nodes = set()
    # ordered_nodes = [rsp, can_rx_raw, can_rx, can_tx, can_tx_raw, urg_node, controller_to_can, delayed_controller_manager,
    #                  my_robot_control, delayed_joint_state_broadcaster, delayed_forward_position_controller, enemy]
    # ordered_nodes_index = 0
    # def start_next_node(event, context):
    #     nonlocal ordered_nodes_index
    #     print(f'node {event.process_name} started.')
    #     already_started_nodes.update([event.process_name])
    #     if len(already_started_nodes) == len(ordered_nodes):
    #         print(f'all required nodes are up, time to start node1')
    #         return None
    #     else:
    #         ordered_nodes_index += 1
    #         return ordered_nodes[ordered_nodes_index]
    #
    #
    # event_handler = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=rsp,
    #         on_start=start_next_node
    #     )
    # )
    # return LaunchDescription([
    #     rsp,
    #     event_handler
    # ])
