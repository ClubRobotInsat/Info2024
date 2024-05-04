import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from scripts import GazeboRosPaths


def generate_launch_description():

    # Get the launch directory
    robot_simu_dir = get_package_share_directory('robot_simu')
    launch_dir = os.path.join(robot_simu_dir, 'launch')

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_my_robot_control = get_package_share_directory('my_robot_control')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch configuration variables specific to this launch file
    use_rsp = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    world = LaunchConfiguration('world')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    world_file = os.path.join(robot_simu_dir, 'worlds', 'testwtable3.world')


# Declare the launch arguments
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Specify the ROS logger level')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='false',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Full path to world model file to load')

    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world, 'verbose' : 'true'}.items(),
    )

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                                            arguments=['-topic', 'robot_description',
                                                       '-entity', 'my_bot',
                                                       '-x', '0', '-y', '0', '-z', '0.1',
                                                       ],
                                            output='screen')

    my_robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_my_robot_control, 'launch', 'my_robot_control.launch.py')
        ),
    )

    pkg_robot_description = get_package_share_directory('robot_description')

    xacro_file = os.path.join(pkg_robot_description, 'robot', 'robot.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file)

    # start_robot_state_publisher_cmd = Node(
    #     condition=IfCondition(use_rsp),
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     namespace=namespace,
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     remappings=remappings)

    start_robot_state_publisher_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(launch_dir,'rsp.launch.py'
            )]), launch_arguments={'use_sim_time': 'true'}.items()
        )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz.launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': '',
                          'use_namespace': 'False'}.items())


    # controller_manager = ExecuteProcess(
    #     cmd=['ros2', 'control', 'controller_manager', 'load', 'robot_state_publisher'],
    #     output='screen'
    # )

    forward_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'forward_velocity_controller'],
        output='screen'
    )

    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    # Launch them all!
    ld = LaunchDescription()
    ld.add_action(declare_log_level_cmd)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_world_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(spawn_entity)

    # Add the actions to launch all the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)

    # Add the actions to launch controllers
    # ld.add_action(controller_manager)
    ld.add_action(joint_state_broadcaster)
    ld.add_action(forward_position_controller)

    # ld.add_action(controller_manager)
    # ld.add_action(rf2o_node)

    ld.add_action(my_robot_control)

    return ld

    # Get the Gazebo paths
    # model_path, plugin_path, media_path = GazeboRosPaths.get_paths()

    # env = {
    #     'GAZEBO_MODEL_PATH': model_path + ':',
    #     'GAZEBO_PLUGIN_PATH': plugin_path,
    #     'GAZEBO_RESOURCE_PATH': media_path
    # }


    # Get the path to robot description
    # robot_description = get_package_share_directory('robot_description')
    #
    # if 'GAZEBO_MODEL_PATH' in os.environ:
    #     model_path =  os.environ['GAZEBO_MODEL_PATH'] + ':' + robot_description
    # else:
    #     model_path =  robot_description
    #
    # update_gazebo_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path)

    # current_gazebo_model_path = os.environ['GAZEBO_MODEL_PATH']
    # updated_gazebo_model_path = current_gazebo_model_path + robot_description
    #
    # print('updated_gazebo_model_path:', updated_gazebo_model_path)
    #
    # update_gazebo_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', updated_gazebo_model_path)
    #


    # world_file = os.path.join(robot_simu_dir, 'worlds', 'testwtable.world')
    #
    # # rviz_config_file = LaunchConfiguration('rviz_config_file')
    # # use_rviz = LaunchConfiguration('use_rviz')
    #
    # declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    #     'use_robot_state_pub',
    #     default_value='True',
    #     description='Whether to start the robot state publisher')
    #
    # # declare_use_rviz_cmd = DeclareLaunchArgument(
    # #     'use_rviz',
    # #     default_value='True',
    # #     description='Whether to start RVIZ')
    #
    # rsp = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(launch_dir,'rsp.launch.py'
    #     )]), launch_arguments={'use_sim_time': 'true'}.items()
    # )
    #
    #
    # # Include the Gazebo launch file, provided by the gazebo_ros package
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    #     launch_arguments={'world': world_file, 'verbose' : 'true'}.items()
    # )
    #
    # # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    # spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
    #                     arguments=['-topic', 'robot_description',
    #                                '-entity', 'my_bot'],
    #                     output='screen')
    #
    # # rviz_cmd = IncludeLaunchDescription(
    # #     PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz.launch.py')),
    # #     condition=IfCondition(use_rviz),
    # #     launch_arguments={'namespace': '',
    # #                       'use_namespace': 'False',
    # #                       'rviz_config': rviz_config_file}.items())
    #
    #
    # forward_position_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'forward_velocity_controller'],
    #     output='screen'
    # )
    #
    # joint_state_broadcaster = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
    #     output='screen'
    # )
    #
    # # Launch them all!
    # ld = LaunchDescription()
    # # ld.add_action(update_gazebo_path)
    # ld.add_action(declare_use_robot_state_pub_cmd)
    # # ld.add_action(declare_use_rviz_cmd)
    # ld.add_action(rsp)
    # ld.add_action(gazebo)
    # ld.add_action(spawn_entity)
    # # ld.add_action(rviz_cmd)
    # ld.add_action(forward_position_controller)
    # ld.add_action(joint_state_broadcaster)
    #
    # return ld
