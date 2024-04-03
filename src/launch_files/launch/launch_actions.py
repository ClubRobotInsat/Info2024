import launch
import launch_ros.actions


def generate_launch_description():
    # Create a launch description with one or more nodes
    ld = launch.LaunchDescription()

    # Add your nodes, actions, or other launch configurations here
    # For example, add a node:
    planner = launch_ros.actions.Node(
        package='planner',  # Replace 'my_package' with your package name
        executable='planner_node',
        name='planner_node',
        output='screen'
    )

    behaviour = launch_ros.actions.Node(
        package='behaviour',  # Replace 'my_package' with your package name
        executable='behaviour_node',
        name='behaviour',
        output='screen'
    )

    ld.add_action(planner)
    ld.add_action(behaviour)

    return ld
