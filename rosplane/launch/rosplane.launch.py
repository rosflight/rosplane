import os
import sys
import launch.actions
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Create the package directory
    rosplane_dir = get_package_share_directory('rosplane')

    # Determine the appropriate control scheme.
    control_type = "default"
    aircraft = "anaconda" # Default aircraft
    use_params = 'false'

    for arg in sys.argv:
        if arg.startswith("control_type:="):
            control_type = arg.split(":=")[1]

        if arg.startswith("aircraft:="):
            aircraft = arg.split(":=")[1]

        if arg.startswith("seed_estimator:="):
            use_params = arg.split(":=")[1].lower()

    autopilot_params = os.path.join(
        rosplane_dir,
        'params',
        aircraft + '_autopilot_params.yaml'
    )

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'command_publisher_remap',
            default_value='/command',
        ),
        Node(
            package='rosplane',
            executable='rosplane_controller',
            name='autopilot',
            parameters=[autopilot_params],
            output='screen',
            arguments=[control_type],
            remappings=[
                ('/command', launch.substitutions.LaunchConfiguration('command_publisher_remap'))
            ]
        ),
        launch.actions.DeclareLaunchArgument(
            'controller_command_publisher_remap',
            default_value='/controller_command',
        ),
        Node(
            package='rosplane',
            executable='rosplane_path_follower',
            name='path_follower',
            parameters=[autopilot_params],
            remappings=[
                ('/controller_command', launch.substitutions.LaunchConfiguration('controller_command_publisher_remap'))
            ]
        ),
        Node(
            package='rosplane',
            executable='rosplane_path_manager',
            name='path_manager',
            parameters=[autopilot_params],
        ),
        Node(
            package='rosplane',
            executable='rosplane_path_planner',
            name='path_planner',
        ),
        Node(
            package='rosplane',
            executable='rosplane_estimator_node',
            name='estimator',
            output='screen',
            parameters = [autopilot_params],
            arguments = [use_params]
        )
    ])
