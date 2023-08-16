import os
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Create the package directory
    rosplane2_dir = get_package_share_directory('rosplane2')

    autopilot_params = os.path.join(
        rosplane2_dir,
        'params',
        'succesive_loop_controller_params.yaml'
    )

    # Determine the appropriate control scheme.
    control_type = "default"

    for arg in sys.argv:
        if arg.startswith("control_type:="):
            control_type = arg.split(":=")[1]

    return LaunchDescription([
        Node(
            package='rosplane2',
            executable='rosplane2_controller',
            name='autopilot',
            parameters = [autopilot_params],
            output = 'screen',
            arguments = [control_type]
        ),
        Node(
            package='rosplane2',
            executable='rosplane2_path_follower',
            name='path_follower',
        ),
        Node(
            package='rosplane2',
            executable='rosplane2_path_manager',
            name='path_manager',
        ),
        Node(
            package='rosplane2',
            executable='rosplane2_path_planner',
            name='path_planner',
        ),
        Node(
            package='rosplane2',
            executable='rosplane2_estimator_node',
            name='estimator'
        )
    ])
