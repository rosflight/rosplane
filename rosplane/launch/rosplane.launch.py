import os
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Create the package directory
    rosplane_dir = get_package_share_directory('rosplane')

    autopilot_params = os.path.join(
        rosplane_dir,
        'params',
        'skyhunter_autopilot_params.yaml'
    )

    # Determine the appropriate control scheme.
    control_type = "default"

    for arg in sys.argv:
        if arg.startswith("control_type:="):
            control_type = arg.split(":=")[1]

    return LaunchDescription([
        Node(
            package='rosplane',
            executable='rosplane_controller',
            name='autopilot',
            parameters = [autopilot_params],
            output = 'screen',
            arguments = [control_type]
        ),
        Node(
            package='rosplane',
            executable='rosplane_path_follower',
            name='path_follower',
        ),
        Node(
            package='rosplane',
            executable='rosplane_path_manager',
            name='path_manager',
            parameters = [autopilot_params],
        ),
        Node(
            package='rosplane',
            executable='rosplane_path_planner',
            name='path_planner',
        ),
        Node(
            package='rosplane',
            executable='rosplane_estimator_node',
            name='estimator'
        )
    ])
