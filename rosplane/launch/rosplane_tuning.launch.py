import os
import sys
from launch import LaunchDescription
from launch.descriptions import executable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Create the package directory
    rosplane_dir = get_package_share_directory('rosplane')

    autopilot_params = os.path.join(
        rosplane_dir,
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
            package='rosplane',
            executable='rosplane_controller',
            name='autopilot',
            parameters = [autopilot_params,
                          {'tuning_debug_override': False}],
            output = 'screen',
            arguments = [control_type]
        ),
        Node(
            package='rosplane',
            executable='rosplane_estimator_node',
            name='estimator'
        ),
        Node(
            package='rosplane',
            executable='tuning_signal_generator',
            name='signal_generator',
            output = 'screen'
        )
    ])
