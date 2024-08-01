import os
import sys
from launch import LaunchDescription
from launch.descriptions import executable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Create the package directory
    rosplane_dir = get_package_share_directory('rosplane')

    # Determine the appropriate control scheme.
    control_type = "default"
    aircraft = "anaconda" # Default aircraft

    for arg in sys.argv:
        if arg.startswith("control_type:="):
            control_type = arg.split(":=")[1]

        if arg.startswith("aircraft:="):
            aircraft = arg.split(":=")[1]

    autopilot_params = os.path.join(
        rosplane_dir,
        'params',
        aircraft + '_autopilot_params.yaml'
    )


    return LaunchDescription([
        Node(
            package='rosplane',
            executable='rosplane_controller',
            name='autopilot',
            parameters = [autopilot_params,
                          {'pitch_tuning_debug_override': False},
                          {'roll_tuning_debug_override': True}],
            output = 'screen',
            arguments = [control_type]
        ),
        Node(
            package='rosplane',
            executable='rosplane_estimator_node',
            name='estimator'
        ),
        Node(
            package='rosplane_tuning',
            executable='signal_generator',
            name='signal_generator',
            output = 'screen'
        )
    ])
