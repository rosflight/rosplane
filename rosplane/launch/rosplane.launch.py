import os
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Create the package directory
    rosplane_dir = get_package_share_directory('rosplane')

    # Determine the appropriate control scheme.
    control_type = "default"
    aircraft = "skyhunter" # Default aircraft 

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
            parameters = [autopilot_params],
            output = 'screen',
            arguments = [control_type]
        ),
        Node(
            package='rosplane',
            executable='rosplane_path_follower',
            name='path_follower',
            parameters = [autopilot_params],
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
            name='estimator',
            parameters = [autopilot_params]
        )
    ])
