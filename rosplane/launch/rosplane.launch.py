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
    init_lat = "0.0" # init lat if seeding the estimator (typically not done)
    init_long = "0.0"
    init_alt = "0.0"
    init_baro_alt = "0.0"

    for arg in sys.argv:
        if arg.startswith("control_type:="):
            control_type = arg.split(":=")[1]
        
        if arg.startswith("aircraft:="):
            aircraft = arg.split(":=")[1]
        
        if arg.startswith("init_lat:="):
            init_lat = float(arg.split(":=")[1])
            assert isinstance(init_lat, float)
            init_lat = str(init_lat)

        if arg.startswith("init_long:="):
            init_long = float(arg.split(":=")[1])
            assert isinstance(init_long, float)
            init_long = str(init_long)

        if arg.startswith("init_alt:="):
            init_alt = float(arg.split(":=")[1])
            assert isinstance(init_alt, float)
            init_alt = str(init_alt)
    
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
            output='screen',
            parameters = [autopilot_params],
            arguments = [init_lat, init_long, init_alt, init_baro_alt]
        )
    ])
