from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os

def generate_launch_description():
    # Create the package directory
    rosplane_dir = get_package_share_path('rosplane')

    # Define the urdf file for visualizing the uav
    urdf_file_name = 'fixed_wing_uav.urdf'
    urdf = rosplane_dir / 'urdf' / urdf_file_name

    # Flag for enabling/disabling use of simulation time instead of wall clock
    use_sim_time = True

    base_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rosplane_dir / 'launch' / 'rosplane.launch.py')
    )

    return LaunchDescription([
        base_launch_include,
        Node (
            package = 'rosplane_sim',
            executable='sim_state_transcriber',
            name='rosplane_truth'
        )
    ])
