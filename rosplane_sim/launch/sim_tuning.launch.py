from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os

def generate_launch_description():
    # Create the package directory
    rosplane_dir = get_package_share_directory('rosplane')

    # Define the urdf file for visualizing the uav
    urdf_file_name = 'fixed_wing_uav.urdf'
    urdf = os.path.join(
        get_package_share_directory('rosplane'),
        'urdf',
        urdf_file_name)

    # Flag for enabling/disabling use of simulation time instead of wall clock
    use_sim_time = True

    base_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosplane'),
                'launch/rosplane_tuning.launch.py'
            )
        )
    )

    return LaunchDescription([
        base_launch_include,
        Node (
            package = 'rosplane_sim',
            executable='rosplane_gazebo_truth_publisher',
            name='gazebo_truth'
        ),
        Node(
            package='data_viz',
            executable='viz_data',
            name='uav_plotter',
            parameters=[{
                'use_sim_time': False,
                't_horizon': 100.,
                'plot_sensors': False
            }],
            output='screen'
        )
    ])

