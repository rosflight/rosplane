from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Create the package directory
    rosplane2_dir = get_package_share_directory('rosplane2')

    # Define the urdf file for visualizing the uav
    urdf_file_name = 'fixed_wing_uav.urdf'
    urdf = os.path.join(
        get_package_share_directory('rosplane2'),
        'urdf',
        urdf_file_name)

    # Flag for enabling/disabling use of simulation time instead of wall clock
    use_sim_time = True

    return LaunchDescription([
        Node(
            package='rosplane2',
            executable='rosplane2_controller',
            name='autopilot',
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
        ),
        Node (
            package = 'rosplane2_sim',
            executable='rosplane2_gazebo_truth_publisher',
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
