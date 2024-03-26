import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Launch arguments
    stabilize_period = LaunchConfiguration('stabilize_period')
    stabilize_period_launch_arg = DeclareLaunchArgument(
        'stabilize_period',
        description='Amount of time to collect data for calculating error'
    )
    continuous_tuning = LaunchConfiguration('continuous_tuning')
    continuous_tuning_launch_arg = DeclareLaunchArgument(
        'continuous_tuning',
        description='Whether to run the tuning sequence continuously or to wait for manual input'
    )
    current_tuning_autopilot = LaunchConfiguration('current_tuning_autopilot')
    current_tuning_autopilot_launch_arg = DeclareLaunchArgument(
        'current_tuning_autopilot',
        description='Autopilot to tune'
    )

    # Launch nodes
    autotune_node = Node(
        package='rosplane_tuning',
        executable='autotune.py',
        output='screen',
        parameters=[
            {'stabilize_period': stabilize_period},
            {'continuous_tuning': continuous_tuning},
            {'current_tuning_autopilot': current_tuning_autopilot}
        ]
    )
    rosplane_tuning_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rosplane_tuning'),
                         'launch/rosplane_tuning.launch.py')
        )
    )

    return LaunchDescription([
        stabilize_period_launch_arg,
        continuous_tuning_launch_arg,
        current_tuning_autopilot_launch_arg,
        autotune_node,
        rosplane_tuning_launch_include
        ])
