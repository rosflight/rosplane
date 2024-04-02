import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    stabilize_period = LaunchConfiguration('stabilize_period')
    stabilize_period_launch_arg = DeclareLaunchArgument(
        'stabilize_period',
        default_value=TextSubstitution(text='5.0'),
        description='Whether to run the tuning sequence continuously or to wait for manual input'
    )
    continuous_tuning = LaunchConfiguration('continuous_tuning')
    continuous_tuning_launch_arg = DeclareLaunchArgument(
        'continuous_tuning',
        default_value=TextSubstitution(text='False'),
        description='Whether to run the tuning sequence continuously or to wait for manual input'
    )
    rosplane_tuning_launch_arg = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rosplane_tuning'),
                         'launch/autotune.launch.py')
        ),
        launch_arguments={
            'stabilize_period': stabilize_period,
            'continuous_tuning': continuous_tuning,
            'current_tuning_autopilot': 'pitch'
        }.items()
    )

    return LaunchDescription([
        stabilize_period_launch_arg,
        continuous_tuning_launch_arg,
        rosplane_tuning_launch_arg
        ])
