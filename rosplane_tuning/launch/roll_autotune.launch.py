import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rosplane_tuning'),
                             'launch/autotune.launch.py')
            ),
            launch_arguments={
                '/autotune/stabilize_period': '5.0',
                '/autotune/current_tuning_autopilot': 'roll'
            }.items()
        )])
