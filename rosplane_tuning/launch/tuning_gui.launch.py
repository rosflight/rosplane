import os
import sys
from launch import LaunchDescription
from launch.descriptions import executable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rosplane_tuning_dir = get_package_share_directory('rosplane_tuning')

    tuning_config = os.path.join(
        rosplane_tuning_dir,
        'resources',
        'param_tuning_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='rosflight_rqt_plugins',
            executable='param_tuning',
            name='tuning_gui',
            output='screen',
            arguments=['--config-filepath', tuning_config]
        )
    ])

