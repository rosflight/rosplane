import os
import sys
from launch import LaunchDescription
from launch.descriptions import executable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Create the package directory
    rosplane_dir = get_package_share_directory('rosplane')

    return LaunchDescription([
        Node(
            package='rosplane_tuning',
            executable='tuning_gui.py',
            name='tuning_gui_node',
            output = 'screen'
        )
    ])
