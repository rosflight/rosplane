import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    input_commands_remap = '/path_follower_controller_commands'
    output_commands_remap = '/input_mapper_controller_commands'

    return LaunchDescription([
        Node(
            package='rosplane',
            executable='input_mapper',
            name='input_mapper',
            output='screen',
            remappings=[
                ('/controller_commands', input_commands_remap),
                ('/mixed_commands', output_commands_remap)
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('rosplane'),
                    'launch/rosplane.launch.py'
                )
            ),
            launch_arguments={
                'controller_commands_publisher_remap': input_commands_remap,
                'controller_commands_subscriber_remap': output_commands_remap
            }.items()
        )
    ])
