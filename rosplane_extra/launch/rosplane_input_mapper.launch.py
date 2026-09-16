from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    input_controller_command_remap = '/path_follower_out_controller_command'
    mapped_controller_command_remap = '/controller_command'
    input_command_remap = '/autopilot_out_command'
    mapped_command_remap = '/command'

    return LaunchDescription([
        Node(
            package='rosplane_extra',
            executable='input_mapper',
            name='input_mapper',
            output='screen',
            remappings=[
                ('/input_controller_command', input_controller_command_remap),
                ('/mapped_controller_command', mapped_controller_command_remap),
                ('/input_command', input_command_remap),
                ('/mapped_command', mapped_command_remap)
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('rosplane'),
                    'launch/rosplane.launch.py'
                ])
            ),
            launch_arguments={
                'controller_command_publisher_remap': input_controller_command_remap,
                'command_publisher_remap': input_command_remap
            }.items()
        )
    ])
