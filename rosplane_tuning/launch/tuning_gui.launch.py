from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rosplane_tuning_share = FindPackageShare('rosplane_tuning')

    tuning_config = PathJoinSubstitution(
        [rosplane_tuning_share, 'resources', 'param_tuning_config.yaml']
    )

    return LaunchDescription(
        [
            Node(
                package='rosflight_rqt_plugins',
                executable='param_tuning',
                name='tuning_gui',
                output='screen',
                arguments=['--config-filepath', tuning_config],
            )
        ]
    )
