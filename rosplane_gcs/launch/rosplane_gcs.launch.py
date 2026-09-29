from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Create the package directory
    rosplane_gcs_share = FindPackageShare('rosplane_gcs')

    rviz2_config_file = PathJoinSubstitution([rosplane_gcs_share, 'config', 'rosplane_gcs.rviz'])
    rviz2_splash_file = PathJoinSubstitution([rosplane_gcs_share, 'resource', 'logo.png'])

    return LaunchDescription(
        [
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=[
                    *['--x', '0'],
                    *['--y', '0'],
                    *['--z', '0'],
                    *['--yaw', '0'],
                    *['--pitch', '0'],
                    *['--roll', '3.1415926535'],
                    *['--frame-id', 'world'],
                    *['--child-frame-id', 'NED'],
                ],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=[
                    *['--x', '0'],
                    *['--y', '0'],
                    *['--z', '0'],
                    *['--yaw', '-1.570796326'],
                    *['--pitch', '0'],
                    *['--roll', '3.1415926535'],
                    *['--frame-id', 'aircraft_body'],
                    *['--child-frame-id', 'stl_frame'],
                ],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz2_config_file, '-s', rviz2_splash_file],
            ),
            Node(package='rosplane_gcs', executable='rviz_waypoint_publisher'),
        ]
    )
