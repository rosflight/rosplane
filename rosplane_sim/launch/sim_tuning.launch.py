from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    # Create the package directory
    rosplane_share = FindPackageShare("rosplane")

    base_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rosplane_share, "launch", "rosplane_tuning.launch.py"])
        )
    )

    return LaunchDescription([
        base_launch_include,
        Node (
            package = 'rosplane_sim',
            executable='sim_state_transcriber',
            name='rosplane_truth'
        )
    ])

