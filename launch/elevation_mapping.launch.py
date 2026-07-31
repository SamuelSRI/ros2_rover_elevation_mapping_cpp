import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "ros2_rover_elevation_mapping_cpp"
    config_file = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "elevation_mapping.yaml",
    )

    return LaunchDescription([
        Node(
            package=package_name,
            executable="elevation_mapping_node",
            name="elevation_mapping_node",
            output="screen",
            parameters=[config_file],
        )
    ])
