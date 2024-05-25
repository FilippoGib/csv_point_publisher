import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('csv_point_publisher'),  # Use the actual name of your package
        'config',
        'csv_point_publisher_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='csv_point_publisher',  # Use the actual name of your package
            executable='csv_point_publisher_node',
            name='csv_point_publisher',
            output='screen',
            parameters=[config]
        )
    ])

