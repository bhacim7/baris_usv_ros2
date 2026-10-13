import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Installed location of this package's config file
    params = os.path.join(
        get_package_share_directory('usv_learn'), 'config', 'learn.yaml')

    return LaunchDescription([
        Node(
            package='usv_learn',
            executable='fake_gps',
            name='fake_gps',
            output='screen',
        ),
        Node(
            package='usv_learn',
            executable='distance_monitor',
            name='distance_monitor',
            output='screen',
            parameters=[params],
            # Restart the node 2 s after it dies
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
