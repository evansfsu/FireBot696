"""Single launch entry point for all three FireBot696 nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('firebot'), 'config', 'firebot_params.yaml'
    )
    return LaunchDescription([
        Node(
            package='firebot',
            executable='vision_node',
            name='vision_node',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='firebot',
            executable='brain_node',
            name='brain_node',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='firebot',
            executable='arduino_bridge_node',
            name='arduino_bridge_node',
            parameters=[config],
            output='screen',
        ),
    ])
