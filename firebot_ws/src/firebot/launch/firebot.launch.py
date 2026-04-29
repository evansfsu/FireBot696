"""Single launch entry point for all three FireBot696 nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('firebot'), 'config', 'firebot_params.yaml'
    )
    skip_vision = os.environ.get('FIREBOT_SKIP_VISION', '').strip().lower() in (
        '1',
        'true',
        'yes',
    )
    nodes = []
    if not skip_vision:
        nodes.append(
            Node(
                package='firebot',
                executable='vision_node',
                name='vision_node',
                parameters=[config],
                output='screen',
            )
        )
    nodes.extend(
        [
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
        ]
    )
    return LaunchDescription(nodes)
