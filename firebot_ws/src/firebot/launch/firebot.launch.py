"""Single launch entry point for all three FireBot696 nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('firebot'), 'config', 'firebot_params.yaml'
    )
    udp_port_s = os.environ.get('FIREBOT_UDP_DETECTION_PORT', '').strip()
    udp_port = None
    if udp_port_s:
        try:
            udp_port = int(udp_port_s)
        except ValueError:
            udp_port = None
    skip_vision = os.environ.get('FIREBOT_SKIP_VISION', '').strip().lower() in (
        '1',
        'true',
        'yes',
    )
    if udp_port is not None:
        skip_vision = True

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
    if udp_port is not None:
        nodes.append(
            Node(
                package='firebot',
                executable='udp_detection_bridge_node',
                name='udp_detection_bridge',
                parameters=[{'listen_port': udp_port}],
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
