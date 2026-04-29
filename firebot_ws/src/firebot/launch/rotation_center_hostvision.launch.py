"""Host live vision (UDP) + rotation-center test drive (no brain, no vision_node).

Use when ``rpi_test_yolo_fire.py --udp-bridge 127.0.0.1:PORT`` runs on the Pi host
and Docker should consume ``/fire/detection`` via ``udp_detection_bridge_node``.

Port: ``FIREBOT_UDP_DETECTION_PORT`` (default in this file: 7766 if unset or invalid).

Docker (merge compose):

  docker compose -f docker-compose.yml -f compose.host-vision-udp-rotation.yml up -d

Host:

  python3 scripts/rpi_test_yolo_fire.py --video-mode --udp-bridge 127.0.0.1:7766
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('firebot'), 'config', 'firebot_params.yaml'
    )
    udp_port_s = os.environ.get('FIREBOT_UDP_DETECTION_PORT', '7766').strip()
    try:
        udp_port = int(udp_port_s)
    except ValueError:
        udp_port = 7766

    return LaunchDescription(
        [
            Node(
                package='firebot',
                executable='udp_detection_bridge_node',
                name='udp_detection_bridge',
                parameters=[{'listen_port': udp_port}],
                output='screen',
            ),
            Node(
                package='firebot',
                executable='arduino_bridge_node',
                name='arduino_bridge_node',
                parameters=[config],
                output='screen',
            ),
            Node(
                package='firebot',
                executable='rotation_center_test_node',
                name='rotation_center_test',
                parameters=[config],
                output='screen',
            ),
        ]
    )
