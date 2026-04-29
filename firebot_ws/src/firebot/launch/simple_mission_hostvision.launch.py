"""Host live vision over UDP + brain simple-mission stack.

Starts **udp_detection_bridge_node**, **arduino_bridge_node**, and **brain_node**
(no ``vision_node`` inside this stack). Run YOLO + Picamera on the **host**
and forward detections into ROS:

  python3 scripts/rpi_test_yolo_fire.py --video-mode --udp-bridge 127.0.0.1:PORT

Use ``config/firebot_params.yaml`` to tune ``brain_node`` (including
``simple_mission_flow`` and ``center_hold_before_warning_sec``).

UDP listen port: environment variable ``FIREBOT_UDP_DETECTION_PORT`` (default **7766**).

Docker (from ``docker/``)::

  docker compose -f docker-compose.yml -f compose.host-vision-udp-simple.yml up -d

Single host command (repo root; starts Docker detached, then vision in foreground)::

  ./scripts/run_simple_mission_hostvision.sh

Native ROS on machine (no Docker)::

  ros2 launch firebot simple_mission_hostvision.launch.py
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
                executable='brain_node',
                name='brain_node',
                parameters=[config],
                output='screen',
            ),
        ]
    )
