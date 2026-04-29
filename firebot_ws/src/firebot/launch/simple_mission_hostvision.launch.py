"""Host live vision over UDP + brain simple-mission stack.

Starts **udp_detection_bridge_node**, **arduino_bridge_node**, and **brain_node**
(no ``vision_node`` inside this stack). Run YOLO + Picamera on the **host**
and forward detections into ROS:

  python3 scripts/rpi_test_yolo_fire.py --video-mode --udp-bridge 127.0.0.1:PORT

Use this launch for the **simple mission** state machine. It overrides ``brain_node``
params: 2 s corner-exit forward when entering SEARCHING, 0.5 s stable fire to leave IDLE,
12 s fire to arm, 5 s grace when lost, in-place spin to reacquire, 15 s centered (after arm)
→ WARNING, 10 s WARNING → extinguish, 300 s SEARCHING cap before timeout to IDLE.

Monitor timers: ``docker compose ... logs -f`` — look for lines prefixed with ``simple:``.

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
                parameters=[
                    config,
                    {
                        'simple_mission_flow': True,
                        'idle_exit_min_fire_sec': 0.5,
                        'simple_fire_confirm_sec': 12.0,
                        'lost_fire_grace_sec': 5.0,
                        'center_hold_before_warning_sec': 15.0,
                        'warning_seconds': 10,
                        'corner_exit_forward_sec': 2.0,
                        'simple_search_timeout_sec': 300.0,
                        'simple_progress_log_period_sec': 1.0,
                    },
                ],
                output='screen',
            ),
        ]
    )
