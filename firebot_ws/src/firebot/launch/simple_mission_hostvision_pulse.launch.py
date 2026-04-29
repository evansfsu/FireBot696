"""Same as simple_mission_hostvision, but **pulse** seek/track (rotation_center_test style).

Bursts: ``simple_pulse_rotate_sec`` on, ``simple_pulse_rest_sec`` off, at least
``simple_pulse_min_interval_sec`` between burst starts. Seek direction can alternate
(``simple_pulse_seek_alternate``).

Docker::

    docker compose -f docker-compose.yml -f compose.host-vision-udp-simple-pulse.yml up -d
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
                        'idle_exit_min_fire_sec': 3.0,
                        'simple_fire_confirm_sec': 12.0,
                        'lost_fire_grace_sec': 5.0,
                        'center_hold_before_warning_sec': 15.0,
                        'simple_mission_center_band_frac': 0.70,
                        'warning_seconds': 10,
                        'corner_exit_forward_sec': 2.0,
                        'search_timeout_sec': 15.0,
                        'simple_search_timeout_sec': 15.0,
                        'simple_progress_log_period_sec': 1.0,
                        'simple_seek_mode': 'pulse',
                        'simple_pulse_rotate_sec': 0.25,
                        'simple_pulse_rest_sec': 0.35,
                        'simple_pulse_min_interval_sec': 3.0,
                        'simple_pulse_seek_alternate': True,
                        'simple_pulse_seek_sign': 1.0,
                    },
                ],
                output='screen',
            ),
        ]
    )
