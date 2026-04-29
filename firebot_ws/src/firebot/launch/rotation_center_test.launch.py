"""Minimal stack for rotation-center field test: bridge + test node only.

**Do not** launch ``brain_node`` at the same time — both would publish ``/cmd/drive``.

For **host live preview** (``rpi_test_yolo_fire.py --udp-bridge``), use
``rotation_center_hostvision.launch.py`` or
``docker compose ... -f compose.host-vision-udp-rotation.yml`` instead — this file has **no** UDP bridge, so ``/fire/detection`` will be empty unless vision runs in the same ROS graph.

Run with vision + detection on ``/fire/detection`` (e.g. full firebot in Docker minus brain,
or host ``rpi_test_yolo_fire.py --udp-bridge`` + ``udp_detection_bridge``).

Examples::

  ros2 launch firebot rotation_center_test.launch.py

Tune ``pulse_rotate_sec``, ``pulse_min_interval_sec``, ``center_band_frac`` in
``config/firebot_params.yaml`` under ``rotation_center_test``.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('firebot'), 'config', 'firebot_params.yaml'
    )
    return LaunchDescription(
        [
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
