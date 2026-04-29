"""Minimal stack for rotation-center field test: bridge + test node only.

**Do not** launch ``brain_node`` at the same time — both would publish ``/cmd/drive``.

Run with vision + detection on ``/fire/detection`` (e.g. full firebot in Docker minus brain,
or host ``rpi_test_yolo_fire.py --udp-bridge`` + ``udp_detection_bridge``).

Examples::

  ros2 launch firebot rotation_center_test.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='firebot',
                executable='arduino_bridge_node',
                name='arduino_bridge_node',
                output='screen',
            ),
            Node(
                package='firebot',
                executable='rotation_center_test_node',
                name='rotation_center_test',
                output='screen',
            ),
        ]
    )
