#!/usr/bin/env python3
"""Show /fire/debug_image from vision_node on the Pi desktop (OpenCV).

Use when `apt install ros-humble-rqt-image-view` is unavailable on your OS/arch.

Requires (on the Pi host, not in Docker):
  - ROS 2 Humble: source /opt/ros/humble/setup.bash
  - sudo apt install -y python3-numpy python3-opencv python3-sensor-msgs (if missing)
  - Docker stack running with publish_debug_image: true

  cd ~/FireBot696
  source /opt/ros/humble/setup.bash
  python3 scripts/rpi_view_debug_image.py
"""
from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class DebugImageViewer(Node):
    def __init__(self) -> None:
        super().__init__('rpi_debug_image_viewer')
        self.sub = self.create_subscription(Image, '/fire/debug_image', self._cb, 10)
        self._warned_enc = False
        try:
            import cv2  # noqa: F401
            import numpy as np  # noqa: F401
        except ImportError as e:
            print(
                'Need OpenCV + NumPy on the host: '
                'sudo apt install -y python3-opencv python3-numpy',
                file=sys.stderr,
            )
            raise SystemExit(1) from e
        self._cv2 = __import__('cv2')
        self._np = __import__('numpy')
        self.get_logger().info('Showing /fire/debug_image (q or Ctrl+C to quit)')

    def _cb(self, msg: Image) -> None:
        if msg.encoding != 'bgr8':
            if not self._warned_enc:
                self.get_logger().warn(f'got encoding {msg.encoding!r} (expected bgr8)')
                self._warned_enc = True
            return
        h, w = int(msg.height), int(msg.width)
        if len(msg.data) != h * w * 3:
            return
        buf = self._np.frombuffer(msg.data, dtype=self._np.uint8)
        arr = buf.reshape((h, w, 3)).copy()
        self._cv2.imshow('FireBot /fire/debug_image (q=quit)', arr)
        key = self._cv2.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q')):
            rclpy.shutdown()


def main() -> int:
    rclpy.init()
    node = DebugImageViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            import cv2
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
