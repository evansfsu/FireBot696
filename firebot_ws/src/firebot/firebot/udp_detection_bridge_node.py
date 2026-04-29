"""Subscribe to host-only vision via UDP JSON → publish sensor_msgs-compatible FireDetection on /fire/detection.

The host runs scripts/rpi_test_yolo_fire.py with --udp-bridge 127.0.0.1:PORT (Picamera2 + window).
Docker (network_mode: host) sets FIREBOT_UDP_DETECTION_PORT=PORT so launch starts this node
instead of vision_node.

Each datagram is one UTF-8 JSON object (see send in rpi_test_yolo_fire.py).
"""

from __future__ import annotations

import json
import socket
import threading
import time

import rclpy
from rclpy.node import Node

from firebot_interfaces.msg import FireDetection


class UdpDetectionBridgeNode(Node):
    def __init__(self):
        super().__init__('udp_detection_bridge')

        self.declare_parameter('listen_host', '0.0.0.0')
        self.declare_parameter('listen_port', 7766)
        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('stale_sec', 1.0)

        host = str(self.get_parameter('listen_host').value)
        port = int(self.get_parameter('listen_port').value)
        self._stale_sec = float(self.get_parameter('stale_sec').value)
        pub_hz = float(self.get_parameter('publish_hz').value)

        self._pub = self.create_publisher(FireDetection, '/fire/detection', 10)
        self._lock = threading.Lock()
        self._rx_mono = 0.0
        self._payload: dict | None = None

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, port))

        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

        period = 1.0 / max(pub_hz, 1.0)
        self.create_timer(period, self._publish_tick)

        self.get_logger().info(
            f'udp_detection_bridge listening on {host}:{port} → /fire/detection '
            f'(run host: python3 scripts/rpi_test_yolo_fire.py --udp-bridge 127.0.0.1:{port})'
        )

    def _recv_loop(self) -> None:
        while True:
            try:
                data, _addr = self._sock.recvfrom(65536)
            except OSError:
                break
            try:
                msg = json.loads(data.decode('utf-8'))
                if not isinstance(msg, dict):
                    continue
            except (json.JSONDecodeError, UnicodeError):
                continue
            with self._lock:
                self._payload = msg
                self._rx_mono = time.monotonic()

    def _publish_tick(self) -> None:
        with self._lock:
            payload = dict(self._payload) if self._payload is not None else None
            last_mono = self._rx_mono

        out = FireDetection()
        now = time.monotonic()
        if payload is None or (now - last_mono) > self._stale_sec:
            out.detected = False
            self._pub.publish(out)
            return

        out.detected = bool(payload.get('detected', False))
        out.confidence = float(payload.get('confidence', 0.0))
        out.bbox_x = float(payload.get('bbox_x', 0.0))
        out.bbox_y = float(payload.get('bbox_y', 0.0))
        out.bbox_w = float(payload.get('bbox_w', 0.0))
        out.bbox_h = float(payload.get('bbox_h', 0.0))
        out.bbox_area = float(payload.get('bbox_area', out.bbox_w * out.bbox_h))
        out.x_offset = float(payload.get('x_offset', (out.bbox_x - 0.5) * 2.0))
        out.label = str(payload.get('label', ''))
        self._pub.publish(out)

    def destroy_node(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UdpDetectionBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
