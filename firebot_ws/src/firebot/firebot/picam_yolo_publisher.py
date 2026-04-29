"""Picamera2 + YOLO publisher for /fire/detection (same pipeline as rpi_test_yolo_fire --video-mode).

Use when vision_node V4L path fails inside Docker but Picamera2 works — run *inside* the
firebot image so no ROS install is required on the Pi host:

  docker compose -f docker/docker-compose.yml -f docker/compose.picam-sidecar.yml up -d

That starts brain/bridge without vision_node and this publisher in a second container.
"""

from __future__ import annotations

import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from firebot_interfaces.msg import FireDetection

try:
    from picamera2 import Picamera2
    PICAMERA_AVAILABLE = True
except ImportError:
    PICAMERA_AVAILABLE = False

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class PicamYoloPublisher(Node):
    def __init__(self):
        super().__init__('picam_yolo_publisher')

        self.declare_parameter('model_path', '/models/best_small.pt')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('inference_imgsz', 512)
        self.declare_parameter('max_det', 3)
        self.declare_parameter('fire_only', True)
        self.declare_parameter('fire_class_name', 'Fire')
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('picamera_video_mode', True)
        self.declare_parameter('picamera_warmup_sec', 1.0)
        self.declare_parameter('detection_fps', 3.0)
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('yolo_device', 'cpu')

        self.model_path = str(self.get_parameter('model_path').value)
        self.conf_threshold = float(self.get_parameter('confidence_threshold').value)
        self.infer_imgsz = int(self.get_parameter('inference_imgsz').value)
        self.max_det = int(self.get_parameter('max_det').value)
        self.fire_only = bool(self.get_parameter('fire_only').value)
        self.fire_class_name = str(self.get_parameter('fire_class_name').value)
        self.cam_w = int(self.get_parameter('camera_width').value)
        self.cam_h = int(self.get_parameter('camera_height').value)
        self.picamera_video_mode = bool(self.get_parameter('picamera_video_mode').value)
        self.picamera_warmup_sec = float(self.get_parameter('picamera_warmup_sec').value)
        self.det_fps = float(self.get_parameter('detection_fps').value)
        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)
        self.yolo_device = str(self.get_parameter('yolo_device').value)

        self.pub = self.create_publisher(FireDetection, '/fire/detection', 10)
        self.pub_debug = (
            self.create_publisher(Image, '/fire/debug_image', 2)
            if self.publish_debug
            else None
        )

        if not PICAMERA_AVAILABLE:
            raise RuntimeError('python3-picamera2 not available in this image')
        if not YOLO_AVAILABLE:
            raise RuntimeError('ultralytics not available in this image')

        self.model = YOLO(self.model_path)
        size = (self.cam_w, self.cam_h)
        self.picam2 = Picamera2()
        if self.picamera_video_mode:
            cfg = self.picam2.create_video_configuration(
                main={'size': size, 'format': 'RGB888'}
            )
            mode = 'video'
        else:
            cfg = self.picam2.create_still_configuration(
                main={'size': size, 'format': 'RGB888'}
            )
            mode = 'still'
        self.picam2.configure(cfg)
        self.picam2.start()
        time.sleep(max(0.0, self.picamera_warmup_sec))

        period = 1.0 / max(self.det_fps, 0.1)
        self.create_timer(period, self._on_timer)

        names = getattr(self.model, 'names', None)
        self.get_logger().info(
            f'picam_yolo_publisher: Picamera2 ({mode}) + {self.model_path} | '
            f'classes={dict(names) if names else "?"} | {self.cam_w}x{self.cam_h} @ {self.det_fps}Hz'
        )

    def _publish_debug(self, frame_rgb, best):
        if self.pub_debug is None or frame_rgb is None:
            return
        vis = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        if best is not None:
            conf, label, x1, y1, x2, y2 = best
            tic = (0, 255, 0)
            cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), tic, 2)
            cv2.putText(
                vis,
                f'{label} {conf:.2f}',
                (int(x1), max(24, int(y1) - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                tic,
                1,
                cv2.LINE_AA,
            )
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_optical_frame'
        msg.height = vis.shape[0]
        msg.width = vis.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = vis.tobytes()
        self.pub_debug.publish(msg)

    def _on_timer(self):
        frame = self.picam2.capture_array()
        if frame is None:
            self._publish_empty()
            return
        if frame.ndim == 3 and frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)

        results = self.model.predict(
            source=frame,
            imgsz=self.infer_imgsz,
            conf=self.conf_threshold,
            max_det=self.max_det,
            verbose=False,
            device=self.yolo_device,
        )

        best_conf = 0.0
        best = None
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = self.model.names.get(cls_id, 'unknown')
                if (
                    self.fire_only
                    and label.strip().lower() != self.fire_class_name.strip().lower()
                ):
                    continue
                if conf > best_conf:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    best_conf = conf
                    best = (conf, label, x1, y1, x2, y2)

        msg = FireDetection()
        if best is None:
            msg.detected = False
            self.pub.publish(msg)
            self._publish_debug(frame, None)
            return

        h, w = frame.shape[:2]
        conf, label, x1, y1, x2, y2 = best
        cx = (x1 + x2) * 0.5 / w
        cy = (y1 + y2) * 0.5 / h
        bw = (x2 - x1) / w
        bh = (y2 - y1) / h

        msg.detected = True
        msg.confidence = float(conf)
        msg.bbox_x = float(cx)
        msg.bbox_y = float(cy)
        msg.bbox_w = float(bw)
        msg.bbox_h = float(bh)
        msg.bbox_area = float(bw * bh)
        msg.x_offset = float(cx - 0.5) * 2.0
        msg.label = str(label)
        self.pub.publish(msg)
        self._publish_debug(frame, best)

    def _publish_empty(self):
        msg = FireDetection()
        msg.detected = False
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            self.picam2.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PicamYoloPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
