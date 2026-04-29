"""Vision node: grabs frames from the ov5647 camera and runs YOLO inference.

Publishes a single `/fire/detection` message per cycle. The message always
carries a `detected` flag; when false, the numeric fields are zero. This keeps
the brain node's subscription callback simple -- no "stale" detection logic.
"""

import rclpy
from rclpy.node import Node

from firebot_interfaces.msg import FireDetection
from sensor_msgs.msg import Image

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

import cv2


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.declare_parameter('model_path', '/models/best_small.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('inference_imgsz', 512)
        self.declare_parameter('max_det', 3)
        self.declare_parameter('fire_only', True)
        self.declare_parameter('fire_class_name', 'Fire')
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('detection_fps', 3.0)
        self.declare_parameter('publish_debug_image', False)

        self.model_path = self.get_parameter('model_path').value
        self.conf_threshold = float(self.get_parameter('confidence_threshold').value)
        self.infer_imgsz = int(self.get_parameter('inference_imgsz').value)
        self.max_det = int(self.get_parameter('max_det').value)
        self.fire_only = bool(self.get_parameter('fire_only').value)
        self.fire_class_name = str(self.get_parameter('fire_class_name').value)
        self.cam_w = int(self.get_parameter('camera_width').value)
        self.cam_h = int(self.get_parameter('camera_height').value)
        self.det_fps = float(self.get_parameter('detection_fps').value)
        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)

        self.pub = self.create_publisher(FireDetection, '/fire/detection', 10)
        self.pub_debug = (
            self.create_publisher(Image, '/fire/debug_image', 2)
            if self.publish_debug
            else None
        )

        self._init_camera()
        self._init_model()

        period = 1.0 / max(self.det_fps, 0.1)
        self.timer = self.create_timer(period, self._detect)
        self.get_logger().info(
            f'vision_node up: model={self.model_path}, '
            f'cam={self.cam_w}x{self.cam_h}@{self.det_fps:.1f}Hz'
        )
        if self.pub_debug is not None:
            self.get_logger().info('publishing /fire/debug_image for desktop preview')

    def _publish_debug_frame(self, frame_rgb, best):
        """BGR8 Image with optional YOLO box (for Pi desktop rqt_image_view)."""
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

    def _init_camera(self):
        self.camera = None
        if PICAMERA_AVAILABLE:
            try:
                cam = Picamera2()
                config = cam.create_still_configuration(
                    main={'size': (self.cam_w, self.cam_h), 'format': 'RGB888'}
                )
                cam.configure(config)
                cam.start()
                self.camera = cam
                self.get_logger().info('picamera2 (ov5647) initialized')
                return
            except Exception as exc:
                self.get_logger().warn(f'picamera2 init failed: {exc}')
                self.camera = None
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cam_w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cam_h)
            self.camera = cap
            self.get_logger().info('OpenCV VideoCapture(0) fallback opened')
        else:
            cap.release()
            self.get_logger().warn(
                'no camera available; vision_node will keep running and '
                'publish detected=False'
            )

    def _init_model(self):
        self.model = None
        if not YOLO_AVAILABLE:
            self.get_logger().warn('ultralytics not installed; detection disabled')
            return
        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info(f'YOLO model loaded: {self.model_path}')
        except Exception as exc:
            self.get_logger().warn(f'YOLO model load failed: {exc}')

    def _capture(self):
        if self.camera is None:
            return None
        if PICAMERA_AVAILABLE and isinstance(self.camera, Picamera2):
            return self.camera.capture_array()
        if isinstance(self.camera, cv2.VideoCapture):
            ok, frame = self.camera.read()
            return frame if ok else None
        return None

    def _publish_empty(self):
        msg = FireDetection()
        msg.detected = False
        self.pub.publish(msg)

    def _detect(self):
        frame = self._capture()
        if frame is None or self.model is None:
            self._publish_empty()
            if frame is not None:
                self._publish_debug_frame(frame, None)
            return

        results = self.model.predict(
            source=frame,
            imgsz=self.infer_imgsz,
            conf=self.conf_threshold,
            max_det=self.max_det,
            verbose=False,
            device='cpu',
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
                if self.fire_only and label.strip().lower() != self.fire_class_name.strip().lower():
                    continue
                if conf > best_conf:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    best_conf = conf
                    best = (conf, label, x1, y1, x2, y2)

        msg = FireDetection()
        if best is None:
            msg.detected = False
            self.pub.publish(msg)
            self._publish_debug_frame(frame, None)
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
        msg.x_offset = float(cx - 0.5) * 2.0  # normalized -1..1 (left/right)
        msg.label = str(label)
        self.pub.publish(msg)
        self._publish_debug_frame(frame, best)

    def destroy_node(self):
        cam = getattr(self, 'camera', None)
        if cam is not None:
            if PICAMERA_AVAILABLE and isinstance(cam, Picamera2):
                try:
                    cam.stop()
                except Exception:
                    pass
            elif isinstance(cam, cv2.VideoCapture):
                cam.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
