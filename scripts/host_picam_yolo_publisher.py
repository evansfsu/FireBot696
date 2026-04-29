#!/usr/bin/env python3
"""Publish /fire/detection from the host using Picamera2 + YOLO (same stack as rpi_test_yolo_fire).

Use when Docker vision_node cannot see the CSI camera (no /dev/video* or OpenCV-only path fails)
but native Picamera2 works with scripts/rpi_test_yolo_fire.py.

Requires ROS 2 + built workspace on the Pi:

  cd /path/to/FireBot696/firebot_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  cd ..
  export ROS_DOMAIN_ID=0
  python3 scripts/host_picam_yolo_publisher.py

Start the Docker stack *without* vision_node (same ROS_DOMAIN_ID, host networking):

  docker compose -f docker/docker-compose.yml run --rm -e FIREBOT_SKIP_VISION=1 ...

Or set environment FIREBOT_SKIP_VISION=1 on the firebot service and rebuild/restart.

Options mirror vision_node defaults; pass --help.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from firebot_interfaces.msg import FireDetection

try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None  # type: ignore

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None  # type: ignore

DEFAULT_MODEL = Path(__file__).resolve().parent.parent / "models" / "best_small.pt"


class HostVisionPublisher(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("host_picam_yolo_publisher")
        self._args = args
        self.pub = self.create_publisher(FireDetection, "/fire/detection", 10)
        self.pub_debug = (
            self.create_publisher(Image, "/fire/debug_image", 2) if args.publish_debug else None
        )

        if Picamera2 is None:
            print("Install picamera2: sudo apt install -y python3-picamera2", file=sys.stderr)
            raise SystemExit(1)
        if YOLO is None:
            print("Install ultralytics (same venv as other Pi scripts)", file=sys.stderr)
            raise SystemExit(1)

        mp = Path(args.model).resolve()
        if not mp.is_file():
            print(f"Model not found: {mp}", file=sys.stderr)
            raise SystemExit(1)

        self.model = YOLO(str(mp))
        size = (args.width, args.height)
        self.picam2 = Picamera2()
        if args.video_mode:
            cfg = self.picam2.create_video_configuration(
                main={"size": size, "format": "RGB888"}
            )
        else:
            cfg = self.picam2.create_still_configuration(
                main={"size": size, "format": "RGB888"}
            )
        self.picam2.configure(cfg)
        self.picam2.start()
        time.sleep(max(0.0, args.warmup_sec))

        period = 1.0 / max(args.fps, 0.1)
        self.create_timer(period, self._on_timer)
        names = getattr(self.model, "names", None)
        print(
            f"host_picam_yolo_publisher: Picamera2 + {mp} | classes={dict(names) if names else '?'}"
            f" | fire_only={args.fire_only} | conf={args.conf} | {args.width}x{args.height} @ {args.fps}Hz",
            flush=True,
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
                f"{label} {conf:.2f}",
                (int(x1), max(24, int(y1) - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                tic,
                1,
                cv2.LINE_AA,
            )
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_optical_frame"
        msg.height = vis.shape[0]
        msg.width = vis.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = vis.tobytes()
        self.pub_debug.publish(msg)

    def _on_timer(self):
        args = self._args
        frame = self.picam2.capture_array()
        if frame is None:
            self._publish_empty()
            return
        if frame.ndim == 3 and frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)

        results = self.model.predict(
            source=frame,
            imgsz=args.imgsz,
            conf=args.conf,
            max_det=args.max_det,
            verbose=False,
            device=args.device,
        )

        best_conf = 0.0
        best = None
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = self.model.names.get(cls_id, "unknown")
                if (
                    args.fire_only
                    and label.strip().lower() != args.fire_class_name.strip().lower()
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


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument(
        "--still-mode",
        action="store_true",
        help="use Picamera2 still configuration (default: video, matches rpi_test_yolo_fire.py --video-mode)",
    )
    p.add_argument("--warmup-sec", type=float, default=1.0)
    p.add_argument("--fps", type=float, default=3.0)
    p.add_argument("--conf", type=float, default=0.25)
    p.add_argument("--imgsz", type=int, default=512)
    p.add_argument("--max-det", type=int, default=3)
    p.add_argument("--fire-only", action="store_true", default=True)
    p.add_argument("--all-classes", action="store_true", help="publish best box regardless of class name")
    p.add_argument("--fire-class-name", default="Fire")
    p.add_argument("--device", default="cpu")
    p.add_argument("--publish-debug", action="store_true", help="also publish /fire/debug_image")
    args = p.parse_args()
    if args.all_classes:
        args.fire_only = False
    args.video_mode = not args.still_mode

    rclpy.init()
    node = HostVisionPublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
