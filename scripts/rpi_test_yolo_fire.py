#!/usr/bin/env python3
"""Live YOLO11 fire-detection test with Picamera2 (OV5647 on Pi).

Trained weights from Flare Guard / Ultralytics (Fire + Smoke classes in model;
this tool defaults to Fire-only overlay to match the deployed vision_node).

  https://github.com/sayedgamal99/Real-Time-Smoke-Fire-Detection-YOLO11

Default weights: repo models/best_small.pt (same family as the GitHub project).

Usage:
  cd /path/to/FireBot696
  python3 scripts/rpi_test_yolo_fire.py
  python3 scripts/rpi_test_yolo_fire.py --headless
  python3 scripts/rpi_test_yolo_fire.py --model models/best_small.pt --width 1280 --height 720
  python3 scripts/rpi_test_yolo_fire.py --ros   # must: source install/setup.bash

Headless (SSH, no DISPLAY): omit OpenCV window; prints detection line each infer.

With --ros and a window (local desktop / VNC), while brain_node runs:
  a  publish /alarm/trigger true
  z  publish /alarm/trigger false
  c  publish /ui/confirm true (operator OK)
  x  publish /ui/confirm false (deny)
  e  publish /cmd/estop true
  h  print key help in terminal

Requires: picamera2, ultralytics, opencv-python (headless or full). ROS: rclpy + std_msgs.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

DEFAULT_MODEL = Path(__file__).resolve().parent.parent / "models" / "best_small.pt"


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--video-mode", action="store_true", help="create_video_configuration (often smoother)")
    p.add_argument("--conf", type=float, default=0.20)
    p.add_argument("--imgsz", type=int, default=512)
    p.add_argument("--max-det", type=int, default=3)
    p.add_argument("--infer-every-n", type=int, default=3, help="run inference every N frames")
    p.add_argument("--target-display-fps", type=float, default=10.0)
    p.add_argument("--device", default="cpu", help="cpu or 0 for first CUDA device (not typical on Pi)")
    p.add_argument("--fire-only", action="store_true", default=True)
    p.add_argument("--all-classes", action="store_true", help="disable fire-only filter")
    p.add_argument("--fire-class-name", default="Fire", help="case-sensitive label when --fire-only")
    p.add_argument("--headless", action="store_true", help="no imshow window")
    p.add_argument("--ros", action="store_true", help="enable keyboard ROS publisher (see docstring)")
    args = p.parse_args()

    if args.all_classes:
        args.fire_only = False

    try:
        from picamera2 import Picamera2
    except ImportError:
        print("picamera2 missing. On Pi: sudo apt install -y python3-picamera2", file=sys.stderr)
        return 1

    try:
        import cv2
    except ImportError:
        print("opencv not installed.", file=sys.stderr)
        return 1

    try:
        from ultralytics import YOLO
    except ImportError:
        print("ultralytics missing: pip install ultralytics", file=sys.stderr)
        return 1

    mp = args.model.resolve()
    if not mp.is_file():
        print(f"Model not found: {mp}", file=sys.stderr)
        print(
            "Download e.g. best_small.pt from Real-Time-Smoke-Fire-Detection-YOLO11 models folder.",
            file=sys.stderr,
        )
        return 1

    ros_node = None
    pub_alarm = pub_confirm = pub_estop = None
    if args.ros:
        try:
            import rclpy
            from std_msgs.msg import Bool
        except ImportError:
            print("ROS2 Python not available; drop --ros or source your workspace.", file=sys.stderr)
            return 1
        rclpy.init()
        ros_node = rclpy.create_node("rpi_test_yolo_fire_operator")
        pub_alarm = ros_node.create_publisher(Bool, "/alarm/trigger", 10)
        pub_confirm = ros_node.create_publisher(Bool, "/ui/confirm", 10)
        pub_estop = ros_node.create_publisher(Bool, "/cmd/estop", 10)
        print("[ROS] publishers: /alarm/trigger, /ui/confirm, /cmd/estop — keys a,z,c,x,e,h")

    def ros_pub_bool(pb, val: bool) -> None:
        from std_msgs.msg import Bool as RosBool

        m = RosBool()
        m.data = val
        pb.publish(m)

    model = YOLO(str(mp))
    size = (args.width, args.height)

    picam2 = Picamera2()
    if args.video_mode:
        config = picam2.create_video_configuration(main={"size": size, "format": "RGB888"})
    else:
        config = picam2.create_still_configuration(main={"size": size, "format": "RGB888"})
    picam2.configure(config)
    picam2.start()
    time.sleep(1.0)

    frame_count = 0
    last_result = None
    last_annotated = None
    last_infer_fps = 0.0

    display_frames = 0
    display_fps = 0.0
    last_display_time = time.time()
    frame_interval = 1.0 / max(args.target_display_fps, 1.0)

    print("Running. Headless:", args.headless, "| model:", mp)
    if not args.headless:
        print("Press q to quit. h = ROS key help (if --ros).")

    try:
        while True:
            loop_start = time.time()

            if ros_node is not None:
                rclpy.spin_once(ros_node, timeout_sec=0)

            frame = picam2.capture_array()
            frame_count += 1

            infer_this = (frame_count % args.infer_every_n == 0) or last_result is None
            if infer_this:
                t0 = time.time()
                results = model.predict(
                    source=frame,
                    imgsz=args.imgsz,
                    conf=args.conf,
                    max_det=args.max_det,
                    verbose=False,
                    device=args.device,
                )
                infer_dt = time.time() - t0
                last_infer_fps = 1.0 / infer_dt if infer_dt > 0 else 0.0
                r0 = results[0]
                # Optional Fire-only filter (match vision_node behavior)
                if args.fire_only and r0.boxes is not None and len(r0.boxes):
                    want = args.fire_class_name.strip().lower()
                    keep = [
                        i
                        for i in range(len(r0.boxes))
                        if (model.names.get(int(r0.boxes.cls[i]), "") or "").strip().lower() == want
                    ]
                    if keep:
                        try:
                            r0.boxes = r0.boxes[keep]
                        except Exception:
                            pass
                    else:
                        r0.boxes = None
                last_result = r0
                try:
                    last_annotated = last_result.plot()
                except Exception:
                    last_annotated = frame.copy()

            shown = last_annotated.copy() if last_annotated is not None else frame.copy()

            n = 0 if last_result is None or last_result.boxes is None else len(last_result.boxes)

            for text, y in (
                (f"Display FPS: {display_fps:.1f}", 32),
                (f"Infer FPS: {last_infer_fps:.1f}", 72),
                (f"Detections: {n}", 112),
            ):
                cv2.putText(shown, text, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)

            if args.headless:
                if infer_this and n > 0:
                    print(f"[infer] n={n} conf~={[float(last_result.boxes.conf[i]) for i in range(n)]}", flush=True)
            else:
                cv2.imshow("Fire YOLO11 (Pi test)", shown)

            key = -1 if args.headless else (cv2.waitKey(1) & 0xFF)
            if key == ord("q"):
                break

            if ros_node is not None and key != 255:
                if key == ord("h"):
                    print(
                        "[ROS keys] a=alarm ON z=alarm OFF c=confirm OK x=deny e=estop",
                        flush=True,
                    )
                elif key == ord("a"):
                    ros_pub_bool(pub_alarm, True)
                    print("[ROS] alarm -> true", flush=True)
                elif key == ord("z"):
                    ros_pub_bool(pub_alarm, False)
                    print("[ROS] alarm -> false", flush=True)
                elif key == ord("c"):
                    ros_pub_bool(pub_confirm, True)
                    print("[ROS] confirm -> true", flush=True)
                elif key == ord("x"):
                    ros_pub_bool(pub_confirm, False)
                    print("[ROS] confirm -> false (deny)", flush=True)
                elif key == ord("e"):
                    ros_pub_bool(pub_estop, True)
                    print("[ROS] estop", flush=True)

            display_frames += 1
            now = time.time()
            if now - last_display_time >= 1.0:
                display_fps = display_frames / (now - last_display_time)
                display_frames = 0
                last_display_time = now

            elapsed = time.time() - loop_start
            rest = frame_interval - elapsed
            if rest > 0:
                time.sleep(rest)
    finally:
        picam2.stop()
        if not args.headless:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        if ros_node is not None:
            ros_node.destroy_node()
            try:
                import rclpy as _rclpy

                _rclpy.shutdown()
            except Exception:
                pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
