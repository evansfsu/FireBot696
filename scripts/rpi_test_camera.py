#!/usr/bin/env python3
"""Quick Picamera2 (e.g. OV5647) capture test on Raspberry Pi.

Verifies the camera pipeline before running the fire vision stack.

Usage (on Pi, from repo root or anywhere):
  python3 scripts/rpi_test_camera.py
  python3 scripts/rpi_test_camera.py --width 640 --height 480 --frames 30
  python3 scripts/rpi_test_camera.py --preview   # blocks until 'q' in window

Requires: picamera2 (not typically available on Windows).
"""
from __future__ import annotations

import argparse
import sys
import time


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--frames", type=int, default=10, help="capture N frames (ignored unless --no-preview)")
    p.add_argument("--preview", action="store_true", help="open OpenCV window until 'q'")
    p.add_argument("--video-mode", action="store_true", help="use create_video_configuration (faster stream)")
    args = p.parse_args()

    try:
        from picamera2 import Picamera2
    except ImportError:
        print("picamera2 not installed. On Raspberry Pi OS: sudo apt install -y python3-picamera2", file=sys.stderr)
        return 1

    try:
        import cv2
    except ImportError:
        print("opencv-python not installed.", file=sys.stderr)
        return 1

    size = (args.width, args.height)
    picam2 = Picamera2()
    if args.video_mode:
        cfg = picam2.create_video_configuration(main={"size": size, "format": "RGB888"})
    else:
        cfg = picam2.create_still_configuration(main={"size": size, "format": "RGB888"})
    picam2.configure(cfg)
    picam2.start()
    time.sleep(0.5)

    if args.preview:
        print("Preview running — press 'q' to quit.")
        fps_t0 = time.time()
        n = 0
        while True:
            frame = picam2.capture_array()
            n += 1
            dt = time.time() - fps_t0
            fps = n / dt if dt > 0 else 0.0
            cv2.putText(
                frame,
                f"{size[0]}x{size[1]}  ~{fps:.1f} FPS",
                (16, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            cv2.imshow("rpi_test_camera", frame)
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break
        cv2.destroyAllWindows()
    else:
        t0 = time.time()
        for i in range(args.frames):
            frame = picam2.capture_array()
            if frame is None or frame.size == 0:
                print(f"frame {i}: empty", file=sys.stderr)
                return 1
        elapsed = time.time() - t0
        print(f"Captured {args.frames} frames at {size} in {elapsed:.2f}s ({args.frames/elapsed:.1f} FPS avg)")

    picam2.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
