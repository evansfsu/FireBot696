#!/usr/bin/env python3
"""Optional: run Picamera2 + YOLO as a ROS node on a machine that already has ROS sourced.

On a Pi *without* /opt/ros/humble (ROS only in Docker), do NOT use this file — use the
sidecar compose file instead:

  docker compose -f docker/docker-compose.yml -f docker/compose.picam-sidecar.yml up -d

If you do have ROS + firebot_ws built locally:

  cd firebot_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
  ros2 run firebot picam_yolo_publisher
"""

import sys

try:
    from firebot.picam_yolo_publisher import main
except ImportError:
    print(
        'Cannot import firebot (source firebot_ws/install/setup.bash first).\n'
        'Or run vision in Docker: docker compose -f docker/docker-compose.yml '
        '-f docker/compose.picam-sidecar.yml up -d',
        file=sys.stderr,
    )
    raise SystemExit(1) from None

if __name__ == '__main__':
    main()
