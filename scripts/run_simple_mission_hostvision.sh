#!/usr/bin/env bash
# Start Docker simple mission stack (detached), then run host vision + UDP bridge.
# Usage (repo root, Pi/Linux): ./scripts/run_simple_mission_hostvision.sh
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
UDP_HOST_PORT="${FIREBOT_UDP_DETECTION_PORT:-7766}"

cd "$ROOT/docker"
docker compose -f docker-compose.yml -f compose.host-vision-udp-simple.yml up -d

exec python3 "$ROOT/scripts/rpi_test_yolo_fire.py" --video-mode --udp-bridge "127.0.0.1:${UDP_HOST_PORT}"
