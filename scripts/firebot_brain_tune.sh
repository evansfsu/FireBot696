#!/usr/bin/env bash
# Tune brain_node seek/drive live (ros2 param set). Requires updated brain_node with
# add_on_set_parameters_callback (reloads cached fields on set).
#
# Usage (native ROS on Pi / Linux, same machine as brain):
#   ./scripts/firebot_brain_tune.sh help
#   ./scripts/firebot_brain_tune.sh mode pulse
#   ./scripts/firebot_brain_tune.sh rotate 0.35
#   ./scripts/firebot_brain_tune.sh speed 40
#
# Docker (simple host-vision stack): from repo root, same compose as run_simple_mission_hostvision.sh
#   FIREBOT_USE_DOCKER=1 ./scripts/firebot_brain_tune.sh status
#
# Override compose files:
#   FIREBOT_USE_DOCKER=1 \
#   FIREBOT_COMPOSE_FILES="-f docker-compose.yml -f compose.host-vision-udp-simple-pulse.yml" \
#   ./scripts/firebot_brain_tune.sh mode pulse
#
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
NODE="${FIREBOT_BRAIN_NODE:-/brain_node}"
DC="${FIREBOT_DOCKER_DIR:-$ROOT/docker}"
COMPOSE_FILES="${FIREBOT_COMPOSE_FILES:--f docker-compose.yml -f compose.host-vision-udp-simple.yml}"
SERVICE="${FIREBOT_DOCKER_SERVICE:-firebot}"

ros2_exec() {
  if [[ -n "${FIREBOT_USE_DOCKER:-}" ]]; then
    (cd "$DC" && docker compose $COMPOSE_FILES exec -T "$SERVICE" ros2 "$@")
  else
    ros2 "$@"
  fi
}

param_set() {
  local name="$1"
  shift
  local val="$*"
  ros2_exec param set "$NODE" "$name" "$val"
}

cmd_help() {
  cat <<'EOF'
Commands (all apply to brain_node immediately if the node supports runtime param reload):

  start-docker      Print docker one-liner to bring up simple mission stack (does not run it).
  start-hostvision  Print command to run host YOLO + UDP after Docker is up.

  status            ros2 param get for seek/drive-related params.
  mode continuous|pulse
  rotate SEC        simple_pulse_rotate_sec (burst on time)
  rest SEC          simple_pulse_rest_sec (gap / coast between bursts)
  interval SEC      simple_pulse_min_interval_sec (min time between burst starts)
  speed PWM         rotate_speed (seek / track yaw command; bridge scales to motors)
  v_approach PWM    forward speed during APPROACHING (full flow)
  kp_yaw N          yaw P gain during approach pulses
  corner_vx PWM     corner_exit_speed
  alternate 0|1     simple_pulse_seek_alternate

Examples:

  ./scripts/firebot_brain_tune.sh mode continuous
  ./scripts/firebot_brain_tune.sh rotate 0.3 && ./scripts/firebot_brain_tune.sh rest 0.5 && ./scripts/firebot_brain_tune.sh speed 36

Raw passthrough (any declared param):

  ./scripts/firebot_brain_tune.sh raw simple_mission_center_band_frac 0.75
EOF
}

cmd_status() {
  for n in simple_seek_mode simple_pulse_rotate_sec simple_pulse_rest_sec \
    simple_pulse_min_interval_sec rotate_speed v_approach kp_yaw corner_exit_speed \
    simple_pulse_seek_alternate; do
    echo "--- $n"
    ros2_exec param get "$NODE" "$n" || true
  done
}

cmd_start_docker() {
  echo "cd \"$ROOT/docker\" && docker compose -f docker-compose.yml -f compose.host-vision-udp-simple.yml up -d"
}

cmd_start_hostvision() {
  echo "python3 \"$ROOT/scripts/rpi_test_yolo_fire.py\" --video-mode --udp-bridge 127.0.0.1:\${FIREBOT_UDP_DETECTION_PORT:-7766}"
}

main() {
  local sub="${1:-help}"
  shift || true
  case "$sub" in
    help|-h|--help) cmd_help ;;
    start-docker) cmd_start_docker ;;
    start-hostvision) cmd_start_hostvision ;;
    status) cmd_status ;;
    mode)
      [[ "${1:-}" =~ ^(continuous|pulse)$ ]] || { echo "usage: mode continuous|pulse"; exit 1; }
      param_set simple_seek_mode "$1"
      ;;
    rotate) param_set simple_pulse_rotate_sec "${1:?seconds}" ;;
    rest) param_set simple_pulse_rest_sec "${1:?seconds}" ;;
    interval) param_set simple_pulse_min_interval_sec "${1:?seconds}" ;;
    speed) param_set rotate_speed "${1:?pwm}" ;;
    v_approach) param_set v_approach "${1:?pwm}" ;;
    kp_yaw) param_set kp_yaw "${1:?value}" ;;
    corner_vx) param_set corner_exit_speed "${1:?pwm}" ;;
    alternate) param_set simple_pulse_seek_alternate "${1:?0_or_1}" ;;
    raw)
      [[ "$#" -ge 2 ]] || { echo "usage: raw <param_name> <value>"; exit 1; }
      param_set "$1" "$2"
      ;;
    *)
      echo "unknown command: $sub (try help)"
      exit 1
      ;;
  esac
}

main "$@"
