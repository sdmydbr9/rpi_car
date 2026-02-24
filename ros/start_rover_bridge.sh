#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROVER_SERVER_URL="${ROVER_SERVER_URL:-}"
ROVER_IP_REFRESH_SEC="${ROVER_IP_REFRESH_SEC:-2}"
SERVER_URL_FILE="$SCRIPT_DIR/.server_url"
CONTAINER_SERVER_URL_FILE="/app/ros/.server_url"

resolve_server_url() {
  if [[ -n "$ROVER_SERVER_URL" ]]; then
    printf '%s' "$ROVER_SERVER_URL"
    return
  fi

  local host_ip
  host_ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
  if [[ -n "$host_ip" ]]; then
    printf 'http://%s:5000' "$host_ip"
  else
    printf 'http://127.0.0.1:5000'
  fi
}

write_server_url_file() {
  local resolved_url
  resolved_url="$(resolve_server_url)"
  printf '%s\n' "$resolved_url" > "${SERVER_URL_FILE}.tmp"
  mv "${SERVER_URL_FILE}.tmp" "$SERVER_URL_FILE"
  printf '%s' "$resolved_url"
}

INITIAL_SERVER_URL="$(write_server_url_file)"

(
  while true; do
    write_server_url_file >/dev/null || true
    sleep "$ROVER_IP_REFRESH_SEC"
  done
) &
IP_MONITOR_PID=$!

cleanup() {
  kill "$IP_MONITOR_PID" >/dev/null 2>&1 || true
}

trap cleanup EXIT INT TERM

"$SCRIPT_DIR/run_ros_docker.sh" bash -lc '
set -euo pipefail
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd /ros_ws
colcon build --symlink-install
set +u
source /ros_ws/install/setup.bash
set -u
ros2 launch rover_control rover_cmd_vel.launch.py \
  server_url:='"$INITIAL_SERVER_URL"' \
  server_url_file:='"$CONTAINER_SERVER_URL_FILE"'
'
