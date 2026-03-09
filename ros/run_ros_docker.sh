#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER_NAME="${ROS_CONTAINER_NAME:-rover_ros2}"
IMAGE="${ROS_DOCKER_IMAGE:-ros:jazzy-ros-base}"

if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
  docker rm -f "$CONTAINER_NAME" >/dev/null
fi

DEVICE_ARGS=(--device=/dev/ttyS0)
for dev in /dev/gpiomem /dev/gpiochip0 /dev/gpiochip1; do
  if [[ -e "$dev" ]]; then
    DEVICE_ARGS+=(--device="$dev")
  fi
done

CMD=("$@")
if [[ ${#CMD[@]} -eq 0 ]]; then
  CMD=(bash)
fi

DOCKER_TTY_ARGS=(-i)
if [[ -t 0 && -t 1 ]]; then
  DOCKER_TTY_ARGS=(-it)
fi

exec docker run "${DOCKER_TTY_ARGS[@]}" \
  --network host \
  "${DEVICE_ARGS[@]}" \
  -v "$SCRIPT_DIR/ws:/ros_ws" \
  -v "$SCRIPT_DIR/..:/app" \
  --name "$CONTAINER_NAME" \
  "$IMAGE" \
  "${CMD[@]}"
