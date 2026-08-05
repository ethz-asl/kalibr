#!/usr/bin/env bash
# Start (or reuse) the kalibr Basler container and open an interactive shell.
# Safe to run from multiple terminals — each call attaches via docker exec.
set -euo pipefail

IMAGE="${IMAGE:-kalibr}"
NAME="${NAME:-kalibr_basler}"

xhost +local:root >/dev/null 2>&1 || true

shell_cmd='source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && cd /catkin_ws && exec bash'

if docker ps -q -f "name=^${NAME}$" | grep -q .; then
  : # already running
elif docker ps -aq -f "name=^${NAME}$" | grep -q .; then
  docker start "${NAME}" >/dev/null
else
  docker run -d \
    --name "${NAME}" \
    --privileged \
    --net=host \
    --entrypoint /bin/bash \
    -e DISPLAY="${DISPLAY:-}" \
    -e QT_X11_NO_MITSHM=1 \
    -e KALIBR_MANUAL_FOCAL_LENGTH_INIT=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev/bus/usb:/dev/bus/usb \
    "${IMAGE}" \
    -lc 'sleep infinity' >/dev/null
fi

exec docker exec -it \
  -e DISPLAY="${DISPLAY:-}" \
  -e QT_X11_NO_MITSHM=1 \
  "${NAME}" \
  /bin/bash -lc "${shell_cmd}"
