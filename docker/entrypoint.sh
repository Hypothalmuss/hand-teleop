#!/bin/bash
# Source ROS and the workspace, then run the given command.
set -e
mkdir -p "$HOME" "$ROS_LOG_DIR"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /ws/install/setup.bash
# With a display, MuJoCo renders through GLFW/GLX like a native run (forcing EGL next to the
# GLFW viewer fails to make the context current). Headless: EGL.
if [ -z "${DISPLAY:-}" ]; then
  export MUJOCO_GL="${MUJOCO_GL:-egl}"
  echo "note: DISPLAY is not set; GUI windows (viewer, camera, console) are unavailable." >&2
fi
exec "$@"
