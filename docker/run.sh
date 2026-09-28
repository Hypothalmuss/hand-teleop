#!/usr/bin/env bash
# Hand teleop in Docker: one entry point for a fresh PC (only Docker + a webcam needed).
#
#   docker/run.sh build                 build the image (first time, ~5-10 min)
#   docker/run.sh setup-hands           handedness check: raise your RIGHT hand
#   docker/run.sh calibrate             workspace calibration (~30 s), saved to config/
#   docker/run.sh teleop [args...]      teleop: MuJoCo viewer + camera window + console window
#   docker/run.sh record [sec] [out]    teleop + split-screen demo recording (default 60 s)
#   docker/run.sh console               keyboard console in this terminal (teleop running)
#   docker/run.sh fake                  teleop with a synthetic hand (no webcam)
#   docker/run.sh test                  run the test suite in the container
#   docker/run.sh shell                 interactive shell in the container
#
# Env: VIDEO_DEVICE (default /dev/video0), GPU=auto|nvidia|none|software (default auto:
# NVIDIA runtime if available, else /dev/dri for Intel/AMD, else software rendering).
set -euo pipefail
cd "$(dirname "$0")/.."

export HOST_UID="$(id -u)" HOST_GID="$(id -g)"
export VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video0}"
export DISPLAY="${DISPLAY:-:0}"
mkdir -p results recordings

compose=(docker compose -f docker-compose.yml)
gpu="${GPU:-auto}"
if [ "$gpu" = "auto" ]; then
  if docker info 2>/dev/null | grep -q "Runtimes:.*nvidia" && command -v nvidia-smi >/dev/null \
      && nvidia-smi >/dev/null 2>&1; then gpu=nvidia; else gpu=none; fi
fi
if [ "$gpu" = "nvidia" ]; then
  compose+=(-f docker-compose.nvidia.yml)
elif [ -d /dev/dri ] && [ "$gpu" != "software" ]; then
  gpu="mesa (/dev/dri)"
  export RENDER_GID="$(stat -c %g /dev/dri/renderD128 2>/dev/null || echo 109)"
  compose+=(-f docker-compose.dri.yml)
fi

needs_camera() {
  if [ -e "$VIDEO_DEVICE" ]; then
    export VIDEO_GID="$(stat -c %g "$VIDEO_DEVICE")"
  else
    echo "webcam $VIDEO_DEVICE not found (set VIDEO_DEVICE=/dev/videoN, or use: $0 fake)" >&2
    exit 1
  fi
}
allow_x() {
  if command -v xhost >/dev/null; then xhost +local: >/dev/null 2>&1 || true; fi
}
run() {  # run a one-off container
  "${compose[@]}" run --rm --name "hand-teleop-$1" teleop "${@:2}"
}

cmd="${1:-teleop}"; shift || true
echo "hand-teleop: $cmd (GPU: $gpu)" >&2
case "$cmd" in
  build)
    "${compose[@]}" build ;;
  setup-hands)
    needs_camera; allow_x; run setup python3 scripts/setup_hand_tracker.py ;;
  calibrate)
    needs_camera; allow_x; run calibrate python3 scripts/calibrate_workspace.py ;;
  teleop)
    needs_camera; allow_x
    run teleop ros2 launch hand_teleop_bringup teleop.launch.py console:=xterm "$@" ;;
  fake)
    export VIDEO_GID="${VIDEO_GID:-44}"; export VIDEO_DEVICE=/dev/null; allow_x
    run fake ros2 launch hand_teleop_bringup teleop.launch.py hand:=fake console:=xterm "$@" ;;
  record)
    needs_camera; allow_x
    secs="${1:-60}"; out="${2:-results/demo_$(date +%Y%m%d_%H%M%S).mp4}"
    run record bash -c "ros2 launch hand_teleop_bringup teleop.launch.py console:=xterm \
      demo_view:=true & sleep 8 && python3 scripts/record_demo_video.py --seconds $secs \
      --out $out; kill -INT %1; wait" ;;
  console)
    docker exec -it hand-teleop-teleop /entrypoint.sh ros2 run teleop_console keyboard_console ;;
  test)
    export VIDEO_GID="${VIDEO_GID:-44}"; export VIDEO_DEVICE=/dev/null
    run test bash -c "mkdir -p /tmp/ws && cd /tmp/ws && cp -r /ws/src /ws/config /ws/pyproject.toml . \
      && colcon build --event-handlers console_cohesion- >/dev/null && . install/setup.bash \
      && colcon test --executor sequential >/dev/null; colcon test-result | tail -3" ;;
  shell)
    export VIDEO_GID="${VIDEO_GID:-44}"; [ -e "$VIDEO_DEVICE" ] || export VIDEO_DEVICE=/dev/null
    allow_x; run shell bash ;;
  *)
    sed -n '2,15p' "$0"; exit 1 ;;
esac
