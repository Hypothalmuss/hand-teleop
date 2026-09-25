#!/usr/bin/env bash
# Demo video v1: launch the teleop chain with the demo camera and record a split screen.
#   scripts/record_demo_video.sh [seconds] [output]
set -euo pipefail
SECONDS_TO_RECORD="${1:-60}"
OUT="${2:-results/demo_v1.mp4}"
ros2 launch hand_teleop_bringup teleop.launch.py demo_view:=true console:=terminal &
LAUNCH_PID=$!
trap 'kill $LAUNCH_PID 2>/dev/null || true' EXIT
sleep 5
python3 "$(dirname "$0")/record_demo_video.py" --seconds "$SECONDS_TO_RECORD" --out "$OUT"
