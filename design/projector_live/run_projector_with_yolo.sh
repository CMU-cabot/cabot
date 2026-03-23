#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"


# Keep L1 negotiation cue strictly human-gated by default in combined mode.
: "${PROJECTOR_HUMAN_TOPIC:=/projector/human_in_front}"
: "${PROJECTOR_REQUIRE_HUMAN_FOR_NEGOTIATION:=1}"
export PROJECTOR_HUMAN_TOPIC PROJECTOR_REQUIRE_HUMAN_FOR_NEGOTIATION
# Clean up stale YOLO detector processes launched from this folder.
mapfile -t STALE_YOLO_PIDS < <(pgrep -f "$SCRIPT_DIR/yolo_human_detector.py" || true)
if [ "${#STALE_YOLO_PIDS[@]}" -gt 0 ]; then
  kill "${STALE_YOLO_PIDS[@]}" 2>/dev/null || true
  sleep 0.2
fi

# Start YOLO detector first in background
"$SCRIPT_DIR/run_yolo_human_detector.sh" &
YOLO_PID=$!

cleanup() {
  kill "$YOLO_PID" 2>/dev/null || true
  mapfile -t REMAINING_YOLO_PIDS < <(pgrep -f "$SCRIPT_DIR/yolo_human_detector.py" || true)
  if [ "${#REMAINING_YOLO_PIDS[@]}" -gt 0 ]; then
    kill "${REMAINING_YOLO_PIDS[@]}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# Short warm-up to let detector initialize
sleep "${PROJECTOR_YOLO_WARMUP_SEC:-2}"

# Run projector in foreground; exiting it will trigger cleanup trap
"$SCRIPT_DIR/run_projector_arrow.sh" "$@"
