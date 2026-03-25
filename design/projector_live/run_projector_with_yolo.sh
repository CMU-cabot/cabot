#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEBUG_DIR="${PROJECTOR_DEBUG_DUMP_DIR:-$SCRIPT_DIR/tmp/projector_debug}"
PROJECTOR_RC=0

# YOLO launcher should use bool human topic from detector output, not /people.
: "${PROJECTOR_HUMAN_SOURCE:=bool}"
: "${PROJECTOR_HUMAN_TOPIC:=/projector/human_in_front}"
: "${PROJECTOR_REQUIRE_HUMAN_FOR_NEGOTIATION:=1}"
export PROJECTOR_HUMAN_SOURCE PROJECTOR_HUMAN_TOPIC PROJECTOR_REQUIRE_HUMAN_FOR_NEGOTIATION

print_run_summary() {
  local latest
  latest="$(ls -1t "$DEBUG_DIR"/run_summary_*.json 2>/dev/null | head -n1 || true)"

  if [ -z "$latest" ]; then
    echo "[run_projector_with_yolo] no run_summary found in $DEBUG_DIR"
    return
  fi

  echo "[run_projector_with_yolo] latest summary: $latest"

  if ! command -v jq >/dev/null 2>&1; then
    echo "[run_projector_with_yolo] jq not found; skipping auto summary output"
    return
  fi

  jq '{blocked_raw_events: .blocked_detection.raw_event_count, blocked_confirmed_events: .blocked_detection.confirmed_event_count, blocked_encounters: (.blocked_detection.merged_event_count // .blocked_detection.confirmed_event_count), blocked_events_with_sound: ([.blocked_detection.easy_blocks[]? | select(.sound_triggered == true)] | length), negotiation_state_events: .negotiation_block_detection.event_count, sound_attempts: .level1_sound.attempt_count, sound_success: .level1_sound.success_count, sound_last_status: .level1_sound.last_status}' "$latest" || true

  echo "[run_projector_with_yolo] block -> snapshots"
  jq -r '.blocked_detection.easy_blocks[]? | "  - \(.blocked_id): sound=\(.sound_count // 0) neg=\(.neg_count // 0) snaps=\((.snapshot_files // []) | if length==0 then "(none)" else join(", ") end)"' "$latest" || true
}

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
  print_run_summary
}
trap cleanup EXIT

# Short warm-up to let detector initialize
sleep "${PROJECTOR_YOLO_WARMUP_SEC:-2}"

# Run projector in foreground; exiting it will trigger cleanup trap
set +e
"$SCRIPT_DIR/run_projector_arrow.sh" "$@"
PROJECTOR_RC=$?
set -e

exit "$PROJECTOR_RC"
