#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi

EXTRA_ARGS=()
DEBUG_DIR="${PROJECTOR_DEBUG_DUMP_DIR:-$SCRIPT_DIR/tmp/projector_debug}"
DEBUG_DUMP_EVERY="${PROJECTOR_DEBUG_DUMP_EVERY:-1}"
DEBUG_MAX_DUMPS="${PROJECTOR_DEBUG_MAX_DUMPS:-0}"

mkdir -p "$DEBUG_DIR"
rm -f "$DEBUG_DIR"/*

if [ -n "${PROJECTOR_HEAD_LENGTH:-}" ]; then
  EXTRA_ARGS+=(--head-length "$PROJECTOR_HEAD_LENGTH")
fi
if [ -n "${PROJECTOR_HEAD_WIDTH:-}" ]; then
  EXTRA_ARGS+=(--head-width "$PROJECTOR_HEAD_WIDTH")
fi
EXTRA_ARGS+=(--debug)
EXTRA_ARGS+=(--debug-dump-dir "$DEBUG_DIR")
EXTRA_ARGS+=(--debug-dump-every "$DEBUG_DUMP_EVERY")
EXTRA_ARGS+=(--debug-max-dumps "$DEBUG_MAX_DUMPS")

python3 "$SCRIPT_DIR/projector_arrow_live.py" \
  --topic "${PROJECTOR_TOPIC:-/cabot/servo_target}" \
  --path-topic "${PROJECTOR_PATH_TOPIC:-/plan}" \
  --line-width "${PROJECTOR_LINE_WIDTH:-20}" \
  --arena-scale "${PROJECTOR_ARENA_SCALE:-0.42}" \
  --trail-sec "${PROJECTOR_TRAIL_SEC:-2.5}" \
  "${EXTRA_ARGS[@]}" \
  "$@"
