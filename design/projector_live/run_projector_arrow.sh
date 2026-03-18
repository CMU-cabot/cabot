#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi

EXTRA_ARGS=()
if [ -n "${PROJECTOR_HEAD_LENGTH:-}" ]; then
  EXTRA_ARGS+=(--head-length "$PROJECTOR_HEAD_LENGTH")
fi
if [ -n "${PROJECTOR_HEAD_WIDTH:-}" ]; then
  EXTRA_ARGS+=(--head-width "$PROJECTOR_HEAD_WIDTH")
fi
if [ -n "${PROJECTOR_DEBUG_DUMP_DIR:-}" ]; then
  EXTRA_ARGS+=(--debug-dump-dir "$PROJECTOR_DEBUG_DUMP_DIR")
  EXTRA_ARGS+=(--debug-dump-every "${PROJECTOR_DEBUG_DUMP_EVERY:-8}")
  EXTRA_ARGS+=(--debug-max-dumps "${PROJECTOR_DEBUG_MAX_DUMPS:-120}")
fi
if [ "${PROJECTOR_DEBUG:-0}" = "1" ] || [ "${PROJECTOR_DEBUGF:-0}" = "1" ]; then
  EXTRA_ARGS+=(--debug)
fi

python3 "$SCRIPT_DIR/projector_arrow_live.py"   --line-width "${PROJECTOR_LINE_WIDTH:-20}"   --arena-scale "${PROJECTOR_ARENA_SCALE:-0.42}"   "${EXTRA_ARGS[@]}"   "$@"
