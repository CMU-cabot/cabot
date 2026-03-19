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
if [ "${PROJECTOR_REQUIRE_HUMAN_FOR_NEGOTIATION:-1}" = "0" ]; then
  EXTRA_ARGS+=(--no-require-human-for-negotiation)
else
  EXTRA_ARGS+=(--require-human-for-negotiation)
fi

python3 "$SCRIPT_DIR/projector_arrow_live.py" \
  --topic "${PROJECTOR_TOPIC:-/cabot/servo_target}" \
  --path-topic "${PROJECTOR_PATH_TOPIC:-/plan}" \
  --motion-topic "${PROJECTOR_MOTION_TOPIC:-/cabot/cmd_vel}" \
  --actual-motion-topic "${PROJECTOR_ACTUAL_MOTION_TOPIC:-/odom}" \
  --scan-topic "${PROJECTOR_SCAN_TOPIC:-/scan}" \
  --human-topic "${PROJECTOR_HUMAN_TOPIC:-/projector/human_in_front}" \
  --human-hold "${PROJECTOR_HUMAN_HOLD:-0.8}" \
  --blocked-demand-linear-threshold "${PROJECTOR_BLOCKED_DEMAND_LINEAR_THRESHOLD:-0.05}" \
  --blocked-demand-angular-threshold "${PROJECTOR_BLOCKED_DEMAND_ANGULAR_THRESHOLD:-0.20}" \
  --blocked-moving-linear-threshold "${PROJECTOR_BLOCKED_MOVING_LINEAR_THRESHOLD:-0.04}" \
  --blocked-moving-angular-threshold "${PROJECTOR_BLOCKED_MOVING_ANGULAR_THRESHOLD:-0.25}" \
  --blocked-clear-linear-threshold "${PROJECTOR_BLOCKED_CLEAR_LINEAR_THRESHOLD:-0.08}" \
  --blocked-clear-angular-threshold "${PROJECTOR_BLOCKED_CLEAR_ANGULAR_THRESHOLD:-0.40}" \
  --blocked-obstacle-front-half-angle-deg "${PROJECTOR_BLOCKED_OBS_FRONT_HALF_ANGLE_DEG:-25.0}" \
  --blocked-obstacle-front-max-dist "${PROJECTOR_BLOCKED_OBS_FRONT_MAX_DIST:-0.75}" \
  --blocked-obstacle-min-fraction "${PROJECTOR_BLOCKED_OBS_MIN_FRACTION:-0.12}" \
  --blocked-obstacle-scan-timeout "${PROJECTOR_BLOCKED_OBS_SCAN_TIMEOUT:-0.8}" \
  --blocked-enter-hold "${PROJECTOR_BLOCKED_ENTER_HOLD:-0.45}" \
  --blocked-exit-hold "${PROJECTOR_BLOCKED_EXIT_HOLD:-0.70}" \
  --blocked-demand-hold "${PROJECTOR_BLOCKED_DEMAND_HOLD:-1.20}" \
  --blocked-confirm-min "${PROJECTOR_BLOCKED_CONFIRM_MIN:-1.00}" \
  --line-width "${PROJECTOR_LINE_WIDTH:-20}" \
  --arena-scale "${PROJECTOR_ARENA_SCALE:-0.42}" \
  --trail-sec "${PROJECTOR_TRAIL_SEC:-2.5}" \
  "${EXTRA_ARGS[@]}" \
  "$@"
