#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

safe_source_setup() {
  local file="$1"
  if [ ! -f "$file" ]; then
    return 0
  fi
  # setup.bash files may reference optional vars (e.g., COLCON_TRACE)
  # that break under `set -u`.
  set +u
  # shellcheck source=/dev/null
  source "$file"
  set -u
}

if [ -f /opt/ros/humble/setup.bash ]; then
  safe_source_setup /opt/ros/humble/setup.bash
fi

REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
if [ -f "$REPO_ROOT/install/setup.bash" ]; then
  safe_source_setup "$REPO_ROOT/install/setup.bash"
fi

# If msgs are missing in the main overlay, try the repo's host_ws overlay.
if ! python3 -c "from cabot_msgs.msg import StopReason; from people_msgs.msg import People" >/dev/null 2>&1; then
  safe_source_setup "$REPO_ROOT/host_ws/install/setup.bash"
fi

# Preflight: stop_reason/people need cabot_msgs + people_msgs in this Python env.
if ! python3 -c "from cabot_msgs.msg import StopReason; from people_msgs.msg import People" >/dev/null 2>&1; then
  {
    echo "[run_projector_arrow] WARNING: cabot_msgs/people_msgs are not importable in current Python env."
    echo "[run_projector_arrow] stop_reason and/or /people handling may be disabled in projector_arrow_live.py."
    echo "[run_projector_arrow] Tried overlays:"
    echo "[run_projector_arrow]   $REPO_ROOT/install/setup.bash"
    echo "[run_projector_arrow]   $REPO_ROOT/host_ws/install/setup.bash"
    echo "[run_projector_arrow] Note: this repo has duplicate package names, so naive colcon build at repo root may fail."
  } >&2
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
if [ "${PROJECTOR_BLOCKED_REQUIRE_FRONT_OBSTACLE:-1}" = "1" ]; then
  EXTRA_ARGS+=(--blocked-require-front-obstacle)
else
  EXTRA_ARGS+=(--no-blocked-require-front-obstacle)
fi
if [ "${PROJECTOR_REQUIRE_TOUCH_FOR_SOUND:-0}" = "0" ]; then
  EXTRA_ARGS+=(--no-require-touch-for-sound)
else
  EXTRA_ARGS+=(--require-touch-for-sound)
fi

python3 "$SCRIPT_DIR/projector_arrow_live.py" \
  --topic "${PROJECTOR_TOPIC:-/cabot/servo_target}" \
  --path-topic "${PROJECTOR_PATH_TOPIC:-/plan}" \
  --deadband "${PROJECTOR_DEADBAND:-0.0}" \
  --motion-topic "${PROJECTOR_MOTION_TOPIC:-/cabot/cmd_vel_adapter}" \
  --actual-motion-topic "${PROJECTOR_ACTUAL_MOTION_TOPIC:-/odom}" \
  --scan-topic "${PROJECTOR_SCAN_TOPIC:-/scan}" \
  --human-topic "${PROJECTOR_HUMAN_TOPIC:-/people}" \
  --people-target-frame "${PROJECTOR_PEOPLE_TARGET_FRAME:-base_footprint}" \
  --people-front-max-dist "${PROJECTOR_PEOPLE_FRONT_MAX_DIST:-2.0}" \
  --people-front-half-angle-deg "${PROJECTOR_PEOPLE_FRONT_HALF_ANGLE_DEG:-60.0}" \
  --touch-topic "${PROJECTOR_TOUCH_TOPIC:-/cabot/touch}" \
  --stop-reason-topic "${PROJECTOR_STOP_REASON_TOPIC:-/stop_reason}" \
  --blocked-topic "${PROJECTOR_BLOCKED_TOPIC:-/projector/blocked_state}" \
  --human-hold "${PROJECTOR_HUMAN_HOLD:-0.0}" \
  --human-fresh-sec "${PROJECTOR_HUMAN_FRESH_SEC:-0.9}" \
  --human-active-min-for-neg "${PROJECTOR_HUMAN_ACTIVE_MIN_FOR_NEG:-0.15}" \
  --neg-enter-hold "${PROJECTOR_NEG_ENTER_HOLD:-0.25}" \
  --neg-exit-hold "${PROJECTOR_NEG_EXIT_HOLD:-0.50}" \
  --neg-person-entry-window "${PROJECTOR_NEG_PERSON_ENTRY_WINDOW:-0.80}" \
  --blocked-moving-linear-threshold "${PROJECTOR_BLOCKED_MOVING_LINEAR_THRESHOLD:-0.04}" \
  --blocked-moving-angular-threshold "${PROJECTOR_BLOCKED_MOVING_ANGULAR_THRESHOLD:-0.25}" \
  --blocked-clear-linear-threshold "${PROJECTOR_BLOCKED_CLEAR_LINEAR_THRESHOLD:-0.08}" \
  --blocked-clear-angular-threshold "${PROJECTOR_BLOCKED_CLEAR_ANGULAR_THRESHOLD:-0.40}" \
  --blocked-obstacle-front-half-angle-deg "${PROJECTOR_BLOCKED_OBS_FRONT_HALF_ANGLE_DEG:-45.0}" \
  --blocked-obstacle-front-max-dist "${PROJECTOR_BLOCKED_OBS_FRONT_MAX_DIST:-0.75}" \
  --blocked-obstacle-min-fraction "${PROJECTOR_BLOCKED_OBS_MIN_FRACTION:-0.12}" \
  --blocked-obstacle-scan-timeout "${PROJECTOR_BLOCKED_OBS_SCAN_TIMEOUT:-0.8}" \
  --blocked-enter-hold "${PROJECTOR_BLOCKED_ENTER_HOLD:-0.00}" \
  --blocked-exit-hold "${PROJECTOR_BLOCKED_EXIT_HOLD:-0.70}" \
  --blocked-confirm-min "${PROJECTOR_BLOCKED_CONFIRM_MIN:-1.00}" \
  --level1-sound-path "${PROJECTOR_LEVEL1_SOUND_PATH:-$SCRIPT_DIR/resources/signal_intention.wav}" \
  --touch-threshold "${PROJECTOR_TOUCH_THRESHOLD:-1}" \
  --touch-hold-sec "${PROJECTOR_TOUCH_HOLD_SEC:-0.25}" \
  --level1-sound-min-play "${PROJECTOR_LEVEL1_SOUND_MIN_PLAY:-0.8}" \
  --haptic-topic "${PROJECTOR_HAPTIC_TOPIC:-/cabot/vibrator1}" \
  --haptic-value "${PROJECTOR_HAPTIC_VALUE:-1}" \
  --line-width "${PROJECTOR_LINE_WIDTH:-20}" \
  --arena-scale "${PROJECTOR_ARENA_SCALE:-0.42}" \
  --trail-sec "${PROJECTOR_TRAIL_SEC:-2.5}" \
  "${EXTRA_ARGS[@]}" \
  "$@"
