#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi

# Work around PyTorch/OpenMP TLS errors on ARM by preloading system libgomp.
LIBGOMP_PATH="/lib/aarch64-linux-gnu/libgomp.so.1"
if [ -f "$LIBGOMP_PATH" ]; then
  if [ -n "${LD_PRELOAD:-}" ]; then
    export LD_PRELOAD="$LIBGOMP_PATH:$LD_PRELOAD"
  else
    export LD_PRELOAD="$LIBGOMP_PATH"
  fi
fi

SNAPSHOT_ALWAYS_ARGS=()
if [ "${YOLO_HUMAN_SNAPSHOT_ALWAYS:-0}" = "1" ]; then
  SNAPSHOT_ALWAYS_ARGS+=(--snapshot-always)
fi

SNAPSHOT_BLOCKED_GATE_ARGS=()
if [ "${YOLO_HUMAN_SNAPSHOT_USE_PROJECTOR_BLOCKED:-1}" = "0" ]; then
  SNAPSHOT_BLOCKED_GATE_ARGS+=(--no-snapshot-use-projector-blocked)
fi

python3 "$SCRIPT_DIR/yolo_human_detector.py" \
  --model "${YOLO_HUMAN_MODEL:-$SCRIPT_DIR/resources/yolo26n.pt}" \
  --image-topic "${YOLO_HUMAN_IMAGE_TOPIC:-/rs1/color/image_raw}" \
  --rotate "${YOLO_HUMAN_ROTATE:-180}" \
  --out-topic "${YOLO_HUMAN_OUT_TOPIC:-/projector/human_in_front}" \
  --blocked-topic "${YOLO_HUMAN_BLOCKED_TOPIC:-/projector/blocked_state}" \
  --blocked-hold-sec "${YOLO_HUMAN_BLOCKED_HOLD_SEC:-0.25}" \
  --snapshot-retry-no-human-sec "${YOLO_HUMAN_SNAPSHOT_RETRY_NO_HUMAN_SEC:-0.25}" \
  --snapshot-retry-max-per-block "${YOLO_HUMAN_SNAPSHOT_RETRY_MAX_PER_BLOCK:-4}" \
  --max-fps "${YOLO_HUMAN_MAX_FPS:-15.0}" \
  --conf "${YOLO_HUMAN_CONF:-0.35}" \
  --object-conf "${YOLO_HUMAN_OBJECT_CONF:-0.25}" \
  --imgsz "${YOLO_HUMAN_IMGSZ:-640}" \
  --device "${YOLO_HUMAN_DEVICE:-}" \
  --front-roi-width "${YOLO_HUMAN_FRONT_ROI_WIDTH:-1.0}" \
  --min-box-area "${YOLO_HUMAN_MIN_BOX_AREA:-0.010}" \
  --near-box-area "${YOLO_HUMAN_NEAR_BOX_AREA:-0.050}" \
  --hold-sec "${YOLO_HUMAN_HOLD_SEC:-0.0}" \
  --snapshot-dir "${YOLO_HUMAN_SNAPSHOT_DIR:-$SCRIPT_DIR/tmp/yolo_blocked_snapshots}" \
  --snapshot-cooldown-sec "${YOLO_HUMAN_SNAPSHOT_COOLDOWN_SEC:-1.0}" \
  --snapshot-max "${YOLO_HUMAN_SNAPSHOT_MAX:-0}" \
  "${SNAPSHOT_ALWAYS_ARGS[@]}" \
  "${SNAPSHOT_BLOCKED_GATE_ARGS[@]}" \
  "$@"
