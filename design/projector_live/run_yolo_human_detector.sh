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

python3 "$SCRIPT_DIR/yolo_human_detector.py" \
  --model "${YOLO_HUMAN_MODEL:-$SCRIPT_DIR/yolo26n.pt}" \
  --image-topic "${YOLO_HUMAN_IMAGE_TOPIC:-/camera/color/image_raw}" \
  --out-topic "${YOLO_HUMAN_OUT_TOPIC:-/projector/human_in_front}" \
  --max-fps "${YOLO_HUMAN_MAX_FPS:-5.0}" \
  --conf "${YOLO_HUMAN_CONF:-0.35}" \
  --object-conf "${YOLO_HUMAN_OBJECT_CONF:-0.25}" \
  --imgsz "${YOLO_HUMAN_IMGSZ:-640}" \
  --device "${YOLO_HUMAN_DEVICE:-}" \
  --front-roi-width "${YOLO_HUMAN_FRONT_ROI_WIDTH:-0.65}" \
  --min-box-area "${YOLO_HUMAN_MIN_BOX_AREA:-0.010}" \
  --hold-sec "${YOLO_HUMAN_HOLD_SEC:-0.6}" \
  "$@"
