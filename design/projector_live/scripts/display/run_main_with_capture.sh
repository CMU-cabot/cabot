#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECTOR_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
OUT_DIR="${PROJECTOR_ROOT}/tmp/auto_screen_debug_$(date +%Y%m%d_%H%M%S)"
CAPTURE_INTERVAL_SEC="${CAPTURE_INTERVAL_SEC:-0.4}"

if ! command -v gnome-screenshot >/dev/null 2>&1; then
  echo "[capture] ERROR: gnome-screenshot not found"
  exit 1
fi

mkdir -p "${OUT_DIR}"
echo "[capture] output_dir=${OUT_DIR}"

python3 "${SCRIPT_DIR}/main.py" "$@" &
APP_PID=$!
echo "[capture] main_pid=${APP_PID}"

count=0
while kill -0 "${APP_PID}" 2>/dev/null; do
  ts="$(date +%s%3N)"
  if gnome-screenshot -f "${OUT_DIR}/screen_${ts}.png" >/dev/null 2>&1; then
    count=$((count + 1))
  fi
  sleep "${CAPTURE_INTERVAL_SEC}"
done

wait "${APP_PID}" || true
echo "[capture] stopped screenshots=${count}"
echo "[capture] output_dir=${OUT_DIR}"
