# Projector Live (CaBot Intent Projection)

Live floor-projection pipeline for CaBot intent visualization and negotiation debugging.

## Overview

This folder contains:

- Real-time projector renderer (servo-intent trajectory)
- Optional planner overlay in debug artifacts
- Blocked-event detection summaries
- YOLO-based human-in-front signal for negotiation gating
- YOLO bbox snapshot debugging utilities

## Files

- `projector_arrow_live.py`: main ROS2 + PyQt projection node
- `run_projector_arrow.sh`: projector launcher (debug enabled by default)
- `yolo_human_detector.py`: YOLO ROS2 detector publishing `/projector/human_in_front`
- `run_yolo_human_detector.sh`: YOLO launcher
- `run_projector_with_yolo.sh`: one-command launcher (YOLO background + projector foreground)
- `resources/`: local assets (model/audio/image files)
- `tmp/projector_debug/`: per-frame and run-summary projector debug dumps
- `tmp/yolo_blocked_snapshots/`: YOLO snapshot dumps (`.jpg` + `.json`)

## Quick Start

From `design/projector_live`:

```bash
./run_projector_with_yolo.sh
```

This starts YOLO first, then projector, and stops YOLO when projector exits.

Projector-only:

```bash
./run_projector_arrow.sh
```

YOLO-only:

```bash
./run_yolo_human_detector.sh
```

YOLO verbose diagnostics:

```bash
./run_yolo_human_detector.sh --verbose
```

## Current Negotiation Behavior

- Blocked detection is from projector side (robot near-stopped + front obstacle evidence).
- Negotiation block is human-gated by default in combined mode.
- Level 1 audio cue is triggered by negotiation block (not raw blocked).
- If negotiation block clears, audio is stopped immediately.
- If still negotiation-blocked after clip ends, Level 1 can replay (respecting cooldown).

## Core Topics

- Servo intent: `/cabot/servo_target`
- Planner path: `/plan`
- Actual motion: `/odom`
- Scan gating: `/scan`
- Human output bool: `/projector/human_in_front`
- Human confidence: `/projector/human_in_front_confidence`
- Projector blocked state pub: `/projector/blocked_state`

## Debug Outputs

Projector debug (`tmp/projector_debug/`):

- Per-frame JSON and SVG
- Run summary JSON and SVG on `Esc`

YOLO debug (`tmp/yolo_blocked_snapshots/`):

- `blocked_*.jpg/.json`: snapshots captured while blocked-gate is active
- `always_*.jpg/.json`: snapshots captured in always-snapshot mode

Snapshot metadata includes `snapshot_source` so you can tell trigger origin, e.g.:

- `blocked_burst_pre`
- `blocked_edge_callback`
- `blocked_burst_post`
- `blocked_retry_no_human`
- `blocked_gate_periodic`

## Useful Commands

Latest run summary quick view:

```bash
latest=$(ls -1t ./tmp/projector_debug/run_summary_*.json | head -n1)
jq '.blocked_detection | {event_count, raw_event_count, confirmed_total_blocked_sec}' "$latest"
jq '.negotiation_block_detection | {event_count, total_blocked_sec, require_human_for_negotiation}' "$latest"
jq '.level1_sound | {attempt_count, success_count, last_status}' "$latest"
```

One-line verdict:

```bash
latest=$(ls -1t ./tmp/projector_debug/run_summary_*.json | head -n1); jq '{blocked_any: (.blocked_detection | {event_count, confirmed_total_blocked_sec}), blocked_human: (.negotiation_block_detection | {event_count, total_blocked_sec}), level1: (.level1_sound | {attempt_count, success_count, last_status})}' "$latest"
```

Human signal logging to file:

```bash
ros2 topic echo /projector/human_in_front > ./tmp/human_in_front.log 2>&1
```

YOLO verbose logging to file:

```bash
./run_yolo_human_detector.sh --verbose 2>&1 | tee ./tmp/yolo_verbose.log
```

## Always-Snapshot Debug Mode

Capture YOLO bbox snapshots even when robot is not blocked:

```bash
YOLO_HUMAN_SNAPSHOT_ALWAYS=1 ./run_yolo_human_detector.sh --verbose
```

Inspect recent snapshot files:

```bash
ls -1 ./tmp/yolo_blocked_snapshots | tail -n 20
```

## Main Environment Variables

Projector launch (`run_projector_arrow.sh`):

- `PROJECTOR_TOPIC`
- `PROJECTOR_PATH_TOPIC`
- `PROJECTOR_ACTUAL_MOTION_TOPIC`
- `PROJECTOR_SCAN_TOPIC`
- `PROJECTOR_HUMAN_TOPIC`
- `PROJECTOR_HUMAN_HOLD`
- `PROJECTOR_REQUIRE_HUMAN_FOR_NEGOTIATION`
- `PROJECTOR_BLOCKED_*`
- `PROJECTOR_LEVEL1_SOUND_PATH`
- `PROJECTOR_HAPTIC_TOPIC`
- `PROJECTOR_HAPTIC_VALUE`
- `PROJECTOR_LINE_WIDTH`
- `PROJECTOR_ARENA_SCALE`
- `PROJECTOR_TRAIL_SEC`
- `PROJECTOR_HEAD_LENGTH`
- `PROJECTOR_HEAD_WIDTH`
- `PROJECTOR_DEBUG_DUMP_DIR`
- `PROJECTOR_DEBUG_DUMP_EVERY`
- `PROJECTOR_DEBUG_MAX_DUMPS`

YOLO launch (`run_yolo_human_detector.sh`):

- `YOLO_HUMAN_MODEL`
- `YOLO_HUMAN_IMAGE_TOPIC`
- `YOLO_HUMAN_ROTATE` (`0`, `90`, `180`, `270`)
- `YOLO_HUMAN_OUT_TOPIC`
- `YOLO_HUMAN_BLOCKED_TOPIC`
- `YOLO_HUMAN_BLOCKED_HOLD_SEC`
- `YOLO_HUMAN_SNAPSHOT_RETRY_NO_HUMAN_SEC`
- `YOLO_HUMAN_SNAPSHOT_RETRY_MAX_PER_BLOCK`
- `YOLO_HUMAN_MAX_FPS`
- `YOLO_HUMAN_CONF`
- `YOLO_HUMAN_OBJECT_CONF`
- `YOLO_HUMAN_IMGSZ`
- `YOLO_HUMAN_DEVICE`
- `YOLO_HUMAN_FRONT_ROI_WIDTH`
- `YOLO_HUMAN_MIN_BOX_AREA`
- `YOLO_HUMAN_HOLD_SEC`
- `YOLO_HUMAN_SNAPSHOT_DIR`
- `YOLO_HUMAN_SNAPSHOT_COOLDOWN_SEC`
- `YOLO_HUMAN_SNAPSHOT_MAX`
- `YOLO_HUMAN_SNAPSHOT_ALWAYS` (`1` enables always-snapshot mode)
- `YOLO_HUMAN_SNAPSHOT_USE_PROJECTOR_BLOCKED` (`1` default)

Combined launcher (`run_projector_with_yolo.sh`):

- `PROJECTOR_YOLO_WARMUP_SEC`

## Notes

- White projection is servo-driven intent, not odometry replay.
- Planner is for debug comparison; it does not drive the main white path.
- Combined launcher defaults negotiation to human-gated behavior.
