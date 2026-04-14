# Projector Live (CaBot Intent Projection)

Live floor-projection pipeline for CaBot intent visualization and negotiation signaling.

## Overview

This folder now uses native CaBot topics only:

- Trajectory projection from servo intent
- Blocked-state detection for debug metrics
- Negotiation gating from `/stop_reason` + `/people`
- Level 1 sound cue when negotiation state is active

Legacy YOLO-based human detection path was removed.

## Files

- `projector_arrow_live.py`: main ROS2 + PyQt projector node
- `run_projector_arrow.sh`: launcher (debug dump enabled by default)
- `resources/`: local assets (for example `signal_intention.wav`)
- `tmp/projector_debug/`: per-frame JSON/SVG and run summaries

## Quick Start

From `design/projector_live`:

```bash
./run_projector_arrow.sh
```

Defaults used by launcher:

- human topic: `/people`
- stop reason topic: `/stop_reason`
- scan topic: `/scan`
- motion topic: `/cabot/cmd_vel_adapter`
- odom topic: `/odom`

## Core Topics

- Servo intent: `/cabot/servo_target`
- Planner path (debug): `/plan`
- Commanded motion: `/cabot/cmd_vel_adapter`
- Actual motion: `/odom`
- Scan: `/scan`
- People: `/people`
- Stop reason: `/stop_reason`
- Touch: `/cabot/touch`
- Published blocked state: `/projector/blocked_state`
- Published haptic cue: `/cabot/vibrator1`

## Negotiation Logic (Current)

Level 1 cue is based on native people-aware stopping:

- stop reason must be recent and equal to `THERE_ARE_PEOPLE_IN_THE_PATH`
- person must be present on `/people` (fresh)
- if touch gating is enabled, touch must be active

This removed the old YOLO bool gating path.

## Debug Output

Debug files are written under:

- `tmp/projector_debug/`

Each run writes:

- frame dumps (`*.json`, `*.svg`)
- one run summary (`run_summary_*.json`, `run_summary_*.svg`) when the app exits

### Useful summary commands

```bash
latest=$(ls -1t ./tmp/projector_debug/run_summary_*.json | head -n1)
jq '.blocked_detection | {raw_event_count, confirmed_event_count, merged_event_count, confirmed_total_blocked_sec}' "$latest"
jq '.negotiation_block_detection | {event_count, total_blocked_sec, require_human_for_negotiation}' "$latest"
jq '.level1_sound | {attempt_count, success_count, last_status}' "$latest"
```

## Main Environment Variables

`run_projector_arrow.sh` supports:

- `PROJECTOR_TOPIC`
- `PROJECTOR_PATH_TOPIC`
- `PROJECTOR_MOTION_TOPIC`
- `PROJECTOR_ACTUAL_MOTION_TOPIC`
- `PROJECTOR_SCAN_TOPIC`
- `PROJECTOR_HUMAN_TOPIC`
- `PROJECTOR_STOP_REASON_TOPIC`
- `PROJECTOR_PEOPLE_TARGET_FRAME`
- `PROJECTOR_PEOPLE_FRONT_MAX_DIST`
- `PROJECTOR_PEOPLE_FRONT_HALF_ANGLE_DEG`
- `PROJECTOR_REQUIRE_HUMAN_FOR_NEGOTIATION`
- `PROJECTOR_BLOCKED_*`
- `PROJECTOR_LEVEL1_SOUND_PATH`
- `PROJECTOR_REQUIRE_TOUCH_FOR_SOUND`
- `PROJECTOR_TOUCH_TOPIC`
- `PROJECTOR_TOUCH_THRESHOLD`
- `PROJECTOR_TOUCH_HOLD_SEC`
- `PROJECTOR_HAPTIC_TOPIC`
- `PROJECTOR_HAPTIC_VALUE`
- `PROJECTOR_DEBUG_DUMP_DIR`
- `PROJECTOR_DEBUG_DUMP_EVERY`
- `PROJECTOR_DEBUG_MAX_DUMPS`
