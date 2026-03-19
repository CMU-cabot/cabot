# Projector Live (Servo Intent Projection)

Live fullscreen floor projection for CaBot intent visualization.

This tool renders a white projected trajectory based on `/cabot/servo_target` (Int16) and can overlay planner data (`/plan`) in debug outputs.

## What It Does

- Runs a fullscreen PyQt window on the projector screen.
- Subscribes to servo command topic (default: `/cabot/servo_target`).
- Draws a short-horizon projected intent path (white), updated in real time.
- Optionally consumes planner path (default: `/plan`) for debug comparison.
- Detects blocked state from commanded motion (`/cabot/cmd_vel`) versus actual motion (`/odom`) and logs transitions/counts.
- Can dump per-frame debug artifacts (`.json` + `.svg`) and a run summary on exit (`Esc`).

## Files

- `projector_arrow_live.py`: Main ROS2 + rendering app.
- `run_projector_arrow.sh`: Convenience launcher (sources ROS Humble if available).
- `yolo_human_detector.py`: Optional YOLO ROS node publishing person presence (`/projector/human_in_front`).
- `run_yolo_human_detector.sh`: Convenience launcher for YOLO person detector.
- `yolo_tracking.py`: Legacy offline script (not used by current live pipeline).
- `tmp/projector_debug/`: Debug dumps (when enabled).

## Requirements

- Python 3
- PyQt6
- ROS2 (tested with Humble setup script)
- ROS2 packages:
  - `rclpy`
  - `std_msgs` (required for servo topic)
  - `nav_msgs` (optional, for planner debug)
  - `sensor_msgs` (for laser and camera topics)
- Optional for person-aware negotiation gating:
  - `ultralytics`
  - `opencv-python`
  - `cv_bridge`

## Quick Start

From this folder:

```bash
./run_projector_arrow.sh
```

The launcher now enables debug by default and clears previous files in `tmp/projector_debug/` before each run.

Optional: run YOLO person detector in parallel (recommended if you want negotiation to count only human-caused blocks):

```bash
./run_yolo_human_detector.sh
```

YOLO launcher default model is:

- `design/projector_live/yolo26n.pt`

Override example:

```bash
YOLO_HUMAN_MODEL=./yolo26n.pt ./run_yolo_human_detector.sh
```

## Launcher Environment Variables

`run_projector_arrow.sh` supports environment overrides, including:

- `PROJECTOR_TOPIC`
- `PROJECTOR_PATH_TOPIC`
- `PROJECTOR_MOTION_TOPIC`
- `PROJECTOR_ACTUAL_MOTION_TOPIC`
- `PROJECTOR_SCAN_TOPIC`
- `PROJECTOR_HUMAN_TOPIC`
- `PROJECTOR_HUMAN_HOLD`
- `PROJECTOR_REQUIRE_HUMAN_FOR_NEGOTIATION` (`1` or `0`)
- `PROJECTOR_BLOCKED_DEMAND_LINEAR_THRESHOLD`
- `PROJECTOR_BLOCKED_DEMAND_ANGULAR_THRESHOLD`
- `PROJECTOR_BLOCKED_MOVING_LINEAR_THRESHOLD`
- `PROJECTOR_BLOCKED_MOVING_ANGULAR_THRESHOLD`
- `PROJECTOR_BLOCKED_CLEAR_LINEAR_THRESHOLD`
- `PROJECTOR_BLOCKED_CLEAR_ANGULAR_THRESHOLD`
- `PROJECTOR_BLOCKED_ENTER_HOLD`
- `PROJECTOR_BLOCKED_EXIT_HOLD`
- `PROJECTOR_BLOCKED_DEMAND_HOLD`
- `PROJECTOR_BLOCKED_CONFIRM_MIN`
- `PROJECTOR_BLOCKED_OBS_FRONT_HALF_ANGLE_DEG`
- `PROJECTOR_BLOCKED_OBS_FRONT_MAX_DIST`
- `PROJECTOR_BLOCKED_OBS_MIN_FRACTION`
- `PROJECTOR_BLOCKED_OBS_SCAN_TIMEOUT`
- `PROJECTOR_LINE_WIDTH`
- `PROJECTOR_ARENA_SCALE`
- `PROJECTOR_TRAIL_SEC`
- `PROJECTOR_HEAD_LENGTH`
- `PROJECTOR_HEAD_WIDTH`
- `PROJECTOR_DEBUG_DUMP_DIR`
- `PROJECTOR_DEBUG_DUMP_EVERY` (default: `1`)
- `PROJECTOR_DEBUG_MAX_DUMPS` (default: `0`, unlimited)
- `YOLO_HUMAN_MODEL`
- `YOLO_HUMAN_IMAGE_TOPIC`
- `YOLO_HUMAN_OUT_TOPIC`
- `YOLO_HUMAN_MAX_FPS`
- `YOLO_HUMAN_CONF`
- `YOLO_HUMAN_OBJECT_CONF`
- `YOLO_HUMAN_IMGSZ`
- `YOLO_HUMAN_DEVICE`
- `YOLO_HUMAN_FRONT_ROI_WIDTH`
- `YOLO_HUMAN_MIN_BOX_AREA`
- `YOLO_HUMAN_HOLD_SEC`

## Recommended Debug Run

```bash
./run_projector_arrow.sh
```

For person-aware negotiation logging, run detector and projector in separate terminals:

```bash
# terminal 1
./run_yolo_human_detector.sh

# terminal 2
./run_projector_arrow.sh
```

Press `Esc` to exit. On exit, a run summary is written to:

- `tmp/projector_debug/run_summary_<timestamp>.json`
- `tmp/projector_debug/run_summary_<timestamp>.svg`

## Main CLI Options

```text
--topic                      Servo Int16 topic (default: /cabot/servo_target)
--path-topic                 Planner path topic for debug (default: /plan)
--motion-topic               Commanded motion topic (default: /cabot/cmd_vel)
--actual-motion-topic        Actual odom topic (default: /odom)
--scan-topic                 LaserScan topic used for front obstacle gating (default: /scan)
--human-topic                Bool topic for person presence (default: /projector/human_in_front)
--screen                     Target screen index (default: 1)
--deadband                   Ignore small servo angles around 0
--smoothing                  Servo smoothing/response
--fps                        Render rate target
--speed                      Path growth speed (px/s)
--trail-sec                  Projection horizon length
--line-width                 White path thickness base
--head-length                Arrow head length
--head-width                 Arrow head half-width
--arena-scale                Path length scaling in screen space
--spike-delta                Reject sudden servo spikes
--reverse-trigger            Reverse-candidate threshold
--reverse-confirm-samples    Reverse-candidate confirmation
--debug-dump-dir             Debug output directory
--debug-dump-every           Dump every N ticks
--debug-max-dumps            Max dumped frames (<=0 means unlimited)
--debug / --debugf           Enable debug dumps to default folder
--blocked-demand-linear-threshold   cmd_vel linear threshold for demand-active
--blocked-demand-angular-threshold  cmd_vel angular threshold for demand-active
--blocked-moving-linear-threshold   odom linear threshold for blocked candidate
--blocked-moving-angular-threshold  odom angular threshold for blocked candidate
--blocked-clear-linear-threshold    odom linear threshold for clear evidence
--blocked-clear-angular-threshold   odom angular threshold for clear evidence
--blocked-require-front-obstacle    Require /scan front-obstacle evidence (default)
--no-blocked-require-front-obstacle Disable /scan front-obstacle gating
--blocked-obstacle-front-half-angle-deg  Front scan half-angle for gating
--blocked-obstacle-front-max-dist        Front obstacle max distance (m)
--blocked-obstacle-min-fraction          Min front-ray fraction below distance
--blocked-obstacle-scan-timeout          Max scan staleness (s) for gating
--require-human-for-negotiation          Count negotiation blocks only when human is present
--no-require-human-for-negotiation       Disable human requirement for negotiation count
--human-hold                              Hold person-presence signal for N seconds
--blocked-enter-hold                Candidate hold before blocked=True
--blocked-exit-hold                 Clear hold before blocked=False
--blocked-demand-hold               Keep demand active after latest cmd_vel
--blocked-confirm-min               Min seconds for confirmed blocked count
```

## Debug Output Format

Debug is enabled by default. Each sampled frame writes:

- `<time_ms>_<frame>.json`: Numeric snapshot (target/draw angles, path points, planner projection, comparison metrics, blocked detection state/decision)
- `<time_ms>_<frame>.svg`: Visual snapshot (white intent path + yellow planner path)

Run summary contains:

- White tip trace across the full run
- Yellow planner tip trace across the full run
- Planner path snapshots
- Blocked event count (`event_count`: confirmed), raw event count, total blocked time, and per-event start/end/duration
- Negotiation block events (`blocked && human_present`) and their durations
- Optional object cue: the YOLO detector prints `object` to terminal when front non-person objects are detected

You can inspect both metrics quickly:

```bash
latest=$(ls -1t ./tmp/projector_debug/run_summary_*.json | head -n1)
jq '.blocked_detection | {event_count, raw_event_count, confirmed_total_blocked_sec}' "$latest"
jq '.negotiation_block_detection | {event_count, total_blocked_sec}' "$latest"
```

## Tuning Notes

- For faster response: reduce `--deadband` and/or increase `--smoothing` response.
- For longer/shorter projection horizon: adjust `--trail-sec`.
- For visual thickness: adjust `--line-width`, `--head-length`, `--head-width`.

## Known Behavior

- Visual projection is servo-driven intent, not odometry replay.
- Planner data is used for debug comparison, not as the primary white visual path.
- If `nav_msgs` is unavailable, planner debug is automatically disabled.
