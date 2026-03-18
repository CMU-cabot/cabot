# Projector Live (Servo Intent Projection)

Live fullscreen floor projection for CaBot intent visualization.

This tool renders a white projected trajectory based on `/cabot/servo_target` (Int16) and can overlay planner data (`/plan`) in debug outputs.

## What It Does

- Runs a fullscreen PyQt window on the projector screen.
- Subscribes to servo command topic (default: `/cabot/servo_target`).
- Draws a short-horizon projected intent path (white), updated in real time.
- Optionally consumes planner path (default: `/plan`) for debug comparison.
- Can dump per-frame debug artifacts (`.json` + `.svg`) and a run summary on exit (`Esc`).

## Files

- `projector_arrow_live.py`: Main ROS2 + rendering app.
- `run_projector_arrow.sh`: Convenience launcher (sources ROS Humble if available).
- `tmp/projector_debug/`: Debug dumps (when enabled).

## Requirements

- Python 3
- PyQt6
- ROS2 (tested with Humble setup script)
- ROS2 packages:
  - `rclpy`
  - `std_msgs` (required for servo topic)
  - `nav_msgs` (optional, for planner debug)

## Quick Start

From this folder:

```bash
./run_projector_arrow.sh --topic /cabot/servo_target --trail-sec 2.5
```

If left/right appears mirrored, add:

```bash
--invert
```

## Launcher Environment Variables

`run_projector_arrow.sh` also supports:

- `PROJECTOR_LINE_WIDTH`
- `PROJECTOR_ARENA_SCALE`
- `PROJECTOR_HEAD_LENGTH`
- `PROJECTOR_HEAD_WIDTH`
- `PROJECTOR_DEBUG_DUMP_DIR`
- `PROJECTOR_DEBUG_DUMP_EVERY`
- `PROJECTOR_DEBUG_MAX_DUMPS`
- `PROJECTOR_DEBUG` or `PROJECTOR_DEBUGF` (set to `1` to enable debug)

## Recommended Debug Run

```bash
rm -f ./tmp/projector_debug/* && ./run_projector_arrow.sh \
  --topic /cabot/servo_target \
  --path-topic /plan \
  --trail-sec 2.5 \
  --debug \
  --debug-dump-dir ./tmp/projector_debug \
  --debug-dump-every 1 \
  --debug-max-dumps 0
```

Press `Esc` to exit. On exit, a run summary is written to:

- `tmp/projector_debug/run_summary_<timestamp>.json`
- `tmp/projector_debug/run_summary_<timestamp>.svg`

## Main CLI Options

```text
--topic                      Servo Int16 topic (default: /cabot/servo_target)
--path-topic                 Planner path topic for debug (default: /plan)
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
--invert                     Invert left/right sign
```

## Debug Output Format

When debug is enabled, each sampled frame writes:

- `<time_ms>_<frame>.json`: Numeric snapshot (target/draw angles, path points, planner projection, comparison metrics)
- `<time_ms>_<frame>.svg`: Visual snapshot (white intent path + yellow planner path)

Run summary contains:

- White tip trace across the full run
- Yellow planner tip trace across the full run
- Planner path snapshots

## Tuning Notes

- For faster response: reduce `--deadband` and/or increase `--smoothing` response.
- For longer/shorter projection horizon: adjust `--trail-sec`.
- For visual thickness: adjust `--line-width`, `--head-length`, `--head-width`.
- If direction is opposite to physical movement, toggle `--invert`.

## Known Behavior

- Visual projection is servo-driven intent, not odometry replay.
- Planner data is used for debug comparison, not as the primary white visual path.
- If `nav_msgs` is unavailable, planner debug is automatically disabled.
