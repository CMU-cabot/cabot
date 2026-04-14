# Display (Step 1)

This is a clean restart of the display stack, isolated from ROS and negotiation logic.

Current scope:
- Fullscreen arrow renderer
- Smooth heading state model
- Low-latency frame loop (precise timer)
- Animated flow stroke on top of the main arrow

Run:

```bash
python3 design/projector_live/scripts/display/main.py --screen 1
```

Useful keys during demo:
- Left: turn target heading left
- Right: turn target heading right
- Up: reset heading to forward
- Esc: exit

