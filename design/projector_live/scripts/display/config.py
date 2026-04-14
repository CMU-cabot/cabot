from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen = True)
class DisplayConfig:
    # General
    fps: int = 60
    screen_index: int = 1
    show_fullscreen: bool = True

    # Performance
    enable_perf_log: bool = False
    render_backend: str = "opengl"  # "opengl" or "qt"
    opengl_vsync: bool = True
    use_antialiasing: bool = False

    # Visualization
    draw_flow_overlay: bool = True
    animate_flow_overlay: bool = False

    # Servo topic
    use_servo_topic: bool = True
    servo_topic: str = "/cabot/servo_target"
    servo_fresh_sec: float = 0.8

    # Geometry
    line_width_px: float = 120.0 # relative to screen height
    arrow_head_len_px: float = 200.0 # relative to screen height
    arrow_head_wing_px: float = 150.0 # relative to head length
    arrow_len_ratio: float = 0.75  # relative to screen height
    origin_y_ratio: float = 0.7 # relative to screen height, where the arrow tail is anchored
    arrow_entry_offset_px: float = 120.0  # larger value = softer/less sudden bottom entry

    # Dynamics
    heading_smoothing_hz: float = 9.0 # how fast the arrow responds to changes 
    arrow_speed_px_s: float = 600.0  # whole arrow artifact translation speed
    flow_speed_px_s: float = 100.0   # cyan dash animation speed

    # Colors (r, g, b, a)
    bg_rgba: Tuple[int, int, int, int] = (0, 0, 0, 255)
    arrow_rgba: Tuple[int, int, int, int] = (255, 255, 255, 245)
    flow_rgba: Tuple[int, int, int, int] = (130, 255, 255, 230)
