from dataclasses import dataclass
import math

# normalizes into [-180, 180] ranges. Like: 190 becomes -170, -220 becomes 140
def _wrap_to_180_deg(angle_deg: float) -> float:
    a = float(angle_deg)
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a

@dataclass
class ArrowState:
    current_arrow_direction: float = 0.0
    arrow_point_towards: float = 0.0
    arrow_translation_offset: float = 0.0
    flow_animation_offset: float = 0.0

    def update(self, dt_sec: float, smoothing_hz: float, arrow_speed_px_s: float, flow_speed_px_s: float) -> None:
        dt = max(0.0, float(dt_sec))

        # Exponential smoothing using response frequency (Hz).
        # alpha = 1 - exp(-2*pi*f*dt)
        alpha = 1.0 - math.exp(-2.0 * math.pi * max(0.0, smoothing_hz) * dt)
        err = _wrap_to_180_deg(self.arrow_point_towards - self.current_arrow_direction)
        self.current_arrow_direction = _wrap_to_180_deg(self.current_arrow_direction + alpha * err)

        self.arrow_translation_offset += max(0.0, float(arrow_speed_px_s)) * dt
        self.flow_animation_offset += max(0.0, float(flow_speed_px_s)) * dt
