import math
import time

from PyQt6 import QtCore, QtGui, QtWidgets

from config import DisplayConfig
from model.arrow_state import ArrowState

class ArrowWidget(QtWidgets.QWidget):
    def __init__(self, cfg: DisplayConfig, state: ArrowState):
        super().__init__()
        self._cfg = cfg
        self._state = state
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_OpaquePaintEvent, True)
        self.setFocusPolicy(QtCore.Qt.FocusPolicy.StrongFocus)
        self.setCursor(QtCore.Qt.CursorShape.BlankCursor)
        self._bg_color = QtGui.QColor(*self._cfg.bg_rgba)

        self._body_pen = QtGui.QPen(QtGui.QColor(*self._cfg.arrow_rgba), self._cfg.line_width_px)
        self._body_pen.setCapStyle(QtCore.Qt.PenCapStyle.RoundCap)
        self._body_pen.setJoinStyle(QtCore.Qt.PenJoinStyle.RoundJoin)

        self._flow_pen = QtGui.QPen(QtGui.QColor(*self._cfg.flow_rgba), max(2.0, self._cfg.line_width_px * 0.35))
        self._flow_pen.setCapStyle(QtCore.Qt.PenCapStyle.RoundCap)
        self._flow_pen.setJoinStyle(QtCore.Qt.PenJoinStyle.RoundJoin)
        self._flow_pen.setStyle(QtCore.Qt.PenStyle.CustomDashLine)
        self._flow_pen.setDashPattern([24.0, 20.0])
        self._paint_samples_ms = []

    def paintEvent(self, event):  # noqa: N802 (linter to ignore warning about method camel case name)
        paint_start_sec = time.perf_counter()
        _ = event
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, self._cfg.use_antialiasing)
        painter.fillRect(self.rect(), self._bg_color)

        screen_w_px = float(self.width())
        screen_h_px = float(self.height())
        arrow_origin_x_px = screen_w_px * 0.5
        arrow_origin_y_px = screen_h_px * self._cfg.origin_y_ratio
        arrow_len_px = screen_h_px * self._cfg.arrow_len_ratio
        arrow_head_len = self._cfg.arrow_head_len_px
        arrow_head_width = self._cfg.arrow_head_wing_px
        arrow_rad = math.radians(self._state.current_arrow_direction)

        # Forward screen direction is up.
        fwd_x = math.sin(arrow_rad)
        fwd_y = -math.cos(arrow_rad)
        left_x = -fwd_y
        left_y = fwd_x

        front_span_px = self._distance_to_screen_edge(start_x_px = arrow_origin_x_px, start_y_px = arrow_origin_y_px,
            dir_x = fwd_x, dir_y = fwd_y, screen_min_x = 0.0, screen_max_x = screen_w_px, screen_min_y = 0.0, screen_max_y = screen_h_px)
        back_span_px = self._distance_to_screen_edge(start_x_px = arrow_origin_x_px, start_y_px = arrow_origin_y_px,
            dir_x = -fwd_x, dir_y = -fwd_y, screen_min_x = 0.0, screen_max_x = screen_w_px, screen_min_y = 0.0, screen_max_y = screen_h_px)
        
        wrap_span_px = max(1.0, front_span_px + back_span_px)
        phase_px = (self._state.arrow_translation_offset - self._cfg.arrow_entry_offset_px) % wrap_span_px

        self._draw_arrow_artifact(painter = painter, base_x = arrow_origin_x_px, base_y = arrow_origin_y_px,
            fwd_x = fwd_x, fwd_y = fwd_y, left_x = left_x, left_y = left_y, arrow_len_px = arrow_len_px,
            arrow_head_len = arrow_head_len, arrow_head_width = arrow_head_width, move_along_fwd_px = phase_px,
            body_visible_len_px = arrow_len_px, draw_head = True)

        forward_artifact_extent_px = arrow_len_px + arrow_head_len
        initial_front_overflow_px = max(0.0, forward_artifact_extent_px - front_span_px)
        current_front_overflow_px = max(0.0, (phase_px + forward_artifact_extent_px) - front_span_px)
        extra_front_overflow_px = current_front_overflow_px - initial_front_overflow_px

        # Only show wrapped re-entry when newly consumed beyond the startup state.
        if extra_front_overflow_px > 1.0:
            wrapped_body_visible_len_px = min(arrow_len_px, extra_front_overflow_px)
            wrapped_head_threshold_px = arrow_len_px + max(8.0, (arrow_head_len * 0.55))
            wrapped_draw_head = extra_front_overflow_px >= wrapped_head_threshold_px
            self._draw_arrow_artifact(painter = painter, base_x = arrow_origin_x_px, base_y = arrow_origin_y_px,
                fwd_x = fwd_x, fwd_y = fwd_y, left_x = left_x, left_y = left_y, arrow_len_px = arrow_len_px,
                arrow_head_len = arrow_head_len, arrow_head_width = arrow_head_width, move_along_fwd_px = (phase_px - wrap_span_px),
                body_visible_len_px = wrapped_body_visible_len_px, draw_head = wrapped_draw_head)
        painter.end()
        
        # Performance logging the paintEvent. Check if paint is taking too long and causing frame drops.
        if self._cfg.enable_perf_log:
            paint_ms = (time.perf_counter() - paint_start_sec) * 1000.0
            self._paint_samples_ms.append(paint_ms)

    def _draw_arrow_artifact(self, painter: QtGui.QPainter, base_x: float, base_y: float, fwd_x: float, fwd_y: float,
        left_x: float, left_y: float, arrow_len_px: float, arrow_head_len: float,
        arrow_head_width: float, move_along_fwd_px: float, body_visible_len_px: float, draw_head: bool) -> None:

        shift_x = fwd_x * move_along_fwd_px
        shift_y = fwd_y * move_along_fwd_px
        effective_body_len_px = max(0.0, min(arrow_len_px, body_visible_len_px))
        if effective_body_len_px <= 1.0:
            return

        tail = QtCore.QPointF(base_x + shift_x, base_y + shift_y)
        tip = QtCore.QPointF(tail.x() + fwd_x * effective_body_len_px, tail.y() + fwd_y * effective_body_len_px)

        # Curved body profile:
        # start tangent is mostly "up" (rising feel), then transitions to final heading at the tip.
        up_x, up_y = 0.0, -1.0
        turn_amount = min(1.0, abs(fwd_x))
        start_blend = 0.18 + (0.18 * turn_amount)
        start_dir_x = (up_x * (1.0 - start_blend)) + (fwd_x * start_blend)
        start_dir_y = (up_y * (1.0 - start_blend)) + (fwd_y * start_blend)
        start_norm = max(1e-6, math.hypot(start_dir_x, start_dir_y))
        start_dir_x /= start_norm
        start_dir_y /= start_norm
        end_dir_x = fwd_x
        end_dir_y = fwd_y

        control_start_distance_px = effective_body_len_px * 0.46
        control_end_distance_px = effective_body_len_px * 0.34
        control_point_start = QtCore.QPointF(
            tail.x() + (start_dir_x * control_start_distance_px),
            tail.y() + (start_dir_y * control_start_distance_px),
        )
        control_point_end = QtCore.QPointF(
            tip.x() - (end_dir_x * control_end_distance_px),
            tip.y() - (end_dir_y * control_end_distance_px),
        )
        body_path = QtGui.QPainterPath(tail)
        body_path.cubicTo(control_point_start, control_point_end, tip)

        painter.setPen(self._body_pen)
        painter.drawPath(body_path)

        if self._cfg.draw_flow_overlay:
            if self._cfg.animate_flow_overlay:
                self._flow_pen.setDashOffset(-self._state.flow_animation_offset)
            painter.setPen(self._flow_pen)
            painter.drawPath(body_path)

        if not draw_head:
            return
        if effective_body_len_px < (arrow_len_px - 1.0):
            return

        # Use cubic end tangent for head direction so the tip follows the curve naturally.
        head_dir_x = tip.x() - control_point_end.x()
        head_dir_y = tip.y() - control_point_end.y()
        head_norm = max(1e-6, math.hypot(head_dir_x, head_dir_y))
        head_dir_x /= head_norm
        head_dir_y /= head_norm
        head_left_x = -head_dir_y
        head_left_y = head_dir_x
        head_wing_width = arrow_head_width * (1.0 - 0.35 * turn_amount)

        left = QtCore.QPointF(
            tip.x() - head_dir_x * arrow_head_len + head_left_x * head_wing_width,
            tip.y() - head_dir_y * arrow_head_len + head_left_y * head_wing_width,
        )
        right = QtCore.QPointF(
            tip.x() - head_dir_x * arrow_head_len - head_left_x * head_wing_width,
            tip.y() - head_dir_y * arrow_head_len - head_left_y * head_wing_width,
        )
        head_polygon = QtGui.QPolygonF([tip, left, right])
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.setBrush(self._body_pen.color())
        painter.drawPolygon(head_polygon)
        painter.setBrush(QtCore.Qt.BrushStyle.NoBrush)

    @staticmethod
    def _distance_to_screen_edge(start_x_px: float, start_y_px: float, dir_x: float, dir_y: float, 
            screen_min_x: float, screen_max_x: float, screen_min_y: float, screen_max_y: float) -> float:
        near_zero = 1e-6
        hit_distances = []

        if dir_x > near_zero:
            distance_to_x_edge = (screen_max_x - start_x_px) / dir_x
            if distance_to_x_edge >= 0.0:
                hit_distances.append(distance_to_x_edge)
        elif dir_x < -near_zero:
            distance_to_x_edge = (screen_min_x - start_x_px) / dir_x
            if distance_to_x_edge >= 0.0:
                hit_distances.append(distance_to_x_edge)

        if dir_y > near_zero:
            distance_to_y_edge = (screen_max_y - start_y_px) / dir_y
            if distance_to_y_edge >= 0.0:
                hit_distances.append(distance_to_y_edge)
        elif dir_y < -near_zero:
            distance_to_y_edge = (screen_min_y - start_y_px) / dir_y
            if distance_to_y_edge >= 0.0:
                hit_distances.append(distance_to_y_edge)

        if not hit_distances:
            return max(screen_max_x - screen_min_x, screen_max_y - screen_min_y)
        return min(hit_distances)

    def consume_paint_stats(self):
        if not self._paint_samples_ms:
            return None
        sorted_samples = sorted(self._paint_samples_ms)
        count = len(sorted_samples)
        p95_index = min(count - 1, max(0, int(count * 0.95) - 1))
        stats = {
            "count": count,
            "avg_ms": sum(sorted_samples) / count,
            "p95_ms": sorted_samples[p95_index],
            "max_ms": sorted_samples[-1],
        }
        self._paint_samples_ms = []
        return stats
