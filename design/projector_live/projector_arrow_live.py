#!/usr/bin/env python3
import argparse
import json
import math
import threading
import time
from collections import deque
from pathlib import Path

from PyQt6 import QtCore, QtGui, QtWidgets


def wrap_deg(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def circular_mean_deg(values) -> float:
    if not values:
        return 0.0
    sx = 0.0
    sy = 0.0
    for v in values:
        r = math.radians(v)
        sx += math.cos(r)
        sy += math.sin(r)
    if abs(sx) < 1e-9 and abs(sy) < 1e-9:
        return 0.0
    return wrap_deg(math.degrees(math.atan2(sy, sx)))


def default_debug_dir() -> str:
    return str((Path(__file__).resolve().parent / "tmp" / "projector_debug").resolve())


class RosBridge(QtCore.QObject):
    angle_changed = QtCore.pyqtSignal(float)
    planned_path_changed = QtCore.pyqtSignal(object, float)
    status = QtCore.pyqtSignal(str)

    def __init__(self, topic: str, invert_sign: bool, path_topic: str):
        super().__init__()
        self.topic = topic
        self.invert_sign = invert_sign
        self.path_topic = path_topic
        self._stop_event = threading.Event()
        self._thread = None

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None

    def _run(self):
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import Int16
        except Exception as ex:
            self.status.emit(f"rclpy/std_msgs import failed: {ex}")
            return

        has_path = False
        try:
            from nav_msgs.msg import Path  # type: ignore
            has_path = True
        except Exception as ex:
            self.status.emit(f"nav_msgs not available: {ex}")
            Path = None  # type: ignore

        class BridgeNode(Node):
            def __init__(self, bridge: RosBridge):
                super().__init__("projector_arrow_live")
                self._bridge = bridge
                self.create_subscription(Int16, bridge.topic, self._cb_servo, 10)

                if has_path and bridge.path_topic:
                    self.create_subscription(Path, bridge.path_topic, self._cb_path, 10)  # type: ignore

            def _cb_servo(self, msg):
                value = float(msg.data)
                if self._bridge.invert_sign:
                    value = -value
                self._bridge.angle_changed.emit(value)

            def _cb_path(self, msg):
                try:
                    points = []
                    for ps in msg.poses:
                        p = ps.pose.position
                        points.append((float(p.x), float(p.y)))

                    stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
                    if stamp <= 0:
                        stamp = time.time()
                    self._bridge.planned_path_changed.emit(points, stamp)
                except Exception:
                    pass

        node = None
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            node = BridgeNode(self)
            self.status.emit(
                "subscribed "
                f"servo={self.topic} "
                f"path={self.path_topic if has_path else 'disabled'}"
            )
            while rclpy.ok() and not self._stop_event.is_set():
                rclpy.spin_once(node, timeout_sec=0.05)
        except Exception as ex:
            self.status.emit(f"ROS loop error: {ex}")
        finally:
            try:
                if node is not None:
                    node.destroy_node()
            except Exception:
                pass
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass


class DynamicPathProjectorWindow(QtWidgets.QWidget):
    def __init__(
        self,
        screen_index: int,
        deadband_deg: float,
        smoothing: float,
        fps: int,
        speed_px: float,
        trail_sec: float,
        line_width: float,
        head_length: float,
        head_width: float,
        arena_scale: float,
        spike_delta_deg: float,
        reverse_trigger_deg: float,
        reverse_confirm_samples: int,
        debug_dump_dir: str,
        debug_dump_every: int,
        debug_max_dumps: int,
        invert_sign: bool,
    ):
        super().__init__()
        self.setWindowFlag(QtCore.Qt.WindowType.FramelessWindowHint, True)
        self.setCursor(QtCore.Qt.CursorShape.BlankCursor)
        self.setWindowTitle("Projector Dynamic Path Arrow")
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_OpaquePaintEvent, True)

        screens = QtWidgets.QApplication.screens()
        use_index = screen_index if 0 <= screen_index < len(screens) else 0
        self._screen = screens[use_index]

        self._target_angle = 0.0
        self._draw_angle = 0.0
        self._deadband = max(0.0, deadband_deg)

        if smoothing <= 1.0:
            self._response_hz = 12.0 + (34.0 * max(0.01, smoothing))
        else:
            self._response_hz = smoothing

        self._artifact_scale = 3.00
        self._stroke_scale = 2.00
        self._path_speed = max(40.0, speed_px) * self._artifact_scale
        self._trail_sec = max(0.4, trail_sec)
        self._line_width = max(1.0, line_width) * self._artifact_scale * self._stroke_scale
        self._head_length = max(12.0, head_length) * self._artifact_scale
        self._head_width = max(8.0, head_width) * self._artifact_scale
        self._size_scale = min(1.0, max(0.2, arena_scale))

        self._max_turn_rate_deg_s = 280.0
        self._angle_jitter_deg = 0.6
        self._turn_reset_deg = 32.0
        self._servo_target_blend = 0.80
        self._servo_to_turn_rate_gain = 1.45
        self._draw_steer_cmd_deg = 0.0
        self._turn_rate_slew_up_deg_s2 = 5200.0
        self._turn_rate_slew_down_deg_s2 = 1300.0
        self._turn_rate_filter_attack_hz = 15.0
        self._turn_rate_filter_release_hz = 5.0
        self._turn_rate_deadband_deg_s = 2.0
        self._turn_rate_cmd_deg_s = 0.0
        self._turn_rate_smooth_deg_s = 0.0
        self._preview_turn_ramp_sec = 0.11
        self._center_snap_deg = 4.0
        self._center_snap_samples = 3
        self._center_snap_count = 0
        self._center_target_limit_deg = 12.0
        self._center_snap_hz = 18.0

        self._spike_delta_deg = max(20.0, spike_delta_deg)
        self._reverse_trigger_deg = max(90.0, reverse_trigger_deg)
        self._reverse_confirm_samples = max(1, reverse_confirm_samples)
        self._max_forward_turn_deg = 88.0
        self._reverse_pending = 0
        self._last_raw_angle = None
        self._recent_angles = deque(maxlen=2)

        self._origin_x = 0.0
        self._origin_y = 0.0
        self._max_path_len_px = self._path_speed * self._trail_sec

        # Path shown on projection.
        self._points = []
        self._path_len_px = 0.0
        self._reset_count = 0

        self._intent_mode = "servo"
        self._intent_scale_px_m = 180.0
        self._planned_path_world = []
        self._planned_path_stamp = 0.0
        self._planned_path_max_age_sec = 120.0
        self._intent_scale_smooth_alpha = 0.18

        self._clock = time.perf_counter
        self._last_tick = self._clock()

        self._debug_dir = Path(debug_dump_dir).expanduser() if debug_dump_dir else None
        self._debug_every = max(1, debug_dump_every)
        self._debug_max = None if debug_max_dumps <= 0 else max(1, debug_max_dumps)
        self._invert_sign = invert_sign
        self._debug_frame = 0
        self._debug_count = 0
        self._run_started_wall = time.time()
        self._run_summary_written = False
        self._run_white_tip_trace = []  # [(x, y), ...]
        self._run_yellow_tip_trace = []  # [(x, y), ...]
        self._run_yellow_paths = []  # [{"time": t, "points": [(x, y), ...]}, ...]
        self._run_last_yellow_sig = None
        if self._debug_dir is not None:
            self._debug_dir.mkdir(parents=True, exist_ok=True)

        interval_ms = max(8, int(1000 / max(24, fps)))
        self._timer = QtCore.QTimer(self)
        self._timer.setTimerType(QtCore.Qt.TimerType.PreciseTimer)
        self._timer.timeout.connect(self._tick)
        self._timer.start(interval_ms)

        self.show()
        QtCore.QTimer.singleShot(0, self._attach_screen)
        QtGui.QShortcut(QtGui.QKeySequence("Escape"), self, activated=self.close)

    def _attach_screen(self):
        handle = self.windowHandle()
        if handle is not None:
            handle.setScreen(self._screen)
        self.setGeometry(self._screen.geometry())
        self.showFullScreen()

        self._origin_x = self.width() * 0.5
        # Match small_app/debug feel: path tail starts almost at screen bottom.
        self._origin_y = self.height() * 0.99

        size_ref = min(self.width(), self.height())
        scale_len = 0.65 + 1.05 * self._size_scale
        self._max_path_len_px = min(self._path_speed * self._trail_sec, size_ref * scale_len)
        # Robot-centric view: keep nominal straight preview within visible upward span.
        up_room = max(120.0, self._origin_y - max(24.0, self._line_width * 0.8))
        self._max_path_len_px = min(self._max_path_len_px, up_room)
        self._reset_path("init")

    def _reset_path(self, _reason: str):
        self._points = [QtCore.QPointF(self._origin_x, self._origin_y)]
        self._path_len_px = 0.0
        self._reset_count += 1
        self._append_trace_point(self._run_white_tip_trace, self._points[0], min_dist_px=0.2)

    @QtCore.pyqtSlot(object, float)
    def on_planned_path(self, points, stamp: float):
        cleaned = []
        if points is not None:
            for p in points:
                if isinstance(p, (list, tuple)) and len(p) >= 2:
                    x = float(p[0])
                    y = float(p[1])
                    if math.isfinite(x) and math.isfinite(y):
                        cleaned.append((x, y))
        self._planned_path_world = cleaned
        self._planned_path_stamp = float(stamp)

    def _planned_path_projected_points(self):
        if len(self._planned_path_world) < 2:
            return []
        if self._planned_path_stamp > 0.0:
            age = time.time() - self._planned_path_stamp
            if age > self._planned_path_max_age_sec:
                return []

        pts_world = self._planned_path_world
        x0, y0 = pts_world[0]

        ref_heading = None
        for i in range(1, len(pts_world)):
            dx = pts_world[i][0] - pts_world[i - 1][0]
            dy = pts_world[i][1] - pts_world[i - 1][1]
            if math.hypot(dx, dy) > 1e-6:
                ref_heading = math.atan2(dy, dx)
                break
        if ref_heading is None:
            return []

        c = math.cos(ref_heading)
        s = math.sin(ref_heading)
        local = []
        for px, py in pts_world:
            dx = px - x0
            dy = py - y0
            forward = c * dx + s * dy
            left = -s * dx + c * dy
            local.append((forward, left))

        f_vals = [f for f, _ in local]
        l_vals = [l for _, l in local]
        f_min, f_max = min(f_vals), max(f_vals)
        l_min, l_max = min(l_vals), max(l_vals)
        f_span = max(0.20, f_max - f_min)
        l_span = max(0.12, l_max - l_min)

        # Match planner debug scale/dimensions to the white servo trajectory:
        # use the same max trail length budget and origin-centered footprint.
        desired_by_len = self._max_path_len_px / f_span
        desired_by_width = (self._max_path_len_px * 0.95) / l_span

        # Keep inside screen limits similarly to live trajectory bounds.
        margin = max(2.0, self._line_width * 0.6)
        usable_h = max(40.0, self.height() - 2.0 * margin)
        usable_w = max(40.0, self.width() - 2.0 * margin)
        desired_by_h = usable_h / f_span
        desired_by_w = usable_w / l_span

        desired_scale = max(
            40.0,
            min(2400.0, desired_by_len, desired_by_width, desired_by_h, desired_by_w),
        )
        if self._intent_scale_px_m <= 0.0:
            self._intent_scale_px_m = desired_scale
        else:
            a = self._intent_scale_smooth_alpha
            self._intent_scale_px_m = (1.0 - a) * self._intent_scale_px_m + a * desired_scale
        scale = self._intent_scale_px_m

        out = []
        for forward, left in local:
            left_vis = -left if self._invert_sign else left
            sx = self._origin_x - left_vis * scale
            sy = self._origin_y - (forward - f_min) * scale
            out.append(QtCore.QPointF(sx, sy))
        return out

    @staticmethod
    def _heading_deg_from_points(points):
        if len(points) < 2:
            return None
        p0 = points[-2]
        p1 = points[-1]
        dx = p1.x() - p0.x()
        dy = p1.y() - p0.y()
        if math.hypot(dx, dy) < 1e-6:
            return None
        return math.degrees(math.atan2(dx, -dy))

    @staticmethod
    def _align_polyline_to_heading(points, source_heading_deg: float, target_heading_deg: float, target_anchor):
        if not points:
            return []
        src_anchor = points[0]

        hs = math.radians(source_heading_deg)
        ht = math.radians(target_heading_deg)

        # Screen-space forward basis (heading 0 -> up).
        src_fx = math.sin(hs)
        src_fy = -math.cos(hs)
        src_lx = src_fy
        src_ly = -src_fx

        tgt_fx = math.sin(ht)
        tgt_fy = -math.cos(ht)
        tgt_lx = tgt_fy
        tgt_ly = -tgt_fx

        out = []
        ax = target_anchor.x()
        ay = target_anchor.y()
        sx0 = src_anchor.x()
        sy0 = src_anchor.y()
        for p in points:
            vx = p.x() - sx0
            vy = p.y() - sy0
            f = vx * src_fx + vy * src_fy
            l = vx * src_lx + vy * src_ly
            nx = ax + f * tgt_fx + l * tgt_lx
            ny = ay + f * tgt_fy + l * tgt_ly
            out.append(QtCore.QPointF(nx, ny))
        return out

    @staticmethod
    def _trim_tail_points(points, max_len_px: float):
        if len(points) <= 2:
            return list(points)

        keep = [points[-1]]
        acc = 0.0
        for i in range(len(points) - 2, -1, -1):
            p0 = points[i]
            p1 = points[i + 1]
            seg = math.hypot(p1.x() - p0.x(), p1.y() - p0.y())
            if seg <= 1e-6:
                continue
            if acc + seg > max_len_px:
                rem = max_len_px - acc
                if rem > 1e-6:
                    t = rem / seg
                    cx = p1.x() + (p0.x() - p1.x()) * t
                    cy = p1.y() + (p0.y() - p1.y()) * t
                    keep.append(QtCore.QPointF(cx, cy))
                break
            keep.append(p0)
            acc += seg

        keep.reverse()
        return keep

    @staticmethod
    def _polyline_length(points) -> float:
        if len(points) < 2:
            return 0.0
        total = 0.0
        for i in range(1, len(points)):
            p0 = points[i - 1]
            p1 = points[i]
            total += math.hypot(p1.x() - p0.x(), p1.y() - p0.y())
        return total

    @staticmethod
    def _append_trace_point(trace, pt: QtCore.QPointF, min_dist_px: float = 0.8):
        x = float(pt.x())
        y = float(pt.y())
        if not trace:
            trace.append((x, y))
            return
        lx, ly = trace[-1]
        if math.hypot(x - lx, y - ly) >= max(0.0, min_dist_px):
            trace.append((x, y))

    def _build_servo_preview_path(self, turn_rate_deg_s: float):
        # Build a short-horizon intent preview from "now", not from accumulated past trail.
        # This keeps the projection robot-centric (forward is always visually up).
        step_dt = 1.0 / 60.0
        seg_len = self._path_speed * step_dt
        max_len = max(12.0, self._max_path_len_px)
        steps = max(2, int(math.ceil(max_len / max(1e-6, seg_len))))

        x = self._origin_x
        y = self._origin_y
        heading_deg = 0.0
        points = [QtCore.QPointF(x, y)]

        for i in range(steps):
            # Ease-in curvature along the predicted horizon so near-field intent remains stable
            # and the line avoids a rigid "instant bend" look.
            t = (i + 1) * step_dt
            ramp = 1.0 - math.exp(-t / max(1e-3, self._preview_turn_ramp_sec))
            turn_step_deg_s = turn_rate_deg_s * ramp

            heading_deg = wrap_deg(heading_deg + turn_step_deg_s * step_dt)
            rad = math.radians(heading_deg)
            ux = math.sin(rad)
            uy = -math.cos(rad)
            x += ux * seg_len
            y += uy * seg_len
            points.append(QtCore.QPointF(x, y))

            if len(points) >= 2 and self._polyline_length(points) >= max_len:
                break

        return points, heading_deg

    @staticmethod
    def _planner_signature(points):
        if len(points) < 2:
            return ("empty", len(points))
        mid = points[len(points) // 2]
        return (
            len(points),
            round(points[0].x(), 1),
            round(points[0].y(), 1),
            round(mid.x(), 1),
            round(mid.y(), 1),
            round(points[-1].x(), 1),
            round(points[-1].y(), 1),
        )

    def _capture_run_planner(self, now: float, planned_proj):
        if len(planned_proj) >= 1:
            self._append_trace_point(self._run_yellow_tip_trace, planned_proj[-1], min_dist_px=0.8)
        if len(planned_proj) < 2:
            return
        sig = self._planner_signature(planned_proj)
        if sig == self._run_last_yellow_sig:
            return
        self._run_last_yellow_sig = sig
        self._run_yellow_paths.append(
            {
                "time": float(now),
                "points": [(float(p.x()), float(p.y())) for p in planned_proj],
            }
        )

    def _write_run_summary(self):
        if self._run_summary_written or self._debug_dir is None:
            return
        self._run_summary_written = True

        try:
            end_wall = time.time()
            duration = max(0.0, end_wall - self._run_started_wall)
            stem = f"run_summary_{int(end_wall * 1000)}"
            json_path = self._debug_dir / f"{stem}.json"
            svg_path = self._debug_dir / f"{stem}.svg"

            summary = {
                "run_started_wall": self._run_started_wall,
                "run_ended_wall": end_wall,
                "run_duration_sec": duration,
                "debug_frames_dumped": self._debug_count,
                "white_tip_trace_count": len(self._run_white_tip_trace),
                "yellow_tip_trace_count": len(self._run_yellow_tip_trace),
                "yellow_path_snapshots_count": len(self._run_yellow_paths),
                "white_tip_trace": [{"x": x, "y": y} for x, y in self._run_white_tip_trace],
                "yellow_tip_trace": [{"x": x, "y": y} for x, y in self._run_yellow_tip_trace],
                "yellow_path_snapshots": [
                    {"time": entry["time"], "points": [{"x": x, "y": y} for x, y in entry["points"]]}
                    for entry in self._run_yellow_paths
                ],
            }
            json_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")

            w = max(1, self.width())
            h = max(1, self.height())
            parts = [
                f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}">',
                '<rect x="0" y="0" width="100%" height="100%" fill="black"/>',
            ]

            for entry in self._run_yellow_paths:
                pts = entry["points"]
                if len(pts) < 2:
                    continue
                pstr = " ".join(f"{x:.2f},{y:.2f}" for x, y in pts)
                parts.append(
                    f'<polyline points="{pstr}" fill="none" stroke="#ffcc00" stroke-width="2.0" '
                    'stroke-linecap="round" stroke-linejoin="round" opacity="0.18"/>'
                )

            if len(self._run_yellow_tip_trace) >= 2:
                pstr = " ".join(f"{x:.2f},{y:.2f}" for x, y in self._run_yellow_tip_trace)
                parts.append(
                    f'<polyline points="{pstr}" fill="none" stroke="#ffcc00" stroke-width="4.0" '
                    'stroke-linecap="round" stroke-linejoin="round" opacity="0.60"/>'
                )

            if len(self._run_white_tip_trace) >= 2:
                pstr = " ".join(f"{x:.2f},{y:.2f}" for x, y in self._run_white_tip_trace)
                parts.append(
                    f'<polyline points="{pstr}" fill="none" stroke="white" stroke-width="{self._line_width * 1.2:.2f}" '
                    'stroke-linecap="round" stroke-linejoin="round" opacity="0.30"/>'
                )
                parts.append(
                    f'<polyline points="{pstr}" fill="none" stroke="white" stroke-width="{self._line_width * 0.65:.2f}" '
                    'stroke-linecap="round" stroke-linejoin="round" opacity="0.95"/>'
                )

            if self._run_white_tip_trace:
                sx, sy = self._run_white_tip_trace[0]
                ex, ey = self._run_white_tip_trace[-1]
                parts.append(
                    f'<circle cx="{sx:.2f}" cy="{sy:.2f}" r="7" fill="#50fa7b" opacity="0.90"/>'
                )
                parts.append(
                    f'<circle cx="{ex:.2f}" cy="{ey:.2f}" r="7" fill="#ff5555" opacity="0.90"/>'
                )

            parts.append("</svg>")
            svg_path.write_text("\n".join(parts), encoding="utf-8")
        except Exception:
            pass

    @QtCore.pyqtSlot(float)
    def on_servo_angle(self, angle_deg: float):
        angle_deg = wrap_deg(angle_deg)

        if abs(angle_deg) < self._deadband:
            angle_deg = 0.0

        if self._last_raw_angle is not None:
            delta_raw = abs(wrap_deg(angle_deg - self._last_raw_angle))
            if delta_raw > self._spike_delta_deg:
                return
        self._last_raw_angle = angle_deg

        # Intent projection should follow forward motion with turns, not flip backward.
        # Reject reverse-like servo values that create false U-turn visuals.
        if abs(angle_deg) >= self._reverse_trigger_deg:
            # Treat reverse-like servo values as invalid for floor intent visualization.
            # Fall back to "go forward" instead of latching a wrong hard turn.
            self._reverse_pending += 1
            angle_deg = 0.0
        else:
            self._reverse_pending = 0

        angle_deg = max(-self._max_forward_turn_deg, min(self._max_forward_turn_deg, angle_deg))

        self._recent_angles.append(angle_deg)
        filt = circular_mean_deg(self._recent_angles)

        delta = wrap_deg(filt - self._target_angle)
        if abs(delta) < self._angle_jitter_deg:
            pass
        else:
            self._target_angle = wrap_deg(self._target_angle + self._servo_target_blend * delta)

        if abs(angle_deg) <= self._center_snap_deg:
            self._center_snap_count += 1
        else:
            self._center_snap_count = 0

    def _head_triangle(self):
        if len(self._points) < 2:
            return None
        tip = self._points[-1]
        lookback = max(self._line_width * 2.0, self._head_length * 0.7)
        rem = lookback
        back = tip

        for i in range(len(self._points) - 2, -1, -1):
            curr = self._points[i + 1]
            prev = self._points[i]
            sx = curr.x() - prev.x()
            sy = curr.y() - prev.y()
            seg = math.hypot(sx, sy)
            if seg < 1e-6:
                continue
            if seg >= rem:
                t = rem / seg
                back = QtCore.QPointF(curr.x() - sx * t, curr.y() - sy * t)
                break
            rem -= seg
            back = prev

        vx = tip.x() - back.x()
        vy = tip.y() - back.y()
        vlen = math.hypot(vx, vy)
        if vlen < 1e-6:
            return None

        ux = vx / vlen
        uy = vy / vlen
        px = -uy
        py = ux

        tip_forward = max(4.0, self._line_width * 0.6)
        tip_x = tip.x() + ux * tip_forward
        tip_y = tip.y() + uy * tip_forward
        base_x = tip_x - ux * self._head_length
        base_y = tip_y - uy * self._head_length

        left_x = base_x + px * self._head_width
        left_y = base_y + py * self._head_width
        right_x = base_x - px * self._head_width
        right_y = base_y - py * self._head_width

        return {
            "tip": (tip_x, tip_y),
            "base": (base_x, base_y),
            "left": (left_x, left_y),
            "right": (right_x, right_y),
            "dir": (ux, uy),
        }

    def _tip_outside(self, pt: QtCore.QPointF) -> bool:
        return pt.x() < 0 or pt.x() > self.width() or pt.y() < 0 or pt.y() > self.height()

    def _debug_dump(self, now: float):
        if self._debug_dir is None:
            return
        self._debug_frame += 1
        if self._debug_frame % self._debug_every != 0:
            return
        if self._debug_max is not None and self._debug_count >= self._debug_max:
            return

        try:
            tri = self._head_triangle()
            planned_proj_raw = self._planned_path_projected_points()
            arrow_h = self._heading_deg_from_points(self._points)
            planned_h_raw = self._heading_deg_from_points(planned_proj_raw)

            planned_proj = planned_proj_raw
            if (
                len(planned_proj_raw) >= 2
                and len(self._points) >= 2
                and arrow_h is not None
                and planned_h_raw is not None
            ):
                planned_proj = self._align_polyline_to_heading(
                    planned_proj_raw,
                    planned_h_raw,
                    arrow_h,
                    self._points[0],
                )
            self._capture_run_planner(now, planned_proj)

            planned_h = self._heading_deg_from_points(planned_proj)
            planned_err = None
            if arrow_h is not None and planned_h is not None:
                planned_err = wrap_deg(arrow_h - planned_h)
            stem = f"{int(now * 1000)}_{self._debug_count:04d}"
            json_path = self._debug_dir / f"{stem}.json"
            svg_path = self._debug_dir / f"{stem}.svg"

            snapshot = {
                "time": now,
                "target_angle_deg": self._target_angle,
                "draw_angle_deg": self._draw_angle,
                "raw_last_angle_deg": self._last_raw_angle,
                "reverse_pending": self._reverse_pending,
                "intent_mode": self._intent_mode,
                "intent_scale_px_m": self._intent_scale_px_m,
                "reset_count": self._reset_count,
                "path_len_px": self._path_len_px,
                "path_points": [{"x": p.x(), "y": p.y()} for p in self._points],
                "arrow_trajectory_screen": [{"x": p.x(), "y": p.y()} for p in self._points],
                "head": tri,
                "planned_path_world": [{"x": x, "y": y} for x, y in self._planned_path_world],
                "planned_path_screen_raw": [{"x": p.x(), "y": p.y()} for p in planned_proj_raw],
                "planned_path_screen": [{"x": p.x(), "y": p.y()} for p in planned_proj],
                "planned_path_stamp": self._planned_path_stamp,
                "comparison": {
                    "arrow_heading_deg": arrow_h,
                    "planned_heading_raw_deg": planned_h_raw,
                    "planned_heading_deg": planned_h,
                    "planned_heading_error_deg": planned_err,
                },
            }
            json_path.write_text(json.dumps(snapshot, indent=2), encoding="utf-8")

            w = max(1, self.width())
            h = max(1, self.height())
            parts = [
                f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}">',
                '<rect x="0" y="0" width="100%" height="100%" fill="black"/>',
            ]

            if len(self._points) >= 2:
                draw_pts = list(self._points)
                if tri is not None:
                    bx, by = tri["base"]
                    draw_pts[-1] = QtCore.QPointF(bx, by)
                pts_str = " ".join(f"{pp.x():.2f},{pp.y():.2f}" for pp in draw_pts)
                parts.append(
                    f'<polyline points="{pts_str}" fill="none" stroke="white" stroke-width="{self._line_width * 1.8:.2f}" stroke-linecap="round" stroke-linejoin="round" opacity="0.35"/>'
                )
                parts.append(
                    f'<polyline points="{pts_str}" fill="none" stroke="white" stroke-width="{self._line_width:.2f}" stroke-linecap="round" stroke-linejoin="round"/>'
                )

            if len(planned_proj) >= 2:
                ppts = " ".join(f"{pp.x():.2f},{pp.y():.2f}" for pp in planned_proj)
                parts.append(
                    '<polyline points="{}" fill="none" stroke="#ffcc00" stroke-width="3" '
                    'stroke-linecap="round" stroke-linejoin="round" opacity="0.75"/>'.format(ppts)
                )

            if tri is not None:
                t = tri["tip"]
                l = tri["left"]
                r = tri["right"]
                head_w = max(2.0, self._line_width * 0.75)
                parts.append(
                    '<line x1="{:.2f}" y1="{:.2f}" x2="{:.2f}" y2="{:.2f}" '
                    'stroke="white" stroke-width="{:.2f}" stroke-linecap="round" opacity="0.35"/>'.format(
                        t[0], t[1], l[0], l[1], head_w * 1.65
                    )
                )
                parts.append(
                    '<line x1="{:.2f}" y1="{:.2f}" x2="{:.2f}" y2="{:.2f}" '
                    'stroke="white" stroke-width="{:.2f}" stroke-linecap="round"/>'.format(
                        t[0], t[1], l[0], l[1], head_w
                    )
                )
                parts.append(
                    '<line x1="{:.2f}" y1="{:.2f}" x2="{:.2f}" y2="{:.2f}" '
                    'stroke="white" stroke-width="{:.2f}" stroke-linecap="round" opacity="0.35"/>'.format(
                        t[0], t[1], r[0], r[1], head_w * 1.65
                    )
                )
                parts.append(
                    '<line x1="{:.2f}" y1="{:.2f}" x2="{:.2f}" y2="{:.2f}" '
                    'stroke="white" stroke-width="{:.2f}" stroke-linecap="round"/>'.format(
                        t[0], t[1], r[0], r[1], head_w
                    )
                )
            parts.append('</svg>')
            svg_path.write_text("\n".join(parts), encoding="utf-8")
            self._debug_count += 1
        except Exception:
            pass

    def _tick(self):
        now = self._clock()
        dt = max(0.001, min(0.05, now - self._last_tick))
        self._last_tick = now
        self._intent_mode = "servo"

        # Interpret servo as steering command (turn rate), not absolute path heading.
        # This produces curved trajectories for sustained turns and avoids false straight lines.
        blend = 1.0 - math.exp(-self._response_hz * dt)
        steer_delta = wrap_deg(self._target_angle - self._draw_steer_cmd_deg)
        self._draw_steer_cmd_deg = wrap_deg(self._draw_steer_cmd_deg + blend * steer_delta)

        center_snap_active = (
            self._center_snap_count >= self._center_snap_samples
            and abs(self._target_angle) <= self._center_target_limit_deg
        )
        if center_snap_active:
            z_blend = 1.0 - math.exp(-self._center_snap_hz * dt)
            self._target_angle = wrap_deg(self._target_angle * (1.0 - z_blend))
            self._draw_steer_cmd_deg = wrap_deg(self._draw_steer_cmd_deg * (1.0 - z_blend))

        turn_rate_raw_deg_s = self._draw_steer_cmd_deg * self._servo_to_turn_rate_gain
        turn_rate_raw_deg_s = max(-self._max_turn_rate_deg_s, min(self._max_turn_rate_deg_s, turn_rate_raw_deg_s))

        # Limit command acceleration so frame-to-frame flips do not produce "snake" previews.
        sign_change = (turn_rate_raw_deg_s * self._turn_rate_cmd_deg_s) < 0.0
        if sign_change or abs(turn_rate_raw_deg_s) > abs(self._turn_rate_cmd_deg_s):
            slew = self._turn_rate_slew_up_deg_s2
        else:
            slew = self._turn_rate_slew_down_deg_s2
        max_step = max(1e-6, slew * dt)
        step = turn_rate_raw_deg_s - self._turn_rate_cmd_deg_s
        if abs(step) > max_step:
            step = math.copysign(max_step, step)
        self._turn_rate_cmd_deg_s = self._turn_rate_cmd_deg_s + step

        # Fast attack for quick turn start, slower release to avoid early straightening in U-turns.
        if abs(self._turn_rate_cmd_deg_s) >= abs(self._turn_rate_smooth_deg_s):
            filt_hz = self._turn_rate_filter_attack_hz
        else:
            filt_hz = self._turn_rate_filter_release_hz
        tr_blend = 1.0 - math.exp(-filt_hz * dt)
        self._turn_rate_smooth_deg_s = self._turn_rate_smooth_deg_s + tr_blend * (
            self._turn_rate_cmd_deg_s - self._turn_rate_smooth_deg_s
        )

        # Fast handover on clear turn-direction reversal to reduce visible lag.
        if (
            self._turn_rate_cmd_deg_s * self._turn_rate_smooth_deg_s < 0.0
            and abs(self._turn_rate_cmd_deg_s) >= 24.0
        ):
            self._turn_rate_smooth_deg_s = self._turn_rate_cmd_deg_s

        if center_snap_active:
            z_blend = 1.0 - math.exp(-self._center_snap_hz * dt)
            self._turn_rate_cmd_deg_s = self._turn_rate_cmd_deg_s * (1.0 - z_blend)
            self._turn_rate_smooth_deg_s = self._turn_rate_smooth_deg_s * (1.0 - z_blend)

        if abs(self._turn_rate_smooth_deg_s) < self._turn_rate_deadband_deg_s:
            self._turn_rate_smooth_deg_s = 0.0

        self._points, self._draw_angle = self._build_servo_preview_path(self._turn_rate_smooth_deg_s)

        self._path_len_px = self._polyline_length(self._points)
        self._append_trace_point(self._run_white_tip_trace, self._points[-1], min_dist_px=0.8)

        self._debug_dump(now)
        self.update()

    def paintEvent(self, _event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)
        p.fillRect(self.rect(), QtGui.QColor(0, 0, 0))

        if len(self._points) < 2:
            return

        tri = self._head_triangle()
        if tri is None:
            return

        draw_pts = list(self._points)
        bx, by = tri["base"]
        draw_pts[-1] = QtCore.QPointF(bx, by)

        path = QtGui.QPainterPath()
        path.moveTo(draw_pts[0])
        for pt in draw_pts[1:]:
            path.lineTo(pt)

        p.setPen(
            QtGui.QPen(
                QtGui.QColor(255, 255, 255, 95),
                self._line_width * 1.8,
                QtCore.Qt.PenStyle.SolidLine,
                QtCore.Qt.PenCapStyle.RoundCap,
                QtCore.Qt.PenJoinStyle.RoundJoin,
            )
        )
        p.drawPath(path)

        p.setPen(
            QtGui.QPen(
                QtGui.QColor(255, 255, 255, 235),
                self._line_width,
                QtCore.Qt.PenStyle.SolidLine,
                QtCore.Qt.PenCapStyle.RoundCap,
                QtCore.Qt.PenJoinStyle.RoundJoin,
            )
        )
        p.drawPath(path)

        tip_pt = QtCore.QPointF(*tri["tip"])
        left_pt = QtCore.QPointF(*tri["left"])
        right_pt = QtCore.QPointF(*tri["right"])
        head_w = max(2.0, self._line_width * 0.75)

        p.setPen(
            QtGui.QPen(
                QtGui.QColor(255, 255, 255, 95),
                head_w * 1.65,
                QtCore.Qt.PenStyle.SolidLine,
                QtCore.Qt.PenCapStyle.RoundCap,
                QtCore.Qt.PenJoinStyle.RoundJoin,
            )
        )
        p.drawLine(tip_pt, left_pt)
        p.drawLine(tip_pt, right_pt)
        p.setPen(
            QtGui.QPen(
                QtGui.QColor(255, 255, 255, 235),
                head_w,
                QtCore.Qt.PenStyle.SolidLine,
                QtCore.Qt.PenCapStyle.RoundCap,
                QtCore.Qt.PenJoinStyle.RoundJoin,
            )
        )
        p.drawLine(tip_pt, left_pt)
        p.drawLine(tip_pt, right_pt)

    def closeEvent(self, event):
        self._write_run_summary()
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser(
        description="Fullscreen dynamic path projector arrow driven by servo topic"
    )
    parser.add_argument("--topic", default="/cabot/servo_target", help="Int16 topic for directional servo target")
    parser.add_argument("--path-topic", default="/plan", help="nav_msgs/Path topic for planned trajectory debug")
    parser.add_argument("--screen", type=int, default=1, help="Target screen index (projector is often 1)")
    parser.add_argument("--deadband", type=float, default=5.0, help="Ignore small angles around zero")
    parser.add_argument("--smoothing", type=float, default=0.65, help="Direction smoothness (<=1 legacy, >1 response Hz)")
    parser.add_argument("--fps", type=int, default=60, help="Render timer target FPS")

    parser.add_argument("--speed", type=float, default=280.0, help="Path growth speed in pixels/second")
    parser.add_argument("--trail-sec", type=float, default=2.2, help="Approximate path max duration")

    parser.add_argument("--line-width", type=float, default=22.0, help="Body line width")
    parser.add_argument("--head-length", type=float, default=None, help="Arrow head length (auto if omitted)")
    parser.add_argument("--head-width", type=float, default=None, help="Arrow head half-width (auto if omitted)")
    parser.add_argument("--arena-scale", type=float, default=0.55, help="Path length scale in screen space")

    parser.add_argument("--spike-delta", type=float, default=110.0, help="Drop raw angle jumps larger than this deg")
    parser.add_argument("--reverse-trigger", type=float, default=120.0, help="Treat angles above this as reverse candidates")
    parser.add_argument("--reverse-confirm-samples", type=int, default=10, help="Samples required before accepting reverse candidate")

    parser.add_argument("--debug-dump-dir", default="", help="Write debug JSON/SVG snapshots to this directory")
    parser.add_argument("--debug-dump-every", type=int, default=8, help="Dump one debug sample every N ticks")
    parser.add_argument("--debug-max-dumps", type=int, default=120, help="Maximum number of debug samples")
    parser.add_argument("--debug", "--debugf", dest="debug", action="store_true", help="Enable debug dump to default projector_live/tmp/projector_debug")
    parser.add_argument("--invert", action="store_true", help="Invert sign if left/right is mirrored")
    args = parser.parse_args()

    if args.debug and not args.debug_dump_dir:
        args.debug_dump_dir = default_debug_dir()
        args.debug_dump_every = min(args.debug_dump_every, 4)
        if args.debug_max_dumps > 0:
            args.debug_max_dumps = max(args.debug_max_dumps, 200)

    head_length = args.head_length if args.head_length is not None else max(56.0, args.line_width * 3.2)
    head_width = args.head_width if args.head_width is not None else max(28.0, args.line_width * 1.6)

    app = QtWidgets.QApplication([])
    win = DynamicPathProjectorWindow(
        screen_index=args.screen,
        deadband_deg=args.deadband,
        smoothing=args.smoothing,
        fps=args.fps,
        speed_px=args.speed,
        trail_sec=args.trail_sec,
        line_width=args.line_width,
        head_length=head_length,
        head_width=head_width,
        arena_scale=args.arena_scale,
        spike_delta_deg=args.spike_delta,
        reverse_trigger_deg=args.reverse_trigger,
        reverse_confirm_samples=args.reverse_confirm_samples,
        debug_dump_dir=args.debug_dump_dir,
        debug_dump_every=args.debug_dump_every,
        debug_max_dumps=args.debug_max_dumps,
        invert_sign=args.invert,
    )

    bridge = RosBridge(topic=args.topic, invert_sign=args.invert, path_topic=args.path_topic)
    bridge.angle_changed.connect(win.on_servo_angle)
    bridge.planned_path_changed.connect(win.on_planned_path)
    bridge.status.connect(lambda text: print(f"[projector_arrow_live] {text}"))
    bridge.start()

    def _cleanup():
        bridge.stop()
        win._write_run_summary()
        time.sleep(0.1)

    app.aboutToQuit.connect(_cleanup)
    app.exec()


if __name__ == "__main__":
    main()
