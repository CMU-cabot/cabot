#!/usr/bin/env python3
import argparse
import json
import math
import subprocess
import sys
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
    intent_motion_changed = QtCore.pyqtSignal(float, float, float)
    actual_motion_changed = QtCore.pyqtSignal(float, float, float)
    obstacle_scan_changed = QtCore.pyqtSignal(float, float, float)
    human_presence_changed = QtCore.pyqtSignal(bool, float, float)
    stop_reason_changed = QtCore.pyqtSignal(str, float, bool, float)
    touch_changed = QtCore.pyqtSignal(int, float)
    status = QtCore.pyqtSignal(str)
    blocked_state_changed = QtCore.pyqtSignal(bool)

    def __init__(
        self,
        topic: str,
        path_topic: str,
        motion_topic: str,
        actual_motion_topic: str,
        scan_topic: str,
        human_topic: str,
        people_target_frame: str,
        people_front_max_dist: float,
        people_front_half_angle_deg: float,
        touch_topic: str,
        stop_reason_topic: str,
        blocked_topic: str,
        haptic_topic: str,
        obstacle_front_half_angle_deg: float,
        obstacle_dist_m: float,
    ):
        super().__init__()
        self.topic = topic
        self.path_topic = path_topic
        self.motion_topic = motion_topic
        self.actual_motion_topic = actual_motion_topic
        self.scan_topic = scan_topic
        self.human_topic = human_topic
        self.people_target_frame = (people_target_frame or "base_footprint").strip() or "base_footprint"
        self.people_front_max_dist = max(0.05, float(people_front_max_dist))
        self.people_front_half_angle_deg = max(1.0, min(179.0, float(people_front_half_angle_deg)))
        self.touch_topic = touch_topic
        self.stop_reason_topic = stop_reason_topic
        self.blocked_topic = blocked_topic
        self.haptic_topic = haptic_topic
        self.obstacle_front_half_angle_deg = max(1.0, float(obstacle_front_half_angle_deg))
        self.obstacle_dist_m = max(0.05, float(obstacle_dist_m))
        self._stop_event = threading.Event()
        self._thread = None
        self._blocked_lock = threading.Lock()
        self._blocked_state_value = False
        self._haptic_lock = threading.Lock()
        self._haptic_pending_value = None

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

    @QtCore.pyqtSlot(bool)
    def on_blocked_state_changed(self, state: bool):
        with self._blocked_lock:
            self._blocked_state_value = bool(state)

    @QtCore.pyqtSlot(int)
    def on_haptic_triggered(self, value: int):
        v = int(value)
        if v < 0:
            v = 0
        if v > 255:
            v = 255
        with self._haptic_lock:
            self._haptic_pending_value = v

    def _run(self):
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from rclpy.time import Time
            from rclpy.duration import Duration
            from geometry_msgs.msg import Twist
            from nav_msgs.msg import Odometry
            from std_msgs.msg import Bool, Int16, UInt8
        except Exception as ex:
            self.status.emit(f"rclpy/ros_msgs import failed: {ex}")
            return

        has_stop_reason = False
        StopReason = None  # type: ignore
        try:
            from cabot_msgs.msg import StopReason  # type: ignore
            has_stop_reason = True
        except Exception as ex:
            self.status.emit(f"cabot_msgs/StopReason not available: {ex}")

        has_nav = False
        try:
            from nav_msgs.msg import Path  # type: ignore
            has_nav = True
        except Exception as ex:
            self.status.emit(f"nav_msgs not available: {ex}")
            Path = None  # type: ignore

        has_scan = False
        try:
            from sensor_msgs.msg import LaserScan  # type: ignore
            has_scan = True
        except Exception as ex:
            self.status.emit(f"sensor_msgs/LaserScan not available: {ex}")
            LaserScan = None  # type: ignore

        has_people = False
        People = None  # type: ignore
        tf2_ros = None  # type: ignore
        try:
            from people_msgs.msg import People  # type: ignore
            has_people = True
        except Exception as ex:
            self.status.emit(f"people_msgs/People not available: {ex}")
        try:
            import tf2_ros  # type: ignore
        except Exception as ex:
            self.status.emit(f"tf2_ros not available for /people transform: {ex}")

        class BridgeNode(Node):
            def __init__(self, bridge: RosBridge):
                super().__init__("projector_arrow_live")
                self._bridge = bridge
                self.create_subscription(Int16, bridge.topic, self._cb_servo, 10)

                if has_nav and bridge.path_topic:
                    self.create_subscription(Path, bridge.path_topic, self._cb_path, 10)  # type: ignore
                if bridge.motion_topic:
                    self.create_subscription(Twist, bridge.motion_topic, self._cb_intent_motion, 20)
                if bridge.actual_motion_topic:
                    self.create_subscription(Odometry, bridge.actual_motion_topic, self._cb_actual_motion, 20)
                if has_scan and bridge.scan_topic:
                    # Match typical LaserScan publishers (BEST_EFFORT) to avoid QoS incompatibility.
                    self.create_subscription(LaserScan, bridge.scan_topic, self._cb_scan, qos_profile_sensor_data)  # type: ignore
                self._people_tf_buffer = None
                self._people_tf_listener = None
                if bridge.human_topic:
                    if has_people and People is not None:
                        self.create_subscription(People, bridge.human_topic, self._cb_human_people, 20)
                        if tf2_ros is not None:
                            self._people_tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=3.0))
                            self._people_tf_listener = tf2_ros.TransformListener(self._people_tf_buffer, self, spin_thread=False)
                    else:
                        bridge.status.emit("/people requested but people_msgs is unavailable")
                if bridge.touch_topic:
                    self.create_subscription(Int16, bridge.touch_topic, self._cb_touch, 20)
                if bridge.stop_reason_topic and has_stop_reason and StopReason is not None:
                    self.create_subscription(StopReason, bridge.stop_reason_topic, self._cb_stop_reason, 20)
                elif bridge.stop_reason_topic:
                    bridge.status.emit("stop_reason disabled (cabot_msgs not available)")

                self._blocked_pub = None
                if bridge.blocked_topic:
                    self._blocked_pub = self.create_publisher(Bool, bridge.blocked_topic, 10)
                    self.create_timer(0.10, self._pub_blocked_state)

                self._haptic_pub = None
                if bridge.haptic_topic:
                    self._haptic_pub = self.create_publisher(UInt8, bridge.haptic_topic, 10)
                    self.create_timer(0.05, self._pub_haptic)

            def _pub_blocked_state(self):
                try:
                    if self._blocked_pub is None:
                        return
                    with self._bridge._blocked_lock:
                        blocked = bool(self._bridge._blocked_state_value)
                    msg = Bool()
                    msg.data = blocked
                    self._blocked_pub.publish(msg)
                except Exception:
                    pass

            def _pub_haptic(self):
                try:
                    if self._haptic_pub is None:
                        return
                    with self._bridge._haptic_lock:
                        val = self._bridge._haptic_pending_value
                        self._bridge._haptic_pending_value = None
                    if val is None:
                        return
                    msg = UInt8()
                    msg.data = int(val)
                    self._haptic_pub.publish(msg)
                except Exception:
                    pass

            def _cb_servo(self, msg):
                value = float(msg.data)
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

            def _cb_intent_motion(self, msg):
                try:
                    lin = float(msg.linear.x)
                    ang = float(msg.angular.z)
                    self._bridge.intent_motion_changed.emit(lin, ang, time.time())
                except Exception:
                    pass

            def _cb_actual_motion(self, msg):
                try:
                    tw = msg.twist.twist
                    lin = float(tw.linear.x)
                    ang = float(tw.angular.z)
                    self._bridge.actual_motion_changed.emit(lin, ang, time.time())
                except Exception:
                    pass

            def _cb_scan(self, msg):
                try:
                    half_rad = math.radians(float(self._bridge.obstacle_front_half_angle_deg))
                    close_dist = float(self._bridge.obstacle_dist_m)
                    a = float(msg.angle_min)
                    da = float(msg.angle_increment)
                    rmin = float(msg.range_min)
                    rmax = float(msg.range_max)

                    valid = 0
                    close = 0
                    min_front = float("inf")
                    for rr in msg.ranges:
                        r = float(rr)
                        if abs(a) <= half_rad and math.isfinite(r):
                            if r >= rmin and (rmax <= 0.0 or r <= rmax):
                                valid += 1
                                if r < min_front:
                                    min_front = r
                                if r <= close_dist:
                                    close += 1
                        a += da

                    if valid <= 0:
                        close_fraction = 0.0
                        min_front = float("inf")
                    else:
                        close_fraction = float(close) / float(valid)
                    self._bridge.obstacle_scan_changed.emit(float(min_front), float(close_fraction), time.time())
                except Exception:
                    pass

            def _transform_people_xy(self, x: float, y: float, z: float, source_frame: str, sec: int, nsec: int):
                if self._people_tf_buffer is None:
                    return None
                try:
                    tf_msg = self._people_tf_buffer.lookup_transform(
                        self._bridge.people_target_frame,
                        source_frame,
                        Time(seconds=int(sec), nanoseconds=int(nsec)),
                        timeout=Duration(seconds=0.05),
                    )
                except Exception:
                    try:
                        tf_msg = self._people_tf_buffer.lookup_transform(
                            self._bridge.people_target_frame,
                            source_frame,
                            Time(),
                            timeout=Duration(seconds=0.05),
                        )
                    except Exception:
                        return None

                t = tf_msg.transform.translation
                q = tf_msg.transform.rotation
                qx = float(q.x)
                qy = float(q.y)
                qz = float(q.z)
                qw = float(q.w)

                xx = qx * qx
                yy = qy * qy
                zz = qz * qz
                xy = qx * qy
                xz = qx * qz
                yz = qy * qz
                wx = qw * qx
                wy = qw * qy
                wz = qw * qz

                m00 = 1.0 - 2.0 * (yy + zz)
                m01 = 2.0 * (xy - wz)
                m02 = 2.0 * (xz + wy)
                m10 = 2.0 * (xy + wz)
                m11 = 1.0 - 2.0 * (xx + zz)
                m12 = 2.0 * (yz - wx)

                rx = m00 * x + m01 * y + m02 * z
                ry = m10 * x + m11 * y + m12 * z
                return float(rx + t.x), float(ry + t.y)

            def _cb_human_people(self, msg):
                try:
                    people = list(getattr(msg, "people", []))
                    present = False
                    conf = 0.0

                    header = getattr(msg, "header", None)
                    frame = ""
                    sec = 0
                    nsec = 0
                    if header is not None:
                        frame = str(getattr(header, "frame_id", "") or "").strip()
                        st = getattr(header, "stamp", None)
                        if st is not None:
                            sec = int(getattr(st, "sec", 0))
                            nsec = int(getattr(st, "nanosec", 0))

                    half_rad = math.radians(float(self._bridge.people_front_half_angle_deg))
                    max_dist = float(self._bridge.people_front_max_dist)
                    best_dist = None

                    for person in people:
                        pos = getattr(person, "position", None)
                        if pos is None:
                            continue
                        x = float(getattr(pos, "x", 0.0))
                        y = float(getattr(pos, "y", 0.0))
                        z = float(getattr(pos, "z", 0.0))

                        if frame and frame != self._bridge.people_target_frame:
                            xy = self._transform_people_xy(x, y, z, frame, sec, nsec)
                            if xy is None:
                                continue
                            x, y = xy

                        if x <= 0.0:
                            continue
                        dist = math.hypot(x, y)
                        if dist > max_dist:
                            continue
                        if abs(math.atan2(y, x)) > half_rad:
                            continue

                        present = True
                        if best_dist is None or dist < best_dist:
                            best_dist = dist

                    if present and best_dist is not None and max_dist > 1e-6:
                        conf = max(0.05, min(1.0, 1.0 - (best_dist / max_dist)))

                    self._bridge.human_presence_changed.emit(bool(present), float(conf), time.time())
                except Exception:
                    pass

            def _cb_touch(self, msg):
                try:
                    self._bridge.touch_changed.emit(int(msg.data), time.time())
                except Exception:
                    pass

            def _cb_stop_reason(self, msg):
                try:
                    reason = str(getattr(msg, "reason", "") or "")
                    duration = float(getattr(msg, "duration", 0.0) or 0.0)
                    summary = bool(getattr(msg, "summary", False))
                    self._bridge.stop_reason_changed.emit(reason, duration, summary, time.time())
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
                f"path={self.path_topic if has_nav else 'disabled'} "
                f"cmd_vel={self.motion_topic or 'disabled'} "
                f"odom={self.actual_motion_topic or 'disabled'} "
                f"scan={self.scan_topic if has_scan else 'disabled'} "
                f"human=people:{self.human_topic or 'disabled'} "
                f"stop_reason={(self.stop_reason_topic if has_stop_reason else 'disabled')} "
                f"touch={self.touch_topic or 'disabled'} "
                f"blocked_pub={self.blocked_topic or 'disabled'} "
                f"haptic_pub={self.haptic_topic or 'disabled'}"
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
    blocked_state_changed = QtCore.pyqtSignal(bool)
    haptic_triggered = QtCore.pyqtSignal(int)
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
        debug_dump_dir: str,
        debug_dump_every: int,
        debug_max_dumps: int,
        blocked_demand_linear_threshold: float,
        blocked_demand_angular_threshold: float,
        blocked_moving_linear_threshold: float,
        blocked_moving_angular_threshold: float,
        blocked_clear_linear_threshold: float,
        blocked_clear_angular_threshold: float,
        blocked_require_front_obstacle: bool,
        blocked_obstacle_front_max_dist: float,
        blocked_obstacle_min_fraction: float,
        blocked_obstacle_scan_timeout: float,
        require_human_for_negotiation: bool,
        human_hold_sec: float,
        human_fresh_sec: float,
        human_active_min_for_neg_sec: float,
        neg_enter_hold_sec: float,
        neg_exit_hold_sec: float,
        neg_person_entry_window_sec: float,
        stop_reason_fresh_sec: float,
        blocked_enter_hold_sec: float,
        blocked_exit_hold_sec: float,
        blocked_demand_hold_sec: float,
        blocked_confirm_min_sec: float,
        level1_sound_path: str,
        level1_sound_cooldown_sec: float,
        level1_sound_min_play_sec: float,
        require_touch_for_sound: bool,
        touch_threshold: int,
        touch_hold_sec: float,
        haptic_value: int,
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
        self._head_length = max(12.0, head_length) * self._artifact_scale * 1.45
        self._head_width = max(8.0, head_width) * self._artifact_scale * 1.5
        self._size_scale = min(1.0, max(0.2, arena_scale))

        self._max_turn_rate_deg_s = 280.0
        self._angle_jitter_deg = 0.6
        self._servo_target_blend = 0.80
        self._servo_to_turn_rate_gain = 1.45
        self._draw_steer_cmd_deg = 0.0
        self._turn_rate_slew_up_deg_s2 = 5200.0
        self._turn_rate_slew_down_deg_s2 = 1300.0
        self._turn_rate_filter_attack_hz = 12.0
        self._turn_rate_filter_release_hz = 11.0
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
        self._anim_phase_px = 0.0
        # Keep motion readable over remote desktop (too fast can look static on VNC).
        self._anim_speed_px_s = self._path_speed * 0.45
        self._anim_head_phase = 0.0
        self._anim_head_hz = 1.8

        self._debug_dir = Path(debug_dump_dir).expanduser() if debug_dump_dir else None
        self._debug_every = max(1, debug_dump_every)
        self._debug_max = None if debug_max_dumps <= 0 else max(1, debug_max_dumps)
        self._debug_frame = 0
        self._debug_count = 0
        self._run_started_wall = time.time()
        self._run_summary_written = False
        self._run_white_tip_trace = []  # [(x, y), ...]
        self._run_yellow_tip_trace = []  # [(x, y), ...]
        self._run_yellow_paths = []  # [{"time": t, "points": [(x, y), ...]}, ...]
        self._run_last_yellow_sig = None

        # Blocked detection (for logging/metrics only, no visual behavior change).
        self._intent_linear_cmd = 0.0
        self._intent_angular_cmd = 0.0
        self._actual_linear = 0.0
        self._actual_angular = 0.0
        self._last_actual_motion_wall = 0.0

        self._blocked_demand_linear_threshold = max(0.0, blocked_demand_linear_threshold)
        self._blocked_demand_angular_threshold = max(0.0, blocked_demand_angular_threshold)
        self._blocked_moving_linear_threshold = max(0.0, blocked_moving_linear_threshold)
        self._blocked_moving_angular_threshold = max(0.0, blocked_moving_angular_threshold)
        self._blocked_clear_linear_threshold = max(0.0, blocked_clear_linear_threshold)
        self._blocked_clear_angular_threshold = max(0.0, blocked_clear_angular_threshold)
        self._blocked_require_front_obstacle = bool(blocked_require_front_obstacle)
        self._blocked_obstacle_front_max_dist = max(0.05, blocked_obstacle_front_max_dist)
        self._blocked_obstacle_min_fraction = min(1.0, max(0.0, blocked_obstacle_min_fraction))
        self._blocked_obstacle_scan_timeout = max(0.05, blocked_obstacle_scan_timeout)
        self._require_human_for_negotiation = bool(require_human_for_negotiation)
        self._human_hold_sec = max(0.0, human_hold_sec)
        self._human_fresh_sec = max(0.05, float(human_fresh_sec))
        self._human_active_min_for_neg_sec = max(0.0, float(human_active_min_for_neg_sec))
        self._neg_enter_hold_sec = max(0.0, float(neg_enter_hold_sec))
        self._neg_exit_hold_sec = max(0.0, float(neg_exit_hold_sec))
        self._neg_person_entry_window_sec = max(0.0, float(neg_person_entry_window_sec))
        self._stop_reason_fresh_sec = max(0.05, float(stop_reason_fresh_sec))
        self._stop_reason_people_codes = {
            "THERE_ARE_PEOPLE_IN_THE_PATH",
        }
        self._blocked_enter_hold_sec = max(0.0, blocked_enter_hold_sec)
        self._blocked_exit_hold_sec = max(0.0, blocked_exit_hold_sec)
        self._blocked_demand_hold_sec = max(0.0, blocked_demand_hold_sec)
        self._blocked_confirm_min_sec = max(0.0, blocked_confirm_min_sec)

        self._level1_sound_path = str(level1_sound_path or "").strip()
        self._level1_sound_cooldown_sec = max(0.0, float(level1_sound_cooldown_sec))
        self._last_level1_sound_wall = 0.0
        self._level1_sound_min_play_sec = max(0.0, float(level1_sound_min_play_sec))
        self._level1_started_wall = 0.0
        self._level1_playing = False
        self._level1_proc = None
        self._level1_lock = threading.Lock()
        self._level1_attempt_count = 0
        self._level1_success_count = 0
        self._level1_last_status = "init"
        self._require_touch_for_sound = bool(require_touch_for_sound)
        self._touch_threshold = max(0, int(touch_threshold))
        self._touch_hold_sec = max(0.0, float(touch_hold_sec))
        self._touch_fresh_sec = max(0.20, self._touch_hold_sec + 0.60)
        self._touch_value = 0
        self._touch_last_wall = 0.0
        self._touch_active_until_wall = 0.0
        self._touch_raw_active = False
        self._last_sound_skip_log_wall = 0.0
        self._haptic_value = max(0, min(255, int(haptic_value)))

        self._blocked_state = False
        self._blocked_candidate_since = None
        self._blocked_clear_since = None
        self._last_demand_mono = 0.0
        self._last_forward_demand_mono = 0.0
        self._blocked_event_count = 0
        self._active_blocked_event_id = None
        self._blocked_total_sec = 0.0
        self._blocked_active_since_mono = None
        self._blocked_events = []
        self._scan_min_front_m = float("inf")
        self._scan_close_fraction = 0.0
        self._scan_last_wall = 0.0
        self._human_present_raw = False
        self._human_conf = 0.0
        self._human_last_wall = 0.0
        self._human_active_until_wall = 0.0
        self._human_prev_present_raw = False
        self._human_state_since_wall = 0.0
        self._stop_reason_value = ""
        self._stop_reason_last_wall = 0.0
        self._stop_reason_duration = 0.0
        self._stop_reason_summary = False
        self._person_event_count = 0
        self._last_person_event_id = None
        self._last_person_event_wall = 0.0
        self._person_link_max_age_sec = 1.0
        self._sound_event_count = 0
        self._sound_active_event_id = None
        self._recent_sound_events = []  # [(sound_id, wall_time), ...]
        self._pre_block_link_window_sec = 1.2
        self._blocked_debug = {
            "decision": "init",
            "candidate": False,
            "demand_active": False,
            "actual_moving": False,
        }
        self._blocked_last_emitted = None

        self._neg_block_state = False
        self._neg_block_event_count = 0
        self._active_neg_block_event_id = None
        self._neg_block_total_sec = 0.0
        self._neg_block_active_since_mono = None
        self._neg_candidate_since = None
        self._neg_clear_since = None
        self._neg_person_seen_for_block = False
        self._neg_person_seen_block_id = None
        self._neg_block_events = []
        self._neg_block_debug = {"decision": "init", "human_active": False, "fused_active": False}
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

    @QtCore.pyqtSlot(float, float, float)
    def on_intent_motion(self, linear_x: float, angular_z: float, stamp: float):
        self._intent_linear_cmd = float(linear_x)
        self._intent_angular_cmd = float(angular_z)

    @QtCore.pyqtSlot(float, float, float)
    def on_actual_motion(self, linear_x: float, angular_z: float, stamp: float):
        self._actual_linear = float(linear_x)
        self._actual_angular = float(angular_z)
        self._last_actual_motion_wall = float(stamp)

    @QtCore.pyqtSlot(float, float, float)
    def on_obstacle_scan(self, min_front_m: float, close_fraction: float, stamp: float):
        self._scan_min_front_m = float(min_front_m)
        self._scan_close_fraction = float(close_fraction)
        self._scan_last_wall = float(stamp)

    @QtCore.pyqtSlot(bool, float, float)
    def on_human_presence(self, present: bool, confidence: float, stamp: float):
        prev_present = bool(self._human_prev_present_raw)
        self._human_present_raw = bool(present)
        self._human_conf = float(confidence)
        self._human_last_wall = float(stamp)
        if self._human_present_raw:
            self._human_active_until_wall = max(self._human_active_until_wall, float(stamp) + self._human_hold_sec)
            if not prev_present or self._human_state_since_wall <= 0.0:
                self._human_state_since_wall = float(stamp)
        else:
            # Do not immediately hard-reset; let hold smooth short detector drops.
            self._human_active_until_wall = max(self._human_active_until_wall, float(stamp))
            self._human_state_since_wall = 0.0
        # Trigger-only logging: print once when person presence rises.
        if self._human_present_raw and not prev_present:
            self._person_event_count += 1
            person_id = f"P{self._person_event_count:04d}"
            self._last_person_event_id = person_id
            self._last_person_event_wall = float(stamp)
            self._record_block_relation(person_id=person_id)
            self._log_event(
                "person_detected",
                person_id=person_id,
                conf=f"{self._human_conf:.2f}",
                blocked_id=(self._active_blocked_event_id or "-"),
                neg_block_id=(self._active_neg_block_event_id or "-"),
                last_person_id=(self._last_person_event_id or "-"),
            )
        self._human_prev_present_raw = self._human_present_raw

    @QtCore.pyqtSlot(str, float, bool, float)
    def on_stop_reason(self, reason: str, duration: float, summary: bool, stamp: float):
        self._stop_reason_value = str(reason or "").strip().upper()
        self._stop_reason_duration = max(0.0, float(duration))
        self._stop_reason_summary = bool(summary)
        self._stop_reason_last_wall = float(stamp)

    @QtCore.pyqtSlot(int, float)
    def on_touch_value(self, value: int, stamp: float):
        v = int(value)
        self._touch_value = v
        self._touch_last_wall = float(stamp)
        raw_active = bool(v >= self._touch_threshold)
        self._touch_raw_active = raw_active

    def _touch_is_active(self, now_wall: float) -> bool:
        _ = now_wall
        return bool(self._touch_raw_active)

    def _log_event(self, event_type: str, **fields):
        return

    @staticmethod
    def _append_unique(items, value):
        if value and value not in items:
            items.append(value)

    def _active_block_record(self):
        if not self._active_blocked_event_id or not self._blocked_events:
            return None
        last = self._blocked_events[-1]
        if last.get("event_id") != self._active_blocked_event_id:
            return None
        if last.get("end_mono") is not None:
            return None
        return last

    def _record_block_relation(self, *, person_id=None, sound_id=None, neg_id=None):
        rec = self._active_block_record()
        if rec is None:
            return
        if person_id:
            self._append_unique(rec.setdefault("person_ids", []), person_id)
        if sound_id:
            self._append_unique(rec.setdefault("sound_ids", []), sound_id)
        if neg_id:
            self._append_unique(rec.setdefault("neg_ids", []), neg_id)

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
            sx = self._origin_x - left * scale
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
    def _polyline_slice_by_length(points, d_start: float, d_end: float):
        if len(points) < 2 or d_end <= d_start:
            return []

        d0 = max(0.0, float(d_start))
        d1 = max(0.0, float(d_end))
        if d1 <= d0:
            return []

        out = []
        traveled = 0.0
        for i in range(1, len(points)):
            p0 = points[i - 1]
            p1 = points[i]
            sx = p1.x() - p0.x()
            sy = p1.y() - p0.y()
            seg = math.hypot(sx, sy)
            if seg <= 1e-9:
                continue

            seg0 = traveled
            seg1 = traveled + seg

            if d1 < seg0:
                break
            if d0 > seg1:
                traveled = seg1
                continue

            t0 = max(0.0, min(1.0, (d0 - seg0) / seg))
            t1 = max(0.0, min(1.0, (d1 - seg0) / seg))
            if t1 <= t0:
                traveled = seg1
                continue

            q0 = QtCore.QPointF(p0.x() + sx * t0, p0.y() + sy * t0)
            q1 = QtCore.QPointF(p0.x() + sx * t1, p0.y() + sy * t1)
            if not out:
                out.append(q0)
            else:
                last = out[-1]
                if math.hypot(last.x() - q0.x(), last.y() - q0.y()) > 1e-6:
                    out.append(q0)
            out.append(q1)
            traveled = seg1

        return out

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

    def _play_level1_sound_worker(self, sound_file: Path, sound_event_id: str):
        cmds = [
            ["/usr/bin/paplay", str(sound_file)],
            ["paplay", str(sound_file)],
        ]
        ok = False
        last_err = "no_player"
        try:
            for cmd in cmds:
                try:
                    proc = subprocess.Popen(
                        cmd,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                    with self._level1_lock:
                        self._level1_proc = proc
                    try:
                        proc.wait(timeout=12)
                    except subprocess.TimeoutExpired:
                        proc.terminate()
                        try:
                            proc.wait(timeout=0.8)
                        except Exception:
                            proc.kill()
                            proc.wait(timeout=0.8)
                    rc = int(proc.returncode if proc.returncode is not None else -1)
                except Exception as ex:
                    last_err = f"{cmd[0]}:{type(ex).__name__}"
                    continue
                if rc == 0:
                    ok = True
                    self._level1_last_status = f"ok:{cmd[0]}"
                    break
                last_err = f"{cmd[0]}:rc{rc}"
            if ok:
                self._level1_success_count += 1
                self._log_event(
                    "sound_done",
                    sound_id=sound_event_id,
                    result="ok",
                    status=self._level1_last_status,
                    blocked_id=(self._active_blocked_event_id or "-"),
                    neg_block_id=(self._active_neg_block_event_id or "-"),
                )
            else:
                self._level1_last_status = f"fail:{last_err}"
                self._log_event(
                    "sound_done",
                    sound_id=sound_event_id,
                    result="fail",
                    status=self._level1_last_status,
                    blocked_id=(self._active_blocked_event_id or "-"),
                    neg_block_id=(self._active_neg_block_event_id or "-"),
                )
        finally:
            with self._level1_lock:
                self._level1_proc = None
            self._sound_active_event_id = None
            self._level1_playing = False

    def _play_level1_sound(self):
        if not self._level1_sound_path:
            self._level1_last_status = "skip:no_path"
            return
        now_wall = time.time()
        if self._require_touch_for_sound and (not self._touch_is_active(now_wall)):
            self._level1_last_status = "skip:no_touch"
            if (now_wall - self._last_sound_skip_log_wall) >= 1.0:
                self._last_sound_skip_log_wall = now_wall
                self._log_event(
                    "sound_skip",
                    reason="no_touch",
                    touch_value=self._touch_value,
                    touch_threshold=self._touch_threshold,
                    touch_active=0,
                    blocked_id=(self._active_blocked_event_id or "-"),
                    neg_block_id=(self._active_neg_block_event_id or "-"),
                )
            return

        rec = self._active_block_record()
        if rec is not None and len(rec.get("sound_ids") or []) > 0:
            self._level1_last_status = "skip:already_played_this_block"
            return

        if self._level1_sound_cooldown_sec > 0.0 and (now_wall - self._last_level1_sound_wall) < self._level1_sound_cooldown_sec:
            self._level1_last_status = "skip:cooldown"
            return
        if self._level1_playing:
            self._level1_last_status = "skip:playing"
            return
        sound_file = Path(self._level1_sound_path).expanduser()
        if not sound_file.is_absolute():
            sound_file = (Path(__file__).resolve().parent / sound_file).resolve()
        if not sound_file.exists():
            self._level1_last_status = "skip:missing_file"
            return

        self._last_level1_sound_wall = now_wall
        self._level1_attempt_count += 1
        self._sound_event_count += 1
        sound_event_id = f"S{self._sound_event_count:04d}"
        self._sound_active_event_id = sound_event_id
        self._level1_started_wall = now_wall
        self._level1_playing = True
        self._level1_last_status = "attempt"
        self._recent_sound_events.append((sound_event_id, now_wall))
        self._recent_sound_events = [
            e for e in self._recent_sound_events if (now_wall - float(e[1])) <= 6.0
        ]
        self._record_block_relation(sound_id=sound_event_id)
        self._log_event(
            "sound_trigger",
            sound_id=sound_event_id,
            blocked_id=(self._active_blocked_event_id or "-"),
            neg_block_id=(self._active_neg_block_event_id or "-"),
            file=sound_file.name,
        )
        threading.Thread(
            target=self._play_level1_sound_worker,
            args=(sound_file, sound_event_id),
            daemon=True,
        ).start()

    def _stop_level1_sound(self, force: bool = False):
        proc = None
        with self._level1_lock:
            proc = self._level1_proc
        if proc is None:
            self._level1_playing = False
            return

        now_wall = time.time()
        elapsed = max(0.0, now_wall - float(self._level1_started_wall))
        if (not force) and elapsed < self._level1_sound_min_play_sec:
            self._level1_last_status = "hold:min_play"
            return

        try:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=0.6)
                except Exception:
                    proc.kill()
                    proc.wait(timeout=0.6)
            self._level1_last_status = "stopped:on_unblock"
        except Exception as ex:
            self._level1_last_status = f"stop_fail:{type(ex).__name__}"
        finally:
            with self._level1_lock:
                self._level1_proc = None
            self._level1_playing = False

    def _trigger_haptic(self):
        self.haptic_triggered.emit(int(self._haptic_value))

    def _start_blocked_event(self, now_mono: float):
        self._blocked_state = True
        self._blocked_event_count += 1
        blocked_event_id = f"B{self._blocked_event_count:04d}"
        self._active_blocked_event_id = blocked_event_id
        self._neg_person_seen_for_block = False
        self._neg_person_seen_block_id = blocked_event_id
        now_wall = time.time()
        pre_sound_ids = [
            sid
            for sid, t in self._recent_sound_events
            if (now_wall - float(t)) <= self._pre_block_link_window_sec
        ]
        self._blocked_active_since_mono = float(now_mono)
        seed_person_ids = []
        if self._last_person_event_id and (now_wall - float(self._last_person_event_wall)) <= self._person_link_max_age_sec:
            seed_person_ids = [self._last_person_event_id]

        self._blocked_events.append(
            {
                "index": self._blocked_event_count,
                "event_id": blocked_event_id,
                "start_mono": float(now_mono),
                "start_wall": now_wall,
                "end_mono": None,
                "end_wall": None,
                "duration_sec": None,
                "person_ids": seed_person_ids,
                "sound_ids": [],
                "neg_ids": [],
                "pre_block_sound_ids": list(pre_sound_ids),
            }
        )
        self._log_event(
            "blocked_enter",
            blocked_id=blocked_event_id,
            last_person_id=(self._last_person_event_id or "-"),
            pre_sound_count=len(pre_sound_ids),
            pre_sound_ids=("|".join(pre_sound_ids) if pre_sound_ids else "-"),
            scan_min_front_m=f"{self._scan_min_front_m:.2f}",
            scan_close_fraction=f"{self._scan_close_fraction:.2f}",
        )
        self._trigger_haptic()

    def _end_blocked_event(self, now_mono: float):
        self._blocked_state = False
        active_id = self._active_blocked_event_id
        if self._blocked_active_since_mono is not None:
            dt = max(0.0, float(now_mono) - float(self._blocked_active_since_mono))
            self._blocked_total_sec += dt
            self._blocked_active_since_mono = None
        if self._blocked_events:
            last = self._blocked_events[-1]
            if last.get("end_mono") is None:
                last["end_mono"] = float(now_mono)
                last["end_wall"] = time.time()
                start_mono = float(last.get("start_mono", now_mono))
                last["duration_sec"] = max(0.0, float(now_mono) - start_mono)
                person_ids = list(last.get("person_ids") or [])
                sound_ids = list(last.get("sound_ids") or [])
                neg_ids = list(last.get("neg_ids") or [])
                pre_sound_ids = list(last.get("pre_block_sound_ids") or [])
                bid = (active_id or last.get("event_id", "-"))
                self._log_event(
                    "blocked_exit",
                    blocked_id=bid,
                    last_person_id=(self._last_person_event_id or "-"),
                    duration_sec=f"{float(last['duration_sec']):.2f}",
                )
                self._log_event(
                    "blocked_summary",
                    blocked_id=bid,
                    duration_sec=f"{float(last['duration_sec']):.2f}",
                    person_count=len(person_ids),
                    sound_count=len(sound_ids),
                    neg_count=len(neg_ids),
                    pre_sound_count=len(pre_sound_ids),
                    person_ids=("|".join(person_ids) if person_ids else "-"),
                    sound_ids=("|".join(sound_ids) if sound_ids else "-"),
                    neg_ids=("|".join(neg_ids) if neg_ids else "-"),
                    pre_sound_ids=("|".join(pre_sound_ids) if pre_sound_ids else "-"),
                )
        self._active_blocked_event_id = None
        self._neg_person_seen_for_block = False
        self._neg_person_seen_block_id = None

    def _start_neg_block_event(self, now_mono: float):
        self._neg_block_state = True
        self._neg_block_event_count += 1
        neg_event_id = f"N{self._neg_block_event_count:04d}"
        self._active_neg_block_event_id = neg_event_id
        self._record_block_relation(neg_id=neg_event_id)
        self._neg_block_active_since_mono = float(now_mono)
        self._neg_block_events.append(
            {
                "index": self._neg_block_event_count,
                "event_id": neg_event_id,
                "start_mono": float(now_mono),
                "start_wall": time.time(),
                "end_mono": None,
                "end_wall": None,
                "duration_sec": None,
            }
        )
        self._play_level1_sound()

    def _end_neg_block_event(self, now_mono: float):
        self._neg_block_state = False
        self._stop_level1_sound()
        active_id = self._active_neg_block_event_id
        if self._neg_block_active_since_mono is not None:
            dt = max(0.0, float(now_mono) - float(self._neg_block_active_since_mono))
            self._neg_block_total_sec += dt
            self._neg_block_active_since_mono = None
        if self._neg_block_events:
            last = self._neg_block_events[-1]
            if last.get("end_mono") is None:
                last["end_mono"] = float(now_mono)
                last["end_wall"] = time.time()
                start_mono = float(last.get("start_mono", now_mono))
                last["duration_sec"] = max(0.0, float(now_mono) - start_mono)
                self._log_event(
                    "neg_block_exit",
                    neg_block_id=(active_id or last.get("event_id", "-")),
                    duration_sec=f"{float(last['duration_sec']):.2f}",
                )
        self._active_neg_block_event_id = None

    def _update_blocked(self, now_mono: float):
        now_wall = time.time()
        act_lin = abs(self._actual_linear)
        act_ang = abs(self._actual_angular)
        cmd_lin = abs(self._intent_linear_cmd)
        cmd_ang = abs(self._intent_angular_cmd)

        actual_moving = (
            act_lin >= self._blocked_moving_linear_threshold
            or act_ang >= self._blocked_moving_angular_threshold
        )
        actual_clearly_moving = (
            act_lin >= self._blocked_clear_linear_threshold
            or act_ang >= self._blocked_clear_angular_threshold
        )

        scan_recent = (now_wall - self._scan_last_wall) <= self._blocked_obstacle_scan_timeout
        obstacle_front = (
            scan_recent
            and self._scan_min_front_m <= self._blocked_obstacle_front_max_dist
            and self._scan_close_fraction >= self._blocked_obstacle_min_fraction
        )
        if self._blocked_require_front_obstacle:
            obstacle_gate = obstacle_front
        else:
            obstacle_gate = True

        # Authoritative blocked decision:
        # robot is commanded to move (demand active) but actual motion is near-stopped.
        demand_now = bool(
            cmd_lin >= self._blocked_demand_linear_threshold
            or cmd_ang >= self._blocked_demand_angular_threshold
        )
        if demand_now:
            self._last_demand_mono = float(now_mono)
        demand_active = bool(
            demand_now
            or (
                self._last_demand_mono > 0.0
                and (float(now_mono) - float(self._last_demand_mono)) <= self._blocked_demand_hold_sec
            )
        )

        # Use forward linear demand to avoid false blocked on turn pauses.
        forward_demand_now = bool(cmd_lin >= self._blocked_demand_linear_threshold)
        if forward_demand_now:
            self._last_forward_demand_mono = float(now_mono)
        forward_demand_active = bool(
            forward_demand_now
            or (
                self._last_forward_demand_mono > 0.0
                and (float(now_mono) - float(self._last_forward_demand_mono)) <= self._blocked_demand_hold_sec
            )
        )

        candidate_via_demand = bool(forward_demand_active and (not actual_moving))
        candidate = bool(candidate_via_demand and obstacle_gate)
        if candidate:
            if self._blocked_candidate_since is None:
                self._blocked_candidate_since = float(now_mono)
        else:
            self._blocked_candidate_since = None

        if not self._blocked_state:
            self._blocked_clear_since = None
            if (
                self._blocked_candidate_since is not None
                and (float(now_mono) - float(self._blocked_candidate_since)) >= self._blocked_enter_hold_sec
            ):
                self._start_blocked_event(now_mono)
                decision = "enter_blocked"
            else:
                decision = "stay_clear"
        else:
            clear_evidence = bool((not candidate) and ((not forward_demand_active) or actual_clearly_moving or (not obstacle_gate)))
            if clear_evidence:
                if self._blocked_clear_since is None:
                    self._blocked_clear_since = float(now_mono)
            else:
                self._blocked_clear_since = None
            if (
                self._blocked_clear_since is not None
                and (float(now_mono) - float(self._blocked_clear_since)) >= self._blocked_exit_hold_sec
            ):
                self._end_blocked_event(now_mono)
                decision = "exit_blocked"
            else:
                decision = "stay_blocked"

        blocked_for = 0.0
        if self._blocked_state and self._blocked_active_since_mono is not None:
            blocked_for = max(0.0, float(now_mono) - float(self._blocked_active_since_mono))

        self._blocked_debug = {
            "decision": decision,
            "candidate": candidate,
            "candidate_via_demand": candidate_via_demand,
            "candidate_via_motion_obstacle": candidate,
            "demand_now": demand_now,
            "demand_active": demand_active,
            "forward_demand_now": forward_demand_now,
            "forward_demand_active": forward_demand_active,
            "last_forward_demand_mono": self._last_forward_demand_mono,
            "cmd_linear_abs": cmd_lin,
            "cmd_angular_abs": cmd_ang,
            "last_demand_mono": self._last_demand_mono,
            "actual_moving": actual_moving,
            "actual_clearly_moving": actual_clearly_moving,
            "scan_recent": scan_recent,
            "scan_min_front_m": self._scan_min_front_m,
            "scan_close_fraction": self._scan_close_fraction,
            "obstacle_front": obstacle_front,
            "obstacle_gate": obstacle_gate,
            "blocked_for_sec": blocked_for,
            "actual_linear_abs": act_lin,
            "actual_angular_abs": act_ang,
            "last_actual_motion_wall": self._last_actual_motion_wall,
            "level1_attempt_count": self._level1_attempt_count,
            "level1_success_count": self._level1_success_count,
            "level1_last_status": self._level1_last_status,
        }

        blocked_now = bool(self._blocked_state)
        if self._blocked_last_emitted is None or self._blocked_last_emitted != blocked_now:
            self._blocked_last_emitted = blocked_now
            self.blocked_state_changed.emit(blocked_now)

    def _update_negotiation_block(self, now_mono: float):
        now_wall = time.time()
        human_recent = bool(self._human_last_wall > 0.0 and (now_wall - self._human_last_wall) <= self._human_fresh_sec)
        human_raw_recent = bool(self._human_present_raw and human_recent)

        stop_reason_recent = bool(
            self._stop_reason_last_wall > 0.0
            and (now_wall - self._stop_reason_last_wall) <= self._stop_reason_fresh_sec
        )
        stop_reason_people = bool(stop_reason_recent and (self._stop_reason_value in self._stop_reason_people_codes))

        touch_active = self._touch_is_active(now_wall)
        if self._require_touch_for_sound and (not touch_active) and self._level1_playing:
            self._stop_level1_sound(force=True)

        rec = self._active_block_record()
        block_active = bool(self._blocked_state and rec is not None)
        block_id = str(rec.get("event_id")) if rec is not None else None
        block_start_wall = float(rec.get("start_wall") or 0.0) if rec is not None else 0.0

        # Reset person latch when block id changes.
        if block_active:
            if self._neg_person_seen_block_id != block_id:
                self._neg_person_seen_block_id = block_id
                self._neg_person_seen_for_block = False
        else:
            self._neg_person_seen_for_block = False
            self._neg_person_seen_block_id = None

        in_entry_window = bool(
            block_active
            and block_start_wall > 0.0
            and (now_wall - block_start_wall) <= self._neg_person_entry_window_sec
        )
        if in_entry_window and human_raw_recent:
            self._neg_person_seen_for_block = True

        # Negotiation/audio should require an active blocked event + people stop reason.
        blocked_effective_for_neg = bool(block_active and stop_reason_people)

        human_continuous_sec = 0.0
        if self._human_present_raw and self._human_state_since_wall > 0.0:
            human_continuous_sec = max(0.0, now_wall - float(self._human_state_since_wall))

        human_seen_at_block = bool(self._neg_person_seen_for_block)
        human_timed_to_block = bool(human_seen_at_block)

        if self._require_human_for_negotiation:
            fused_active_raw = bool(blocked_effective_for_neg and human_timed_to_block)
        else:
            fused_active_raw = bool(blocked_effective_for_neg)

        # Keep negotiation semantics aligned with audio policy: if touch is required
        # and not active, do not enter negotiation state.
        if self._require_touch_for_sound and (not touch_active):
            fused_active_raw = False

        if not self._neg_block_state:
            self._neg_clear_since = None
            if fused_active_raw:
                if self._neg_candidate_since is None:
                    self._neg_candidate_since = float(now_mono)
            else:
                self._neg_candidate_since = None

            if (
                self._neg_candidate_since is not None
                and (float(now_mono) - float(self._neg_candidate_since)) >= self._neg_enter_hold_sec
            ):
                self._start_neg_block_event(now_mono)
                decision = "enter_neg_blocked"
            else:
                decision = "stay_neg_clear"
        else:
            self._neg_candidate_since = None
            if not fused_active_raw:
                if self._neg_clear_since is None:
                    self._neg_clear_since = float(now_mono)
            else:
                self._neg_clear_since = None

            if (
                self._neg_clear_since is not None
                and (float(now_mono) - float(self._neg_clear_since)) >= self._neg_exit_hold_sec
            ):
                self._end_neg_block_event(now_mono)
                decision = "exit_neg_blocked"
            else:
                decision = "stay_neg_blocked"

        active_for = 0.0
        if self._neg_block_state and self._neg_block_active_since_mono is not None:
            active_for = max(0.0, float(now_mono) - float(self._neg_block_active_since_mono))

        self._neg_block_debug = {
            "decision": decision,
            "touch_required_for_sound": self._require_touch_for_sound,
            "touch_value": self._touch_value,
            "touch_threshold": self._touch_threshold,
            "touch_last_wall": self._touch_last_wall,
            "touch_active": touch_active,
            "human_active": human_timed_to_block,
            "human_recent": human_recent,
            "human_raw_recent": human_raw_recent,
            "human_hold_active": bool(self._human_active_until_wall > now_wall),
            "human_present_raw": self._human_present_raw,
            "human_confidence": self._human_conf,
            "human_last_wall": self._human_last_wall,
            "blocked_state": bool(self._blocked_state),
            "blocked_candidate_now": bool(self._blocked_debug.get("candidate", False)),
            "block_id": (block_id or "-"),
            "block_start_wall": block_start_wall,
            "in_entry_window": in_entry_window,
            "neg_person_entry_window_sec": self._neg_person_entry_window_sec,
            "neg_person_seen_for_block": self._neg_person_seen_for_block,
            "human_active_min_for_neg_sec": self._human_active_min_for_neg_sec,
            "human_continuous_sec": human_continuous_sec,
            "human_seen_at_block": human_seen_at_block,
            "human_timed_to_block": human_timed_to_block,
            "stop_reason_value": self._stop_reason_value,
            "stop_reason_recent": stop_reason_recent,
            "stop_reason_people": stop_reason_people,
            "stop_reason_duration": self._stop_reason_duration,
            "stop_reason_summary": self._stop_reason_summary,
            "fused_active_raw": fused_active_raw,
            "neg_enter_hold_sec": self._neg_enter_hold_sec,
            "neg_exit_hold_sec": self._neg_exit_hold_sec,
            "neg_candidate_since": self._neg_candidate_since,
            "neg_clear_since": self._neg_clear_since,
            "active_for_sec": active_for,
            "require_human_for_negotiation": self._require_human_for_negotiation,
        }

    def _confirmed_block_events(self):
        out = []
        min_sec = float(self._blocked_confirm_min_sec)
        for ev in self._blocked_events:
            dur = float(ev.get("duration_sec") or 0.0)
            if dur >= min_sec:
                out.append(ev)
        return out

    def _write_run_summary(self):
        if self._run_summary_written or self._debug_dir is None:
            return
        self._run_summary_written = True

        try:
            end_wall = time.time()
            duration = max(0.0, end_wall - self._run_started_wall)
            if self._blocked_state:
                self._end_blocked_event(time.perf_counter())
            if self._neg_block_state:
                self._end_neg_block_event(time.perf_counter())
            stem = f"run_summary_{int(end_wall * 1000)}"
            json_path = self._debug_dir / f"{stem}.json"
            svg_path = self._debug_dir / f"{stem}.svg"
            confirmed_events = self._confirmed_block_events()
            confirmed_total_sec = sum(float(ev.get("duration_sec") or 0.0) for ev in confirmed_events)

            merged_gap_sec = 1.5
            merged_event_count = 0
            prev_end_wall = None
            for ev in confirmed_events:
                start_wall = float(ev.get("start_wall") or 0.0)
                end_wall_ev = float(ev.get("end_wall") or start_wall)
                if prev_end_wall is None or (start_wall - prev_end_wall) > merged_gap_sec:
                    merged_event_count += 1
                if prev_end_wall is None:
                    prev_end_wall = end_wall_ev
                else:
                    prev_end_wall = max(prev_end_wall, end_wall_ev)

            easy_blocks = []
            for ev in self._blocked_events:
                person_ids = list(ev.get("person_ids") or [])
                sound_ids = list(ev.get("sound_ids") or [])
                neg_ids = list(ev.get("neg_ids") or [])
                pre_sound_ids = list(ev.get("pre_block_sound_ids") or [])
                easy_blocks.append(
                    {
                        "blocked_id": ev.get("event_id", f"B{int(ev.get('index', 0)):04d}"),
                        "index": ev.get("index"),
                        "duration_sec": ev.get("duration_sec"),
                        "person_detected": bool(person_ids),
                        "sound_triggered": bool(sound_ids),
                        "negotiation_triggered": bool(neg_ids),
                        "person_count": len(person_ids),
                        "sound_count": len(sound_ids),
                        "neg_count": len(neg_ids),
                        "pre_sound_count": len(pre_sound_ids),
                        "person_ids": person_ids,
                        "sound_ids": sound_ids,
                        "neg_ids": neg_ids,
                        "pre_block_sound_ids": pre_sound_ids,
                    }
                )

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
                "blocked_detection": {
                    "event_count": len(confirmed_events),
                    "raw_event_count": self._blocked_event_count,
                    "confirmed_event_count": len(confirmed_events),
                    "confirmed_min_duration_sec": self._blocked_confirm_min_sec,
                    "total_blocked_sec": self._blocked_total_sec,
                    "confirmed_total_blocked_sec": confirmed_total_sec,
                    "merged_gap_sec": merged_gap_sec,
                    "merged_event_count": merged_event_count,
                    "events": self._blocked_events,
                    "easy_blocks": easy_blocks,
                    "confirmed_events": confirmed_events,
                    "thresholds": {
                        "demand_linear": self._blocked_demand_linear_threshold,
                        "demand_angular": self._blocked_demand_angular_threshold,
                        "moving_linear": self._blocked_moving_linear_threshold,
                        "moving_angular": self._blocked_moving_angular_threshold,
                        "clear_linear": self._blocked_clear_linear_threshold,
                        "clear_angular": self._blocked_clear_angular_threshold,
                        "require_front_obstacle": self._blocked_require_front_obstacle,
                        "obstacle_front_max_dist": self._blocked_obstacle_front_max_dist,
                        "obstacle_min_fraction": self._blocked_obstacle_min_fraction,
                        "obstacle_scan_timeout": self._blocked_obstacle_scan_timeout,
                        "enter_hold_sec": self._blocked_enter_hold_sec,
                        "exit_hold_sec": self._blocked_exit_hold_sec,
                        "demand_hold_sec": self._blocked_demand_hold_sec,
                    },
                },
                "negotiation_block_detection": {
                    "event_count": self._neg_block_event_count,
                    "total_blocked_sec": self._neg_block_total_sec,
                    "events": self._neg_block_events,
                    "human_hold_sec": self._human_hold_sec,
                    "human_fresh_sec": self._human_fresh_sec,
                    "require_human_for_negotiation": self._require_human_for_negotiation,
                },
                "level1_sound": {
                    "attempt_count": self._level1_attempt_count,
                    "success_count": self._level1_success_count,
                    "last_status": self._level1_last_status,
                    "path": self._level1_sound_path,
                    "cooldown_sec": self._level1_sound_cooldown_sec,
                },
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
            # Reverse-like servo values are ambiguous; keep a strong turn in the
            # incoming servo direction instead of collapsing to straight-forward.
            self._reverse_pending += 1
            if abs(angle_deg) > 1e-6:
                turn_sign = 1.0 if angle_deg > 0.0 else -1.0
            else:
                turn_sign = 0.0
            if turn_sign == 0.0:
                angle_deg = 0.0
            else:
                angle_deg = turn_sign * (self._max_forward_turn_deg * 0.95)
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
                "blocked_detection": {
                    "is_blocked": self._blocked_state,
                    "event_count": self._blocked_event_count,
                    "total_blocked_sec": self._blocked_total_sec,
                    "diag": dict(self._blocked_debug),
                },
                "human_detection": {
                    "human_present_raw": self._human_present_raw,
                    "human_confidence": self._human_conf,
                    "human_last_wall": self._human_last_wall,
                    "human_hold_sec": self._human_hold_sec,
                    "human_fresh_sec": self._human_fresh_sec,
                    "human_active_until_wall": self._human_active_until_wall,
                    "human_recent": bool(self._human_last_wall > 0.0 and (time.time() - self._human_last_wall) <= self._human_fresh_sec),
                    "human_active": bool(self._human_present_raw and self._human_last_wall > 0.0 and (time.time() - self._human_last_wall) <= self._human_fresh_sec),
                },
                "negotiation_block_detection": {
                    "is_blocked_for_negotiation": self._neg_block_state,
                    "event_count": self._neg_block_event_count,
                    "total_blocked_sec": self._neg_block_total_sec,
                    "diag": dict(self._neg_block_debug),
                },
            }
            json_path.write_text(json.dumps(snapshot, indent=2), encoding="utf-8")

            w = max(1, self.width())
            h = max(1, self.height())
            parts = [
                f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}">',
                '<rect x="0" y="0" width="100%" height="100%" fill="black"/>',
            ]

            draw_pts = None
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

            # Include cyan motion pulse in debug SVG to match on-screen animation.
            if draw_pts is not None and len(draw_pts) >= 2:
                total_len = self._polyline_length(draw_pts)
                if total_len > 4.0:
                    pulse_head = self._anim_phase_px % total_len
                    pulse_len = max(40.0, self._line_width * 1.7)
                    pulse_tail = max(0.0, pulse_head - pulse_len)
                    pulse_pts = self._polyline_slice_by_length(draw_pts, pulse_tail, pulse_head)
                    if len(pulse_pts) >= 2:
                        pstr = " ".join(f"{pp.x():.2f},{pp.y():.2f}" for pp in pulse_pts)
                        parts.append(
                            '<polyline points="{}" fill="none" stroke="rgba(80,220,255,0.59)" stroke-width="{:.2f}" '
                            'stroke-linecap="round" stroke-linejoin="round"/>'.format(
                                pstr, max(2.0, self._line_width * 0.86)
                            )
                        )
                        parts.append(
                            '<polyline points="{}" fill="none" stroke="rgba(120,245,255,1.0)" stroke-width="{:.2f}" '
                            'stroke-linecap="round" stroke-linejoin="round"/>'.format(
                                pstr, max(1.5, self._line_width * 0.44)
                            )
                        )
                        hp = pulse_pts[-1]
                        hx = hp.x()
                        hy = hp.y()
                        r1 = max(5.0, self._line_width * 0.28)
                        r2 = max(2.0, self._line_width * 0.12)
                        parts.append(
                            '<circle cx="{:.2f}" cy="{:.2f}" r="{:.2f}" fill="rgba(80,220,255,0.67)"/>'.format(
                                hx, hy, r1
                            )
                        )
                        parts.append(
                            '<circle cx="{:.2f}" cy="{:.2f}" r="{:.2f}" fill="rgba(180,255,255,1.0)"/>'.format(
                                hx, hy, r2
                            )
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
                head_w = max(2.0, self._line_width * 1.05)
                parts.append(
                    '<line x1="{:.2f}" y1="{:.2f}" x2="{:.2f}" y2="{:.2f}" '
                    'stroke="white" stroke-width="{:.2f}" stroke-linecap="round" opacity="0.35"/>'.format(
                        t[0], t[1], l[0], l[1], head_w * 1.75
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
                        t[0], t[1], r[0], r[1], head_w * 1.75
                    )
                )
                parts.append(
                    '<line x1="{:.2f}" y1="{:.2f}" x2="{:.2f}" y2="{:.2f}" '
                    'stroke="white" stroke-width="{:.2f}" stroke-linecap="round"/>'.format(
                        t[0], t[1], r[0], r[1], head_w
                    )
                )
                parts.append(
                    '<circle cx="{:.2f}" cy="{:.2f}" r="{:.2f}" fill="white" opacity="0.35"/>'.format(
                        t[0], t[1], max(1.2, head_w * 0.2)
                    )
                )
                parts.append(
                    '<circle cx="{:.2f}" cy="{:.2f}" r="{:.2f}" fill="white"/>'.format(
                        t[0], t[1], max(0.8, head_w * 0.30)
                    )
                )
            blocked_line = (
                f"blocked={1 if self._blocked_state else 0} "
                f"count={self._blocked_event_count} "
                f"human={1 if self._neg_block_debug.get('human_active', False) else 0} "
                f"neg={1 if self._neg_block_state else 0} "
                f"decision={self._blocked_debug.get('decision', 'na')}"
            )
            parts.append(
                '<text x="16" y="34" fill="#9af7ff" font-size="20" font-family="monospace">'
                f"{blocked_line}"
                "</text>"
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
        self._anim_phase_px = (self._anim_phase_px + self._anim_speed_px_s * dt) % 100000.0
        self._anim_head_phase = (self._anim_head_phase + dt * self._anim_head_hz * 2.0 * math.pi) % (
            2.0 * math.pi
        )

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

        self._update_blocked(now)
        self._update_negotiation_block(now)
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

        # Strong, visible pulse: travels from bottom -> top and repeats.
        total_len = self._polyline_length(draw_pts)
        if total_len > 4.0:
            pulse_head = self._anim_phase_px % total_len
            pulse_len = max(40.0, self._line_width * 1.7)
            pulse_tail = max(0.0, pulse_head - pulse_len)
            pulse_pts = self._polyline_slice_by_length(draw_pts, pulse_tail, pulse_head)

            if len(pulse_pts) >= 2:
                pulse_path = QtGui.QPainterPath()
                pulse_path.moveTo(pulse_pts[0])
                for pt in pulse_pts[1:]:
                    pulse_path.lineTo(pt)

                # Animate the main white projection itself (not a separate inner white line).
                # Use a narrow bright highlight so motion is visible over the static white body.
                p.setPen(
                    QtGui.QPen(
                        QtGui.QColor(255, 255, 255, 210),
                        max(2.0, self._line_width * 0.46),
                        QtCore.Qt.PenStyle.SolidLine,
                        QtCore.Qt.PenCapStyle.RoundCap,
                        QtCore.Qt.PenJoinStyle.RoundJoin,
                    )
                )
                p.drawPath(pulse_path)
                p.setPen(
                    QtGui.QPen(
                        QtGui.QColor(255, 255, 255, 255),
                        max(1.2, self._line_width * 0.18),
                        QtCore.Qt.PenStyle.SolidLine,
                        QtCore.Qt.PenCapStyle.RoundCap,
                        QtCore.Qt.PenJoinStyle.RoundJoin,
                    )
                )
                p.drawPath(pulse_path)

                # Cyan pulse
                p.setPen(
                    QtGui.QPen(
                        QtGui.QColor(80, 220, 255, 150),
                        max(2.0, self._line_width * 0.86),
                        QtCore.Qt.PenStyle.SolidLine,
                        QtCore.Qt.PenCapStyle.RoundCap,
                        QtCore.Qt.PenJoinStyle.RoundJoin,
                    )
                )
                p.drawPath(pulse_path)
                p.setPen(
                    QtGui.QPen(
                        QtGui.QColor(120, 245, 255, 255),
                        max(1.5, self._line_width * 0.44),
                        QtCore.Qt.PenStyle.SolidLine,
                        QtCore.Qt.PenCapStyle.RoundCap,
                        QtCore.Qt.PenJoinStyle.RoundJoin,
                    )
                )
                p.drawPath(pulse_path)

                # Cyan head marker
                head_pt = pulse_pts[-1]
                r1 = max(5.0, self._line_width * 0.28)
                r2 = max(2.0, self._line_width * 0.12)
                p.setPen(QtCore.Qt.PenStyle.NoPen)
                # White moving tip marker for the same rising motion perception.
                p.setBrush(QtGui.QColor(255, 255, 255, 175))
                p.drawEllipse(head_pt, max(3.6, r1 * 0.86), max(3.6, r1 * 0.86))
                p.setBrush(QtGui.QColor(255, 255, 255, 255))
                p.drawEllipse(head_pt, max(1.9, r2 * 1.35), max(1.9, r2 * 1.35))

                p.setBrush(QtGui.QColor(80, 220, 255, 170))
                p.drawEllipse(head_pt, r1, r1)
                p.setBrush(QtGui.QColor(180, 255, 255, 255))
                p.drawEllipse(head_pt, r2, r2)

        tip_pt = QtCore.QPointF(*tri["tip"])
        left_pt = QtCore.QPointF(*tri["left"])
        right_pt = QtCore.QPointF(*tri["right"])
        pulse = 0.90 + 0.24 * (0.5 + 0.5 * math.sin(self._anim_head_phase))
        head_w = max(2.0, self._line_width * 1.05 * pulse)

        p.setPen(
            QtGui.QPen(
                QtGui.QColor(255, 255, 255, 95),
                head_w * 1.75,
                QtCore.Qt.PenStyle.SolidLine,
                QtCore.Qt.PenCapStyle.RoundCap,
                QtCore.Qt.PenJoinStyle.RoundJoin,
            )
        )
        p.drawLine(tip_pt, left_pt)
        p.drawLine(tip_pt, right_pt)

        p.setPen(QtCore.Qt.PenStyle.NoPen)
        p.setBrush(QtGui.QColor(255, 255, 255, 95))
        p.drawEllipse(tip_pt, max(1.2, head_w * 0.52), max(1.2, head_w * 0.52))
        p.setBrush(QtGui.QColor(255, 255, 255, 235))
        p.drawEllipse(tip_pt, max(0.8, head_w * 0.30), max(0.8, head_w * 0.30))
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
        p.setPen(QtCore.Qt.PenStyle.NoPen)
        p.setBrush(QtGui.QColor(255, 255, 255, 245))
        p.drawEllipse(tip_pt, max(0.7, head_w * 0.22), max(0.7, head_w * 0.22))

    def closeEvent(self, event):
        self._write_run_summary()
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser(
        description="Fullscreen dynamic path projector arrow driven by servo topic"
    )
    parser.add_argument("--topic", default="/cabot/servo_target", help="Int16 topic for directional servo target")
    parser.add_argument("--path-topic", default="/plan", help="nav_msgs/Path topic for planned trajectory debug")
    parser.add_argument("--motion-topic", default="/cabot/cmd_vel_adapter", help="geometry_msgs/Twist topic for demanded robot motion (pre-safety)")
    parser.add_argument("--actual-motion-topic", default="/odom", help="nav_msgs/Odometry topic for actual robot motion")
    parser.add_argument("--scan-topic", default="/scan", help="sensor_msgs/LaserScan topic for front obstacle gating")
    parser.add_argument("--human-topic", default="/people", help="Human topic (people_msgs/People)")
    parser.add_argument("--people-target-frame", default="base_footprint", help="Target robot frame used to evaluate /people front proximity")
    parser.add_argument("--people-front-max-dist", type=float, default=2.0, help="Max forward distance (m) for /people to count as human in front")
    parser.add_argument("--people-front-half-angle-deg", type=float, default=60.0, help="Front half-angle (deg) for /people gating (total cone = 2x this value)")
    parser.add_argument("--touch-topic", default="/cabot/touch", help="std_msgs/Int16 topic from handle touch sensor")
    parser.add_argument("--stop-reason-topic", default="/stop_reason", help="cabot_msgs/StopReason topic from cabot_ui stop reasoner")
    parser.add_argument("--blocked-topic", default="/projector/blocked_state", help="std_msgs/Bool topic publishing projector blocked state")
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

    parser.add_argument("--debug-dump-dir", default="", help="Write debug JSON/SVG snapshots to this directory")
    parser.add_argument("--debug-dump-every", type=int, default=8, help="Dump one debug sample every N ticks")
    parser.add_argument("--debug-max-dumps", type=int, default=120, help="Maximum number of debug samples")
    parser.add_argument("--debug", "--debugf", dest="debug", action="store_true", help="Enable debug dump to default projector_live/tmp/projector_debug")
    parser.add_argument("--blocked-demand-linear-threshold", type=float, default=0.05, help="cmd_vel linear threshold for demand-active")
    parser.add_argument("--blocked-demand-angular-threshold", type=float, default=0.20, help="cmd_vel angular threshold for demand-active")
    parser.add_argument("--blocked-moving-linear-threshold", type=float, default=0.04, help="odom linear threshold for blocked candidate (actual-moving)")
    parser.add_argument("--blocked-moving-angular-threshold", type=float, default=0.25, help="odom angular threshold for blocked candidate (actual-moving)")
    parser.add_argument("--blocked-clear-linear-threshold", type=float, default=0.08, help="odom linear threshold for clear evidence while blocked")
    parser.add_argument("--blocked-clear-angular-threshold", type=float, default=0.40, help="odom angular threshold for clear evidence while blocked")
    parser.add_argument("--blocked-require-front-obstacle", action="store_true", default=True, help="Require front obstacle evidence from /scan to enter blocked")
    parser.add_argument("--no-blocked-require-front-obstacle", dest="blocked_require_front_obstacle", action="store_false", help="Do not require /scan obstacle evidence")
    parser.add_argument("--blocked-obstacle-front-half-angle-deg", type=float, default=45.0, help="Front scan half-angle used for obstacle evidence (total cone = 2x this value)")
    parser.add_argument("--blocked-obstacle-front-max-dist", type=float, default=0.75, help="Front obstacle max distance (m) to consider blocked")
    parser.add_argument("--blocked-obstacle-min-fraction", type=float, default=0.12, help="Minimum fraction of front rays within max distance")
    parser.add_argument("--blocked-obstacle-scan-timeout", type=float, default=0.8, help="Max scan staleness (s) for obstacle evidence")
    parser.add_argument("--require-human-for-negotiation", action="store_true", default=True, help="Count negotiation blocks only when blocked and a person is present")
    parser.add_argument("--no-require-human-for-negotiation", dest="require_human_for_negotiation", action="store_false", help="Do not require person presence for negotiation block count")
    parser.add_argument("--human-hold", type=float, default=0.0, help="Keep person presence active this many seconds after last positive detection")
    parser.add_argument("--human-fresh-sec", type=float, default=0.9, help="Treat human topic as valid only if updated within this many seconds")
    parser.add_argument("--human-active-min-for-neg", type=float, default=0.15, help="Minimum continuous human-active time (s) before entering negotiation")
    parser.add_argument("--neg-enter-hold", type=float, default=0.25, help="Seconds fused negotiation candidate must persist before enter")
    parser.add_argument("--neg-exit-hold", type=float, default=0.50, help="Seconds fused negotiation clear must persist before exit")
    parser.add_argument("--neg-person-entry-window", type=float, default=0.80, help="Seconds after blocked-enter to accept person detection for this blocked event")
    parser.add_argument("--stop-reason-fresh-sec", type=float, default=0.8, help="Treat stop_reason topic as valid only if updated within this many seconds")
    parser.add_argument("--blocked-enter-hold", type=float, default=0.45, help="Seconds candidate must persist before blocked=True")
    parser.add_argument("--blocked-exit-hold", type=float, default=0.70, help="Seconds clear must persist before blocked=False")
    parser.add_argument("--blocked-demand-hold", type=float, default=1.20, help="Seconds demand remains active after last cmd_vel demand")
    parser.add_argument("--blocked-confirm-min", type=float, default=1.00, help="Minimum event duration (s) to count as confirmed blocked event")
    parser.add_argument("--level1-sound-path", default="resources/signal_intention.wav", help="Audio file played on blocked enter (Level 1 cue)")
    parser.add_argument("--level1-sound-cooldown", type=float, default=2.0, help="Minimum seconds between Level 1 cue plays")
    parser.add_argument("--level1-sound-min-play", type=float, default=0.8, help="Minimum audible seconds before Level 1 cue can be stopped")
    parser.add_argument("--require-touch-for-sound", action="store_true", default=True, help="Play negotiation sound only while touch sensor is active")
    parser.add_argument("--no-require-touch-for-sound", dest="require_touch_for_sound", action="store_false", help="Do not require touch sensor for negotiation sound")
    parser.add_argument("--touch-threshold", type=int, default=1, help="Touch value threshold to consider user finger present")
    parser.add_argument("--touch-hold-sec", type=float, default=0.25, help="Keep touch active this many seconds after last above-threshold sample")
    parser.add_argument("--haptic-topic", default="/cabot/vibrator1", help="std_msgs/UInt8 topic for haptic cue on blocked enter")
    parser.add_argument("--haptic-value", type=int, default=1, help="UInt8 payload for haptic cue")
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
        debug_dump_dir=args.debug_dump_dir,
        debug_dump_every=args.debug_dump_every,
        debug_max_dumps=args.debug_max_dumps,
        blocked_demand_linear_threshold=args.blocked_demand_linear_threshold,
        blocked_demand_angular_threshold=args.blocked_demand_angular_threshold,
        blocked_moving_linear_threshold=args.blocked_moving_linear_threshold,
        blocked_moving_angular_threshold=args.blocked_moving_angular_threshold,
        blocked_clear_linear_threshold=args.blocked_clear_linear_threshold,
        blocked_clear_angular_threshold=args.blocked_clear_angular_threshold,
        blocked_require_front_obstacle=args.blocked_require_front_obstacle,
        blocked_obstacle_front_max_dist=args.blocked_obstacle_front_max_dist,
        blocked_obstacle_min_fraction=args.blocked_obstacle_min_fraction,
        blocked_obstacle_scan_timeout=args.blocked_obstacle_scan_timeout,
        require_human_for_negotiation=args.require_human_for_negotiation,
        human_hold_sec=args.human_hold,
        human_fresh_sec=args.human_fresh_sec,
        human_active_min_for_neg_sec=args.human_active_min_for_neg,
        neg_enter_hold_sec=args.neg_enter_hold,
        neg_exit_hold_sec=args.neg_exit_hold,
        neg_person_entry_window_sec=args.neg_person_entry_window,
        stop_reason_fresh_sec=args.stop_reason_fresh_sec,
        blocked_enter_hold_sec=args.blocked_enter_hold,
        blocked_exit_hold_sec=args.blocked_exit_hold,
        blocked_demand_hold_sec=args.blocked_demand_hold,
        blocked_confirm_min_sec=args.blocked_confirm_min,
        level1_sound_path=args.level1_sound_path,
        level1_sound_cooldown_sec=args.level1_sound_cooldown,
        level1_sound_min_play_sec=args.level1_sound_min_play,
        require_touch_for_sound=args.require_touch_for_sound,
        touch_threshold=args.touch_threshold,
        touch_hold_sec=args.touch_hold_sec,
        haptic_value=args.haptic_value,
    )

    bridge = RosBridge(
        topic=args.topic,
        path_topic=args.path_topic,
        motion_topic=args.motion_topic,
        actual_motion_topic=args.actual_motion_topic,
        scan_topic=args.scan_topic,
        human_topic=args.human_topic,
        people_target_frame=args.people_target_frame,
        people_front_max_dist=args.people_front_max_dist,
        people_front_half_angle_deg=args.people_front_half_angle_deg,
        touch_topic=args.touch_topic,
        stop_reason_topic=args.stop_reason_topic,
        blocked_topic=args.blocked_topic,
        haptic_topic=args.haptic_topic,
        obstacle_front_half_angle_deg=args.blocked_obstacle_front_half_angle_deg,
        obstacle_dist_m=args.blocked_obstacle_front_max_dist,
    )
    bridge.angle_changed.connect(win.on_servo_angle)
    bridge.planned_path_changed.connect(win.on_planned_path)
    bridge.intent_motion_changed.connect(win.on_intent_motion)
    bridge.actual_motion_changed.connect(win.on_actual_motion)
    bridge.obstacle_scan_changed.connect(win.on_obstacle_scan)
    bridge.human_presence_changed.connect(win.on_human_presence)
    bridge.stop_reason_changed.connect(win.on_stop_reason)
    bridge.touch_changed.connect(win.on_touch_value)
    win.blocked_state_changed.connect(bridge.on_blocked_state_changed)
    win.haptic_triggered.connect(bridge.on_haptic_triggered)
    bridge.status.connect(lambda msg: print(f"[projector_arrow_live] {msg}", file=sys.stderr, flush=True))
    bridge.start()

    def _cleanup():
        bridge.stop()
        win._write_run_summary()
        time.sleep(0.1)

    app.aboutToQuit.connect(_cleanup)
    app.exec()


if __name__ == "__main__":
    main()
