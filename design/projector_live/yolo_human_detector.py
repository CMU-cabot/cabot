#!/usr/bin/env python3
import argparse
import json
import threading
import time
from collections import deque
from pathlib import Path


def parse_args():
    here = Path(__file__).resolve().parent
    p = argparse.ArgumentParser(description="YOLO person-in-front detector publisher for projector negotiation")
    p.add_argument("--model", default=str(here / "resources" / "yolo26n.pt"), help="Path to YOLO model file")
    p.add_argument("--image-topic", default="/camera/color/image_raw", help="sensor_msgs/Image topic")
    p.add_argument("--rotate", type=int, default=0, choices=[0, 90, 180, 270], help="Rotate input image before inference")
    p.add_argument("--out-topic", default="/projector/human_in_front", help="Output std_msgs/Bool topic")
    p.add_argument("--max-fps", type=float, default=5.0, help="Maximum inference FPS")
    p.add_argument("--conf", type=float, default=0.35, help="Minimum person confidence")
    p.add_argument("--object-conf", type=float, default=0.25, help="Minimum non-person object confidence")
    p.add_argument("--imgsz", type=int, default=640, help="Inference image size")
    p.add_argument("--device", default="", help="YOLO device string, e.g. cpu, cuda:0")
    p.add_argument("--front-roi-width", type=float, default=0.65, help="Center ROI width ratio [0..1]")
    p.add_argument("--min-box-area", type=float, default=0.010, help="Minimum bbox area ratio [0..1]")
    p.add_argument("--hold-sec", type=float, default=0.6, help="Hold positive detection for this many seconds")

    p.add_argument("--motion-topic", default="/cabot/cmd_vel", help="Commanded motion topic (geometry_msgs/Twist)")
    p.add_argument("--actual-motion-topic", default="/odom", help="Actual odom topic (nav_msgs/Odometry)")
    p.add_argument("--blocked-topic", default="/projector/blocked_state", help="Projector blocked-state topic (std_msgs/Bool)")
    p.add_argument("--demand-hold-sec", type=float, default=1.0, help="Hold commanded demand active for this long")
    p.add_argument("--stop-demand-linear-threshold", type=float, default=0.05, help="Linear cmd demand threshold")
    p.add_argument("--stop-demand-angular-threshold", type=float, default=0.20, help="Angular cmd demand threshold")
    p.add_argument("--stop-moving-linear-threshold", type=float, default=0.04, help="Linear odom moving threshold")
    p.add_argument("--stop-moving-angular-threshold", type=float, default=0.25, help="Angular odom moving threshold")
    p.add_argument("--stop-hold-sec", type=float, default=0.45, help="Stop candidate must persist this long")

    p.add_argument(
        "--snapshot-dir",
        default=str(here / "tmp" / "yolo_blocked_snapshots"),
        help="Directory for blocked-stop YOLO bbox snapshots",
    )
    p.add_argument("--snapshot-cooldown-sec", type=float, default=1.0, help="Min seconds between snapshots")
    p.add_argument("--snapshot-max", type=int, default=0, help="Max snapshots (<=0 means unlimited)")
    p.add_argument("--snapshot-always", action="store_true", help="Save snapshots even when robot is not in blocked-stop state")
    p.add_argument("--snapshot-use-projector-blocked", action="store_true", default=True, help="Use projector blocked topic as snapshot gate")
    p.add_argument("--no-snapshot-use-projector-blocked", dest="snapshot_use_projector_blocked", action="store_false", help="Use local cmd/odom stop gate for snapshots")
    p.add_argument("--blocked-hold-sec", type=float, default=0.25, help="Hold projector blocked state for this many seconds")
    p.add_argument("--snapshot-retry-no-human-sec", type=float, default=0.25, help="Retry interval while blocked when no person is in ROI")
    p.add_argument("--snapshot-retry-max-per-block", type=int, default=4, help="Max extra retries per blocked event when no person in ROI")

    p.add_argument("--verbose", action="store_true", help="Print detector status")
    return p.parse_args()


def main():
    args = parse_args()

    try:
        # Import YOLO/Torch first on ARM to avoid OpenMP TLS conflicts
        # that can occur when OpenCV stack is loaded first.
        from ultralytics import YOLO
        import cv2
        from cv_bridge import CvBridge
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Bool, Float32
    except Exception as ex:
        print(f"[yolo_human_detector] import failed: {ex}")
        return 1

    class HumanDetectorNode(Node):
        def __init__(self):
            super().__init__("yolo_human_detector")
            self.bridge = CvBridge()
            self.model = YOLO(args.model)
            self.pub_bool = self.create_publisher(Bool, args.out_topic, 10)
            self.pub_conf = self.create_publisher(Float32, args.out_topic.rstrip("/") + "_confidence", 10)
            self.sub = self.create_subscription(Image, args.image_topic, self._on_image, qos_profile_sensor_data)
            self.sub_cmd = self.create_subscription(Twist, args.motion_topic, self._on_cmd_vel, 10)
            self.sub_odom = self.create_subscription(Odometry, args.actual_motion_topic, self._on_odom, 10)
            self.sub_blocked = self.create_subscription(Bool, args.blocked_topic, self._on_projector_blocked, 10)

            self._lock = threading.Lock()
            self._latest_frame = None
            self._latest_stamp = 0.0
            self._last_infer = 0.0
            self._last_publish = 0.0
            self._human_until = 0.0
            self._last_conf = 0.0
            self._interval = 1.0 / max(0.5, float(args.max_fps))
            self._object_active = False
            self._last_detect = None

            self._last_cmd_time = 0.0
            self._cmd_lin = 0.0
            self._cmd_ang = 0.0
            self._last_odom_time = 0.0
            self._odom_lin = 0.0
            self._odom_ang = 0.0

            self._projector_blocked_raw = False
            self._projector_blocked_until = 0.0
            self._projector_blocked_last_time = 0.0
            self._pending_blocked_snapshot = False
            self._blocked_retry_count = 0
            self._last_blocked_retry_time = 0.0
            self._blocked_event_wall = 0.0
            self._blocked_event_frame = None
            self._blocked_event_stamp = 0.0
            self._blocked_burst_pre_sec = 0.20
            self._blocked_burst_post_sec = 0.20
            self._blocked_burst_post_remaining = 0
            self._blocked_burst_next_time = 0.0
            self._frame_history = deque(maxlen=16)

            self._stop_candidate_since = None
            self._stop_active = False
            self._last_snapshot_time = 0.0
            self._snapshot_count = 0
            self._snapshot_dir = Path(args.snapshot_dir)
            self._snapshot_dir.mkdir(parents=True, exist_ok=True)

            self.timer = self.create_timer(0.02, self._tick)
            self.get_logger().info(
                f"model={args.model} image={args.image_topic} rotate={args.rotate} out={args.out_topic} max_fps={args.max_fps} "
                f"motion={args.motion_topic} odom={args.actual_motion_topic} blocked_topic={args.blocked_topic} "
                f"snapshot_dir={self._snapshot_dir} use_projector_blocked={int(args.snapshot_use_projector_blocked)}"
            )

        def _on_image(self, msg):
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception:
                return
            if args.rotate == 90:
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            elif args.rotate == 180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            elif args.rotate == 270:
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            if stamp <= 0:
                stamp = time.time()
            with self._lock:
                self._latest_frame = frame
                self._latest_stamp = stamp
                self._frame_history.append((time.time(), frame.copy(), float(stamp)))

        def _on_cmd_vel(self, msg):
            self._cmd_lin = float(msg.linear.x)
            self._cmd_ang = float(msg.angular.z)
            self._last_cmd_time = time.time()

        def _on_odom(self, msg):
            self._odom_lin = float(msg.twist.twist.linear.x)
            self._odom_ang = float(msg.twist.twist.angular.z)
            self._last_odom_time = time.time()

        def _on_projector_blocked(self, msg):
            now = time.time()
            new_state = bool(msg.data)
            if new_state and (not self._projector_blocked_raw):
                self._pending_blocked_snapshot = True
                self._blocked_retry_count = 0
                self._last_blocked_retry_time = 0.0
                self._blocked_event_wall = now
                with self._lock:
                    if self._latest_frame is not None:
                        self._blocked_event_frame = self._latest_frame.copy()
                        self._blocked_event_stamp = float(self._latest_stamp)
                    else:
                        self._blocked_event_frame = None
                        self._blocked_event_stamp = 0.0
                self._snapshot_blocked_enter(now)
            if (not new_state) and self._projector_blocked_raw:
                self._blocked_retry_count = 0
                self._last_blocked_retry_time = 0.0
                self._blocked_event_frame = None
                self._blocked_event_stamp = 0.0
                self._blocked_event_wall = 0.0
            self._projector_blocked_raw = new_state
            self._projector_blocked_last_time = now
            if self._projector_blocked_raw:
                self._projector_blocked_until = max(self._projector_blocked_until, now + max(0.0, float(args.blocked_hold_sec)))

        def _projector_blocked_active(self, now):
            return bool((self._projector_blocked_raw) or (now <= self._projector_blocked_until))

        def _can_save_snapshot(self):
            return not (int(args.snapshot_max) > 0 and self._snapshot_count >= int(args.snapshot_max))

        def _history_frame_near(self, wall_t, max_abs_sec=0.45):
            with self._lock:
                hist = list(self._frame_history)
            if not hist:
                return None
            best = None
            best_dt = None
            target = float(wall_t)
            limit = max(0.0, float(max_abs_sec))
            for wt, fr, st in hist:
                dt = abs(float(wt) - target)
                if best is None or dt < best_dt:
                    best = (wt, fr, st)
                    best_dt = dt
            if best is None:
                return None
            if best_dt is not None and best_dt > limit:
                return None
            return best

        def _snapshot_blocked_enter(self, now):
            if not self._can_save_snapshot():
                return

            # Pre frame near (t - 0.2s)
            pre = self._history_frame_near(float(now) - float(self._blocked_burst_pre_sec))
            if pre is not None and self._can_save_snapshot():
                pre_wall, pre_frame, pre_stamp = pre
                pre_detect = self._last_detect or {
                    "present": False,
                    "best_conf": 0.0,
                    "object_present": False,
                    "reason": "blocked_burst_pre_no_detect",
                    "person_count": 0,
                    "roi_person_count": 0,
                    "object_count": 0,
                    "detections": [],
                }
                self._save_snapshot(
                    now,
                    pre_frame,
                    float(pre_stamp),
                    pre_detect,
                    extra_meta={
                        "snapshot_source": "blocked_burst_pre",
                        "blocked_event_wall": float(now),
                        "blocked_to_snapshot_delay_sec": float(pre_wall) - float(now),
                    },
                )

            with self._lock:
                if self._latest_frame is None:
                    return
                frame = self._latest_frame.copy()
                frame_stamp = float(self._latest_stamp)
            detect = self._last_detect
            if detect is None:
                detect = {
                    "present": False,
                    "best_conf": 0.0,
                    "object_present": False,
                    "reason": "blocked_enter_no_detect",
                    "person_count": 0,
                    "roi_person_count": 0,
                    "object_count": 0,
                    "detections": [],
                }
            extra_meta = {
                "snapshot_source": "blocked_edge_callback",
                "blocked_event_wall": float(now),
                "blocked_to_snapshot_delay_sec": 0.0,
            }
            if self._can_save_snapshot():
                self._save_snapshot(now, frame, frame_stamp, detect, extra_meta=extra_meta)
            self._pending_blocked_snapshot = False
            self._last_blocked_retry_time = now
            self._blocked_burst_post_remaining = 1
            self._blocked_burst_next_time = float(now) + float(self._blocked_burst_post_sec)

        def _tick(self):
            now = time.time()

            with self._lock:
                frame = self._latest_frame
                frame_stamp = self._latest_stamp

            detect = self._last_detect
            reason = "cached"

            if frame is None:
                self._publish_hold(now, reason="no_frame")
                return

            if now - self._last_infer >= self._interval:
                self._last_infer = now
                detect = self._infer_front_entities(frame)
                self._last_detect = detect
                reason = str(detect.get("reason", "none"))

                present = bool(detect["present"])
                conf = float(detect["best_conf"])
                object_present = bool(detect["object_present"])

                if present:
                    self._human_until = max(self._human_until, now + max(0.0, float(args.hold_sec)))
                    self._last_conf = conf
                self._object_active = object_present
            else:
                reason = "throttled"

            if detect is None:
                detect = {
                    "present": False,
                    "best_conf": 0.0,
                    "object_present": False,
                    "reason": reason,
                    "person_count": 0,
                    "roi_person_count": 0,
                    "object_count": 0,
                    "detections": [],
                }

            self._update_stop_state_and_snapshot(now, frame, frame_stamp, detect)

            if self._blocked_burst_post_remaining > 0 and now >= self._blocked_burst_next_time and self._can_save_snapshot():
                post_detect = detect if detect is not None else {
                    "present": False,
                    "best_conf": 0.0,
                    "object_present": False,
                    "reason": "blocked_burst_post_no_detect",
                    "person_count": 0,
                    "roi_person_count": 0,
                    "object_count": 0,
                    "detections": [],
                }
                self._save_snapshot(
                    now,
                    frame.copy(),
                    float(frame_stamp),
                    post_detect,
                    extra_meta={
                        "snapshot_source": "blocked_burst_post",
                        "blocked_event_wall": float(self._blocked_event_wall),
                        "blocked_to_snapshot_delay_sec": float(now) - float(self._blocked_event_wall) if self._blocked_event_wall > 0 else None,
                    },
                )
                self._blocked_burst_post_remaining -= 1
                self._blocked_burst_next_time = float(now) + float(self._blocked_burst_post_sec)

            self._publish_hold(now, reason=reason, detect=detect)

        def _publish_hold(self, now, reason="none", detect=None):
            active = bool(now <= self._human_until)
            msg_b = Bool()
            msg_b.data = active
            self.pub_bool.publish(msg_b)

            msg_c = Float32()
            msg_c.data = float(self._last_conf if active else 0.0)
            self.pub_conf.publish(msg_c)

            if args.verbose and now - self._last_publish >= 1.0:
                self._last_publish = now
                frame_age = max(0.0, now - self._latest_stamp) if self._latest_stamp > 0 else -1.0
                demand_active = self._demand_active(now)
                moving_active = self._moving_active(now)
                best_conf = 0.0 if detect is None else float(detect.get("best_conf", 0.0))
                person_count = 0 if detect is None else int(detect.get("person_count", 0))
                roi_person_count = 0 if detect is None else int(detect.get("roi_person_count", 0))
                object_count = 0 if detect is None else int(detect.get("object_count", 0))
                projector_blocked = self._projector_blocked_active(now)
                self.get_logger().info(
                    " ".join(
                        [
                            f"human_active={int(active)}",
                            f"conf={msg_c.data:.2f}",
                            f"hold_left={max(0.0, self._human_until-now):.2f}s",
                            f"reason={reason}",
                            f"best_person_conf={best_conf:.2f}",
                            f"persons={person_count}",
                            f"roi_persons={roi_person_count}",
                            f"objects={object_count}",
                            f"frame_age={frame_age:.2f}s",
                            f"demand={int(demand_active)}",
                            f"moving={int(moving_active)}",
                            f"stop_active={int(self._stop_active)}",
                            f"projector_blocked={int(projector_blocked)}",
                            f"snapshots={self._snapshot_count}",
                        ]
                    )
                )

        def _demand_active(self, now):
            cmd_recent = (now - self._last_cmd_time) <= max(0.0, float(args.demand_hold_sec))
            cmd_mag = (abs(self._cmd_lin) >= float(args.stop_demand_linear_threshold)) or (
                abs(self._cmd_ang) >= float(args.stop_demand_angular_threshold)
            )
            return bool(cmd_recent and cmd_mag)

        def _moving_active(self, now):
            odom_recent = (now - self._last_odom_time) <= 1.0
            odom_mag = (abs(self._odom_lin) >= float(args.stop_moving_linear_threshold)) or (
                abs(self._odom_ang) >= float(args.stop_moving_angular_threshold)
            )
            return bool(odom_recent and odom_mag)

        def _update_stop_state_and_snapshot(self, now, frame, frame_stamp, detect):
            stop_candidate = self._demand_active(now) and (not self._moving_active(now))
            if stop_candidate:
                if self._stop_candidate_since is None:
                    self._stop_candidate_since = now
                if (not self._stop_active) and ((now - self._stop_candidate_since) >= float(args.stop_hold_sec)):
                    self._stop_active = True
            else:
                self._stop_candidate_since = None
                self._stop_active = False

            projector_blocked = self._projector_blocked_active(now)
            if args.snapshot_use_projector_blocked:
                blocked_gate = bool(projector_blocked)
            else:
                blocked_gate = bool(self._stop_active)

            force_snapshot = bool(args.snapshot_use_projector_blocked and self._pending_blocked_snapshot)
            roi_persons = int(detect.get("roi_person_count", 0))
            retry_no_human = bool(
                args.snapshot_use_projector_blocked
                and blocked_gate
                and (not args.snapshot_always)
                and (not force_snapshot)
                and roi_persons <= 0
                and self._blocked_retry_count < max(0, int(args.snapshot_retry_max_per_block))
                and (now - self._last_blocked_retry_time) >= max(0.0, float(args.snapshot_retry_no_human_sec))
            )

            snapshot_gate_active = bool(args.snapshot_always or blocked_gate or force_snapshot or retry_no_human)
            if not snapshot_gate_active:
                return
            if int(args.snapshot_max) > 0 and self._snapshot_count >= int(args.snapshot_max):
                return
            if (not force_snapshot) and (not retry_no_human) and float(args.snapshot_cooldown_sec) > 0 and (now - self._last_snapshot_time) < float(args.snapshot_cooldown_sec):
                return
            save_frame = frame
            save_stamp = frame_stamp
            save_detect = detect
            save_meta = {}
            if force_snapshot:
                if self._blocked_event_frame is not None:
                    save_frame = self._blocked_event_frame
                    save_stamp = float(self._blocked_event_stamp)
                    try:
                        save_detect = self._infer_front_entities(save_frame)
                    except Exception:
                        save_detect = detect
                    save_meta["snapshot_source"] = "blocked_edge_frame"
                else:
                    save_meta["snapshot_source"] = "blocked_edge_live_fallback"
                save_meta["blocked_event_wall"] = float(self._blocked_event_wall)
                save_meta["blocked_to_snapshot_delay_sec"] = max(0.0, float(now) - float(self._blocked_event_wall)) if self._blocked_event_wall > 0 else None

            if retry_no_human:
                save_meta.setdefault("snapshot_source", "blocked_retry_no_human")
                save_meta.setdefault("blocked_event_wall", float(self._blocked_event_wall))
                save_meta.setdefault(
                    "blocked_to_snapshot_delay_sec",
                    (float(now) - float(self._blocked_event_wall)) if self._blocked_event_wall > 0 else None,
                )
            elif blocked_gate and (not force_snapshot):
                save_meta.setdefault("snapshot_source", "blocked_gate_periodic")
                save_meta.setdefault("blocked_event_wall", float(self._blocked_event_wall))
                save_meta.setdefault(
                    "blocked_to_snapshot_delay_sec",
                    (float(now) - float(self._blocked_event_wall)) if self._blocked_event_wall > 0 else None,
                )

            self._save_snapshot(now, save_frame, save_stamp, save_detect, extra_meta=save_meta)
            if force_snapshot:
                self._pending_blocked_snapshot = False
                self._blocked_retry_count = 0
                self._last_blocked_retry_time = 0.0
            elif retry_no_human:
                self._blocked_retry_count += 1
                self._last_blocked_retry_time = now

        def _save_snapshot(self, now, frame, frame_stamp, detect, extra_meta=None):
            projector_blocked = self._projector_blocked_active(now)
            blocked_gate = projector_blocked if args.snapshot_use_projector_blocked else self._stop_active
            annotated = frame.copy()
            h, w = annotated.shape[:2]
            roi_w = min(1.0, max(0.1, float(args.front_roi_width)))
            x0 = int((0.5 - roi_w * 0.5) * w)
            x1 = int((0.5 + roi_w * 0.5) * w)
            cv2.rectangle(annotated, (x0, 0), (x1, h - 1), (0, 255, 255), 2)

            detections = detect.get("detections", [])
            for d in detections:
                bx0, by0, bx1, by1 = [int(v) for v in d.get("xyxy", [0, 0, 0, 0])]
                cls_id = int(d.get("cls_id", -1))
                conf = float(d.get("conf", 0.0))
                in_roi = bool(d.get("in_roi", False))
                is_person = cls_id == 0
                color = (0, 255, 0) if is_person else (0, 128, 255)
                if not in_roi:
                    color = (128, 128, 128)
                cv2.rectangle(annotated, (bx0, by0), (bx1, by1), color, 2)
                label = f"{'person' if is_person else 'obj'} c={conf:.2f} roi={int(in_roi)}"
                cv2.putText(annotated, label, (bx0, max(15, by0 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

            status = (
                f"stop_active={int(self._stop_active)} proj_blocked={int(projector_blocked)} gate={int(blocked_gate)} always={int(args.snapshot_always)} demand=({self._cmd_lin:.3f},{self._cmd_ang:.3f}) "
                f"odom=({self._odom_lin:.3f},{self._odom_ang:.3f}) human={int(detect.get('present', False))} "
                f"conf={float(detect.get('best_conf', 0.0)):.2f} reason={detect.get('reason', 'none')}"
            )
            cv2.putText(annotated, status, (10, max(20, h - 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)

            ts_ms = int(now * 1000)
            prefix = "always" if args.snapshot_always and (not blocked_gate) else "blocked"
            img_path = self._snapshot_dir / f"{prefix}_{ts_ms:013d}_{self._snapshot_count:04d}.jpg"
            json_path = self._snapshot_dir / f"{prefix}_{ts_ms:013d}_{self._snapshot_count:04d}.json"

            cv2.imwrite(str(img_path), annotated)
            payload = {
                "saved_wall": now,
                "frame_stamp": frame_stamp,
                "cmd": {"linear": self._cmd_lin, "angular": self._cmd_ang, "last_time": self._last_cmd_time},
                "odom": {"linear": self._odom_lin, "angular": self._odom_ang, "last_time": self._last_odom_time},
                "stop_active": self._stop_active,
                "projector_blocked_raw": bool(self._projector_blocked_raw),
                "projector_blocked_active": bool(projector_blocked),
                "snapshot_use_projector_blocked": bool(args.snapshot_use_projector_blocked),
                "snapshot_gate_blocked": bool(blocked_gate),
                "blocked_retry_count": int(self._blocked_retry_count),
                "snapshot_retry_no_human_sec": float(args.snapshot_retry_no_human_sec),
                "snapshot_retry_max_per_block": int(args.snapshot_retry_max_per_block),
                "snapshot_always": bool(args.snapshot_always),
                "detect": {
                    "present": bool(detect.get("present", False)),
                    "best_conf": float(detect.get("best_conf", 0.0)),
                    "reason": detect.get("reason", "none"),
                    "person_count": int(detect.get("person_count", 0)),
                    "roi_person_count": int(detect.get("roi_person_count", 0)),
                    "object_count": int(detect.get("object_count", 0)),
                    "detections": detect.get("detections", []),
                },
            }
            if extra_meta:
                payload.update(extra_meta)
            with json_path.open("w") as f:
                json.dump(payload, f, indent=2)

            self._last_snapshot_time = now
            self._snapshot_count += 1
            if args.verbose:
                self.get_logger().info(f"saved_snapshot image={img_path.name} meta={json_path.name}")

        def _infer_front_entities(self, frame):
            h, w = frame.shape[:2]
            roi_w = min(1.0, max(0.1, float(args.front_roi_width)))
            x0 = (0.5 - roi_w * 0.5) * w
            x1 = (0.5 + roi_w * 0.5) * w
            min_area = min(1.0, max(0.0, float(args.min_box_area))) * float(w * h)

            try:
                result = self.model.predict(
                    source=frame,
                    conf=min(float(args.conf), float(args.object_conf)),
                    imgsz=int(args.imgsz),
                    device=args.device if args.device else None,
                    verbose=False,
                )
            except Exception as ex:
                if args.verbose:
                    self.get_logger().warn(f"inference failed: {ex}")
                return {
                    "present": False,
                    "best_conf": 0.0,
                    "object_present": False,
                    "reason": "inference_failed",
                    "person_count": 0,
                    "roi_person_count": 0,
                    "object_count": 0,
                    "detections": [],
                }

            if not result:
                return {
                    "present": False,
                    "best_conf": 0.0,
                    "object_present": False,
                    "reason": "no_result",
                    "person_count": 0,
                    "roi_person_count": 0,
                    "object_count": 0,
                    "detections": [],
                }

            boxes = getattr(result[0], "boxes", None)
            if boxes is None:
                return {
                    "present": False,
                    "best_conf": 0.0,
                    "object_present": False,
                    "reason": "no_boxes",
                    "person_count": 0,
                    "roi_person_count": 0,
                    "object_count": 0,
                    "detections": [],
                }

            best = 0.0
            object_present = False
            detections = []
            person_count = 0
            roi_person_count = 0
            object_count = 0

            for box in boxes:
                try:
                    xyxy = box.xyxy[0].cpu().numpy().tolist()
                    conf = float(box.conf[0].item())
                    cls_id = int(box.cls[0].item())
                except Exception:
                    continue

                bx0, by0, bx1, by1 = xyxy
                bw = max(0.0, bx1 - bx0)
                bh = max(0.0, by1 - by0)
                area = bw * bh
                cx = 0.5 * (bx0 + bx1)
                area_ratio = area / float(max(1, w * h))
                area_ok = area >= min_area
                in_roi = bool(x0 <= cx <= x1)
                is_person = cls_id == 0

                if is_person:
                    person_count += 1
                elif conf >= float(args.object_conf):
                    object_count += 1

                detections.append(
                    {
                        "cls_id": cls_id,
                        "conf": conf,
                        "xyxy": [float(bx0), float(by0), float(bx1), float(by1)],
                        "area_ratio": area_ratio,
                        "in_roi": in_roi,
                        "area_ok": area_ok,
                    }
                )

                if not area_ok:
                    continue
                if not in_roi:
                    continue
                if is_person:
                    roi_person_count += 1
                    if conf > best:
                        best = conf
                else:
                    if conf >= float(args.object_conf):
                        object_present = True

            present = bool(best >= float(args.conf))
            if present:
                reason = "person_detected"
            elif person_count == 0:
                reason = "no_person_boxes"
            elif roi_person_count == 0:
                reason = "person_outside_roi_or_small"
            else:
                reason = "person_below_conf"

            return {
                "present": present,
                "best_conf": best,
                "object_present": object_present,
                "reason": reason,
                "person_count": person_count,
                "roi_person_count": roi_person_count,
                "object_count": object_count,
                "detections": detections,
            }

    rclpy.init(args=None)
    node = HumanDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
