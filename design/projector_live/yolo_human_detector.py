#!/usr/bin/env python3
import argparse
import threading
import time
from pathlib import Path


def parse_args():
    here = Path(__file__).resolve().parent
    p = argparse.ArgumentParser(description="YOLO person-in-front detector publisher for projector negotiation")
    p.add_argument("--model", default=str(here / "yolo26n.pt"), help="Path to YOLO model file")
    p.add_argument("--image-topic", default="/camera/color/image_raw", help="sensor_msgs/Image topic")
    p.add_argument("--out-topic", default="/projector/human_in_front", help="Output std_msgs/Bool topic")
    p.add_argument("--max-fps", type=float, default=5.0, help="Maximum inference FPS")
    p.add_argument("--conf", type=float, default=0.35, help="Minimum person confidence")
    p.add_argument("--object-conf", type=float, default=0.25, help="Minimum non-person object confidence")
    p.add_argument("--imgsz", type=int, default=640, help="Inference image size")
    p.add_argument("--device", default="", help="YOLO device string, e.g. cpu, cuda:0")
    p.add_argument("--front-roi-width", type=float, default=0.65, help="Center ROI width ratio [0..1]")
    p.add_argument("--min-box-area", type=float, default=0.010, help="Minimum bbox area ratio [0..1]")
    p.add_argument("--hold-sec", type=float, default=0.6, help="Hold positive detection for this many seconds")
    p.add_argument("--verbose", action="store_true", help="Print detector status")
    return p.parse_args()


def main():
    args = parse_args()

    try:
        # Import YOLO/Torch first on ARM to avoid OpenMP TLS conflicts
        # that can occur when OpenCV stack is loaded first.
        from ultralytics import YOLO
        from cv_bridge import CvBridge
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
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

            self._lock = threading.Lock()
            self._latest_frame = None
            self._latest_stamp = 0.0
            self._last_infer = 0.0
            self._last_publish = 0.0
            self._human_until = 0.0
            self._last_conf = 0.0
            self._interval = 1.0 / max(0.5, float(args.max_fps))
            self._object_active = False

            self.timer = self.create_timer(0.02, self._tick)
            self.get_logger().info(
                f"model={args.model} image={args.image_topic} out={args.out_topic} max_fps={args.max_fps}"
            )

        def _on_image(self, msg):
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception:
                return
            stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            if stamp <= 0:
                stamp = time.time()
            with self._lock:
                self._latest_frame = frame
                self._latest_stamp = stamp

        def _tick(self):
            now = time.time()
            if now - self._last_infer < self._interval:
                self._publish_hold(now)
                return

            with self._lock:
                frame = self._latest_frame
            if frame is None:
                self._publish_hold(now)
                return

            self._last_infer = now
            present, conf, object_present = self._infer_front_entities(frame)
            if present:
                self._human_until = max(self._human_until, now + max(0.0, float(args.hold_sec)))
                self._last_conf = conf
            if object_present and not self._object_active:
                print("object", flush=True)
            self._object_active = bool(object_present)

            self._publish_hold(now)

        def _publish_hold(self, now):
            active = bool(now <= self._human_until)
            msg_b = Bool()
            msg_b.data = active
            self.pub_bool.publish(msg_b)

            msg_c = Float32()
            msg_c.data = float(self._last_conf if active else 0.0)
            self.pub_conf.publish(msg_c)

            if args.verbose and now - self._last_publish >= 1.0:
                self._last_publish = now
                self.get_logger().info(
                    f"human_active={int(active)} conf={msg_c.data:.2f} hold_left={max(0.0, self._human_until-now):.2f}s"
                )

        def _infer_front_entities(self, frame):
            h, w = frame.shape[:2]
            roi_w = min(1.0, max(0.1, float(args.front_roi_width)))
            x0 = (0.5 - roi_w * 0.5) * w
            x1 = (0.5 + roi_w * 0.5) * w
            min_area = min(1.0, max(0.0, float(args.min_box_area))) * float(w * h)

            try:
                result = self.model.predict(
                    source=frame,
                    conf=float(args.conf),
                    imgsz=int(args.imgsz),
                    device=args.device if args.device else None,
                    verbose=False,
                )
            except Exception as ex:
                if args.verbose:
                    self.get_logger().warn(f"inference failed: {ex}")
                return False, 0.0, False

            best = 0.0
            object_present = False
            if not result:
                return False, 0.0, False
            boxes = getattr(result[0], "boxes", None)
            if boxes is None:
                return False, 0.0, False

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
                if area < min_area:
                    continue
                if not (x0 <= cx <= x1):
                    continue
                if cls_id == 0:
                    if conf > best:
                        best = conf
                else:
                    if conf >= float(args.object_conf):
                        object_present = True

            return (best >= float(args.conf)), best, object_present

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
