import threading
import time
from typing import Optional, Tuple


class RosServoSource:
    """Background ROS2 subscriber for servo heading (std_msgs/Int16)."""

    def __init__(self, topic: str):
        self._topic = topic
        self._lock = threading.Lock()
        self._latest_angle_deg = 0.0
        self._latest_stamp_sec = 0.0
        self._stop_event = threading.Event()
        self._thread = None
        self._warned_unavailable = False

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target = self._run, name = "ros-servo-source", daemon = True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout = 1.5)
            self._thread = None

    def get_latest(self) -> Optional[Tuple[float, float]]:
        with self._lock:
            if self._latest_stamp_sec <= 0.0:
                return None
            return (self._latest_angle_deg, self._latest_stamp_sec)

    def _set_latest(self, angle_deg: float) -> None:
        with self._lock:
            self._latest_angle_deg = float(angle_deg)
            self._latest_stamp_sec = time.perf_counter()

    def _run(self) -> None:
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from std_msgs.msg import Int16
        except Exception as ex:
            if not self._warned_unavailable:
                print(f"[ros_servo_source] unavailable ({ex}); running without servo topic")
                self._warned_unavailable = True
            return

        source = self

        class ServoNode(Node):
            def __init__(self):
                super().__init__("display_servo_source")
                self.create_subscription(Int16, source._topic, self._on_servo, 20)

            def _on_servo(self, msg):
                source._set_latest(float(msg.data))

        node = None
        executor = None
        initialized_here = False
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
                initialized_here = True
            node = ServoNode()
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            while not self._stop_event.is_set() and rclpy.ok():
                executor.spin_once(timeout_sec=0.1)
        except Exception as ex:
            print(f"[ros_servo_source] stopped ({ex})")
        finally:
            try:
                if executor is not None and node is not None:
                    executor.remove_node(node)
            except Exception:
                pass
            try:
                if node is not None:
                    node.destroy_node()
            except Exception:
                pass
            try:
                if initialized_here and rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass
