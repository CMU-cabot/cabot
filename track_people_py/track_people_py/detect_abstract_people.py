# Copyright (c) 2021, 2022  IBM Corporation and Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from abc import ABCMeta, abstractmethod
import time
import queue
import threading

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from matplotlib import pyplot as plt
from message_filters import ApproximateTimeSynchronizer
import message_filters
import numpy as np

import rclpy
import rclpy.node
from rclpy.duration import Duration
from rclpy.time import Time

from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import ColorRGBA
from std_srvs.srv import SetBool

from rclpy.qos import qos_profile_sensor_data

import tf2_ros
from track_people_msgs.msg import BoundingBox, TrackedBox, TrackedBoxes

from .pointcloud_utils import open3d_utils


class AbsDetectPeople(rclpy.node.Node):
    __metaclass__ = ABCMeta

    def __init__(self, device):
        # start initialization
        # rospy.init_node('detect_people_py', anonymous=True)
        super().__init__('detect_people_py')

        self.device = device

        # constant parameters
        self.lookup_transform_duration = 1
        self.vis_detect_image = False

        # load detect model
        self.detection_threshold = self.declare_parameter('detection_threshold', 0.25).value
        self.minimum_detection_size_threshold = self.declare_parameter('minimum_detection_size_threshold', 50.0).value

        self.map_frame_name = self.declare_parameter('map_frame', 'map').value
        self.camera_id = self.declare_parameter('camera_id', 'camera').value
        self.camera_link_frame_name = self.declare_parameter('camera_link_frame', 'camera_link').value
        self.camera_info_topic_name = self.declare_parameter('camera_info_topic', 'color/camera_info').value
        self.image_rect_topic_name = self.declare_parameter('image_rect_topic', 'color/image_raw').value
        self.depth_registered_topic_name = self.declare_parameter('depth_registered_topic', 'aligned_depth_to_color/image_raw').value
        self.depth_unit_meter = self.declare_parameter('depth_unit_meter', False).value
        self.target_fps = self.declare_parameter('target_fps', 15.0).value

        self.enable_detect_people = True
        self.toggle_srv = self.create_service(SetBool, 'enable_detect_people', self.enable_detect_people_cb)

        self.get_logger().info("Waiting for camera_info topic: {}".format(self.camera_info_topic_name))
        # wait_for_message is not available on galactic or humble but rolling
        # rclpy.wait_for_message(CameraInfo, self, self.camera_info_topic_name)

        self.get_logger().info("Found camera_info topic.")
        self.camera_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic_name, self.camera_info_cb, 10)

        self.bridge = CvBridge()
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer, self)

        self.rgb_image_sub = message_filters.Subscriber(self, Image, self.image_rect_topic_name, qos_profile=qos_profile_sensor_data)
        self.depth_image_sub = message_filters.Subscriber(self, Image, self.depth_registered_topic_name, qos_profile=qos_profile_sensor_data)
        self.rgb_depth_img_synch = ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub], queue_size=10, slop=1.0/self.target_fps)
        self.rgb_depth_img_synch.registerCallback(self.rgb_depth_img_cb)
        self.detected_boxes_pub = self.create_publisher(TrackedBoxes, '/people/detected_boxes', 1)

        self.prev_img_time_sec = 0

        self.get_logger().info("set timer, %.2f" % (1.0 / self.target_fps))
        self.timer = self.create_timer(1.0/self.target_fps, self.fps_callback)
        self.current_input = None

        self.pipeline1_input = queue.Queue(maxsize=1)
        self.pipeline2_input = queue.Queue(maxsize=1)
        self.pipeline1_thread = threading.Thread(target=self.pipeline1_run)
        self.pipeline2_thread = threading.Thread(target=self.pipeline2_run)
        self.pipeline1_thread.start()
        self.pipeline2_thread.start()

    def enable_detect_people_cb(self, data):
        self.enable_detect_people = data.data

        resp = SetBool.Response()
        if self.enable_detect_people:
            resp.message = "detect people enabled"
        else:
            resp.message = "detect people disabled"
        resp.success = True
        return resp

    def camera_info_cb(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height
        self.focal_length = msg.k[0]
        self.center_x = msg.k[2]
        self.center_y = msg.k[5]
        self.camera_info_sub = None

    @abstractmethod
    def is_detector_initialized(self):
        pass

    def rgb_depth_img_cb(self, rgb_img_msg, depth_img_msg):
        # self.get_logger().info("rgb_depth_img_cb")
        # check if detector is enabled and initialized
        if not self.enable_detect_people or not self.is_detector_initialized():
            return

        # check if image is received in correct time order
        cur_img_time_sec = rgb_img_msg.header.stamp.sec + rgb_img_msg.header.stamp.nanosec / 1000000000
        if cur_img_time_sec < self.prev_img_time_sec:
            return
        self.prev_img_time_sec = cur_img_time_sec
        # check if image is received faster than target FPS
        # cur_prev_time_diff = abs(cur_img_time_sec-self.prev_img_time_sec)
        # if cur_prev_time_diff<1.0/self.target_fps:
        #     return

        try:
            input_pose = self.get_camera_link_pose(Time.from_msg(rgb_img_msg.header.stamp))
        except RuntimeError as e:
            self.get_logger().error(str(e))
            return

        self.current_input = (input_pose, rgb_img_msg, depth_img_msg)

    def fps_callback(self):
        # self.get_logger().info("fps_callback")
        if not self.current_input:
            self.get_logger().warn(F"no image incoming in target frame rate {self.target_fps}", throttle_duration_sec=1)
            return

        (input_pose, rgb_img_msg, depth_img_msg) = self.current_input
        self.current_input = None

        try:
            # notice : openCV default is BGR, but use RGB for Re-identification model
            input_rgb_image = self.bridge.imgmsg_to_cv2(rgb_img_msg, "rgb8")
            input_depth_image = self.bridge.imgmsg_to_cv2(depth_img_msg, "32FC1")
        except CvBridgeError as e:
            self.get_logger().error(e)
            return

        (frame_resized, native_image) = self.prepare_image(input_rgb_image)

        try:
            self.pipeline1_input.put_nowait((input_pose, rgb_img_msg, depth_img_msg, input_rgb_image, input_depth_image, frame_resized, native_image))
        except:  # noqa: E722
            # self.get_logger().error("failed to put_nowait (pipeline1)")
            pass

    """
    only detecting by darknet
    """

    def pipeline1_run(self):
        while rclpy.ok():
            try:
                (input_pose, rgb_img_msg, depth_img_msg, input_rgb_image, input_depth_image, frame_resized, native_image) = self.pipeline1_input.get_nowait()
            except:  # noqa: E722
                time.sleep(0.01)
                # self.get_logger().error("failed to get_nowait (pipeline1)")
                continue

            boxes_res = self.detect_people(input_rgb_image, frame_resized, native_image)

            self.pipeline2_input.put((input_pose, rgb_img_msg, depth_img_msg, input_rgb_image, input_depth_image, frame_resized, boxes_res))

    def get_camera_link_pose(self, time=None):
        try:
            if time:
                target = time
            else:
                target = self.get_clock().now()
            t = self.tf2_buffer.lookup_transform(self.map_frame_name, self.camera_link_frame_name, target, Duration(seconds=self.lookup_transform_duration))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            now = self.get_clock().now()
            # self.get_logger().error("failed to get_nowait (pipeline2)")
            raise RuntimeError('Cannot fetch the transform from {0:s} to {1:s},  target time {2:s}, current time {3:s}, time difference {4:s}'
                               .format(self.map_frame_name, self.camera_link_frame_name, str(target.nanoseconds/1e9), str(now.nanoseconds/1e9),
                                       str((now-target).nanoseconds/1e9)))

        camera_link_pose = PoseStamped()
        camera_link_pose.header.stamp = t.header.stamp
        camera_link_pose.pose.position.x = t.transform.translation.x
        camera_link_pose.pose.position.y = t.transform.translation.y
        camera_link_pose.pose.position.z = t.transform.translation.z
        camera_link_pose.pose.orientation.x = t.transform.rotation.x
        camera_link_pose.pose.orientation.y = t.transform.rotation.y
        camera_link_pose.pose.orientation.z = t.transform.rotation.z
        camera_link_pose.pose.orientation.w = t.transform.rotation.w
        return camera_link_pose

    def pose2transform(self, pose):
        # convert Pose to transformation matrix
        M = np.identity(4)
        M[:3, :3] = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]).as_matrix()
        M[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        return M

    @abstractmethod
    def detect_people(self, rgb_img, frame_resized, darknet_image):
        pass

    def post_process(self, rgb_img, frame_resized, boxes_res):
        pass

    """
    process depth
    """

    def pipeline2_run(self):
        while rclpy.ok():
            self._pipeline2_run()

    def _pipeline2_run(self):
        try:
            (input_pose, rgb_img_msg, depth_img_msg, input_rgb_image, input_depth_image, frame_resized, boxes_res) = self.pipeline2_input.get_nowait()
        except:  # noqa: E722
            time.sleep(0.01)
            return

        detect_results = self.post_process(input_depth_image, frame_resized, boxes_res)

        input_pose_transform = self.pose2transform(input_pose.pose)

        if len(detect_results) > 0:
            # delete small detections
            small_detection = np.where(detect_results[:, 3]-detect_results[:, 1] < self.minimum_detection_size_threshold)[0]
            detect_results = np.delete(detect_results, small_detection, axis=0)

        center3d_list = []
        center_bird_eye_global_list = []
        if len(detect_results) > 0:
            # start_time = time.time()
            invalid_detect_list = []
            for detect_idx, detect_result in enumerate(detect_results):
                box_xtl = int(detect_result[0])
                box_ytl = int(detect_result[1])
                box_xbr = int(detect_result[2])
                box_ybr = int(detect_result[3])
                box_width = box_xbr - box_xtl
                box_height = box_ybr - box_ytl
                box_center_xtl = box_xtl + int(box_width/4)
                box_center_ytl = box_ytl + int(box_height/4)
                box_center_xbr = box_xtl + int(box_width/4)*3
                box_center_ybr = box_ytl + int(box_height/4)*3
                # self.get_logger().info("[box] detect_idx = " + str(detect_idx) + ", xtl = " + str(box_xtl) + ", ytl = " + str(box_ytl) + ", xbr = " + str(box_xbr) + ", ybr = " + str(box_ybr))

                # create point cloud for box
                box_depth = np.zeros(input_depth_image.shape, input_depth_image.dtype)
                box_depth[box_center_ytl:box_center_ybr, box_center_xtl:box_center_xbr] = input_depth_image[box_center_ytl:box_center_ybr, box_center_xtl:box_center_xbr]
                box_depth[np.isnan(box_depth)] = 0
                box_depth[np.isinf(box_depth)] = 0
                box_points, box_colors = open3d_utils.generate_pointcloud(self.image_width, self.image_height, self.focal_length,
                                                                          self.center_x, self.center_y, input_rgb_image, box_depth,
                                                                          depth_unit_meter=self.depth_unit_meter)
                # self.get_logger().info("[box] depth for box region = " + str(box_depth[box_ytl:box_ybr, box_xtl:box_xbr]))
                if len(box_points) == 0:
                    self.get_logger().info("Cannot find point cloud for box " + str(detect_idx))
                    if detect_idx not in invalid_detect_list:
                        invalid_detect_list.append(detect_idx)
                else:
                    # obtain center in point cloud
                    box_center = np.median(np.asarray(box_points), axis=0)
                    if np.isnan(box_center).any() or np.isinf(box_center).any():
                        self.get_logger().info("Box center has invalid value for box " + str(detect_idx))
                        if detect_idx not in invalid_detect_list:
                            invalid_detect_list.append(detect_idx)
                    else:
                        # self.get_logger().info("[box] center = " + str(box_center))
                        # convert coordinate from x-right,y-down,z-forward coordinate to x-forward,y-left,z-up coordinate
                        ros_box_center = np.empty(box_center.shape)
                        ros_box_center[0] = box_center[2]
                        ros_box_center[1] = -box_center[0]
                        ros_box_center[2] = -box_center[1]
                        center3d_list.append(ros_box_center)

                        # convert coordinate from x-right,y-down,z-forward coordinate to x-forward,y-left,z-up coordinate
                        center_pose_local = PoseStamped()
                        center_pose_local.header.stamp = rgb_img_msg.header.stamp
                        center_pose_local.header.frame_id = self.camera_link_frame_name
                        center_pose_local.pose.position.x = box_center[2]
                        center_pose_local.pose.position.y = -box_center[0]
                        center_pose_local.pose.position.z = 0.0
                        center_pose_local.pose.orientation.x = 0.0
                        center_pose_local.pose.orientation.y = 0.0
                        center_pose_local.pose.orientation.z = 0.0
                        center_pose_local.pose.orientation.w = 1.0

                        # convert local pose to global pose
                        center_pose_local_transform = self.pose2transform(center_pose_local.pose)
                        center_pose_global_transform = np.dot(input_pose_transform, center_pose_local_transform)
                        center_pose_global = np.dot(center_pose_global_transform, np.array([0, 0, 0, 1]))
                        center_bird_eye_global_list.append(center_pose_global[:3])

            detect_results = np.delete(detect_results, invalid_detect_list, axis=0)
            if len(detect_results) != len(center3d_list):
                raise RuntimeError("Error : number of detect and number of center 3D should be same.")
            # elapsed_time = time.time() - start_time
            # self.get_logger().info("time for calculating centr 3D :{0}".format(elapsed_time) + "[sec]")

        # self.get_logger().info("camera ID = " + self.camera_id + ", number of detected people = " + str(len(detect_results)))
        # return detect_results, center3d_list, center_bird_eye_global_list
        self.pub_result(rgb_img_msg, input_pose, detect_results, center_bird_eye_global_list)
        self.vis_result(input_rgb_image, detect_results)

    def pub_result(self, img_msg, pose, detect_results, center3d_list):
        # publish tracked boxes message
        detected_boxes_msg = TrackedBoxes()
        detected_boxes_msg.header.stamp = img_msg.header.stamp
        detected_boxes_msg.header.frame_id = self.map_frame_name
        detected_boxes_msg.camera_id = self.camera_id
        detected_boxes_msg.pose = pose.pose
        for idx_bbox, bbox in enumerate(detect_results):
            detected_box = TrackedBox()
            detected_box.header.stamp = img_msg.header.stamp
            detected_box.header.frame_id = self.map_frame_name
            detected_box.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.0)
            detected_box.box = BoundingBox()
            detected_box.box.class_name = "person"
            detected_box.box.xmin = int(bbox[0])
            detected_box.box.ymin = int(bbox[1])
            detected_box.box.xmax = int(bbox[2])
            detected_box.box.ymax = int(bbox[3])
            detected_box.box.probability = bbox[4]
            detected_box.center3d.x = center3d_list[idx_bbox][0]
            detected_box.center3d.y = center3d_list[idx_bbox][1]
            detected_box.center3d.z = center3d_list[idx_bbox][2]
            detected_boxes_msg.tracked_boxes.append(detected_box)
        self.detected_boxes_pub.publish(detected_boxes_msg)

    def vis_result(self, input_rgb_image, detect_results):
        # visualization result in image
        if self.vis_detect_image:
            vis_rgb_image = input_rgb_image.copy()
            for idx_bbox, bbox in enumerate(detect_results):
                xmin, ymin, xmax, ymax = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
                cv2.rectangle(vis_rgb_image, (xmin, ymin), (xmax, ymax), color=(255, 0, 0), thickness=2)

            plt.figure(1)
            plt.cla()
            ax = plt.gca()
            ax.set_title("detected people in image, camera="+self.camera_id)
            ax.grid(False)
            plt.imshow(cv2.resize(vis_rgb_image, None, fx=0.5, fy=0.5))
            plt.draw()
            plt.pause(0.00000000001)
