#!/usr/bin/env python3

# Copyright (c) 2021  IBM Corporation and Carnegie Mellon University
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

import math
from abc import abstractmethod

import rclpy
import rclpy.time
import rclpy.node
from rclpy.duration import Duration
from visualization_msgs.msg import Marker, MarkerArray
from people_msgs.msg import People
from track_people_msgs.msg import TrackedBoxes

import numpy as np
from collections import deque
from scipy.linalg import block_diag
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

from matplotlib import pyplot as plt
from diagnostic_updater import Updater, HeaderlessTopicDiagnostic, FrequencyStatusParam
from tf_transformations import quaternion_from_euler


class PredictKfBuffer():
    def __init__(self):
        self.track_input_queue_dict = {}
        self.track_color_dict = {}

        self.track_id_kf_model_dict = {}
        self.track_id_missing_time_dict = {}



class PredictKfAbstract(rclpy.node.Node):
    def __init__(self, name, input_time, duration_inactive_to_remove, duration_inactive_to_stop_publish, fps_est_time):
        super().__init__(name)
        # settings for visualization
        self.vis_pred_image = False

        # start initialization
        self.input_time = input_time
        self.duration_inactive_to_remove = duration_inactive_to_remove
        self.duration_inactive_to_stop_publish = duration_inactive_to_stop_publish
        self.fps_est_time = fps_est_time

        # buffer to calculate FPS for each track, in multiple camera mode FPS might be different for each track
        self.track_predict_fps = {}
        self.track_prev_predict_timestamp = {}
        self.track_vel_hist_dict = {}

        # buffers to predict
        self.predict_buf = PredictKfBuffer()

        # set subscriber, publisher
        self.tracked_boxes_sub = self.create_subscription(TrackedBoxes, 'people/combined_detected_boxes', self.tracked_boxes_cb, 10)
        self.people_pub = self.create_publisher(People, 'people', 10)
        self.vis_marker_array_pub = self.create_publisher(MarkerArray, 'people/prediction_visualization', 10)

        # buffer to merge people tracking, prediction results before publish
        self.camera_id_predicted_tracks_dict = {}
        self.camera_id_people_dict = {}
        self.camera_id_vis_marker_array_dict = {}

        self.updater = Updater(self)
        target_fps = self.declare_parameter('target_fps', 10.0).value
        diagnostic_name = self.declare_parameter('diagnostic_name', "PeoplePredict").value
        self.htd = HeaderlessTopicDiagnostic(diagnostic_name, self.updater,
                                             FrequencyStatusParam({'min': target_fps, 'max': target_fps}, 0.2, 2))

        self.stationary_detect_threshold_duration_ = self.declare_parameter('stationary_detect_threshold_duration', 1.0).value

    @abstractmethod
    def pub_result(self, msg, alive_track_id_list, track_pos_dict, track_vel_dict, track_vel_hist_dict):
        pass

    @abstractmethod
    def on_tracked_boxes_cb(self, msg):
        pass

    def vis_result(self, msg, alive_track_id_list, track_pos_dict, track_vel_dict):
        # publish visualization marker array for rviz
        marker_array = MarkerArray()
        # plot sphere for current position, arrow for current direction
        for track_id in track_pos_dict.keys():
            marker = Marker()
            marker.header = msg.header
            marker.ns = "predict-origin-position"
            marker.id = track_id*2
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.lifetime = Duration(nanoseconds=1e8).to_msg()
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.1
            marker.pose.position.x = track_pos_dict[track_id][0]
            marker.pose.position.y = track_pos_dict[track_id][1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.color.r = self.predict_buf.track_color_dict[track_id][0]
            marker.color.g = self.predict_buf.track_color_dict[track_id][1]
            marker.color.b = self.predict_buf.track_color_dict[track_id][2]
            if track_id in alive_track_id_list:
                marker.color.a = 1.0
            else:
                marker.color.a = 0.5
            marker_array.markers.append(marker)

            track_velocity = track_vel_dict[track_id]
            velocity_norm = np.linalg.norm(np.array(track_velocity))
            velocity_orientation = math.atan2(track_velocity[1], track_velocity[0])
            velocity_orientation_quat = quaternion_from_euler(0.0, 0.0, velocity_orientation)
            marker = Marker()
            marker.header = msg.header
            marker.ns = "predict-origin-direction"
            marker.id = track_id*2+1
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.lifetime = Duration(nanoseconds=1e8).to_msg()
            marker.scale.x = 1.0 * min(velocity_norm, 1.0)
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.pose.position.x = track_pos_dict[track_id][0]
            marker.pose.position.y = track_pos_dict[track_id][1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = velocity_orientation_quat[0]
            marker.pose.orientation.y = velocity_orientation_quat[1]
            marker.pose.orientation.z = velocity_orientation_quat[2]
            marker.pose.orientation.w = velocity_orientation_quat[3]
            marker.color.r = self.predict_buf.track_color_dict[track_id][0]
            marker.color.g = self.predict_buf.track_color_dict[track_id][1]
            marker.color.b = self.predict_buf.track_color_dict[track_id][2]
            if track_id in alive_track_id_list:
                marker.color.a = 1.0
            else:
                marker.color.a = 0.5
            marker_array.markers.append(marker)

        self.vis_marker_array_pub.publish(marker_array)


    def tracked_boxes_cb(self, msg):
        self.htd.tick()
        self.on_tracked_boxes_cb(msg)

        # 2022.01.12: remove time check for multiple detection, instead check individual box
        # check message time order is correct
        # for _, tbox in enumerate(msg.tracked_boxes):
        #     if tbox.box.Class == "person":
        #         track_id = tbox.track_id
        #         if (track_id in self.track_prev_predict_timestamp) and (msg.header.stamp.to_sec()<self.track_prev_predict_timestamp[track_id][-1]):
        #             rospy.logwarn("skip wrong time order message. msg timestamp = " + str(msg.header.stamp.to_sec())
        #                 + "track_id = " + str(track_id) + ", previous time stamp for track = " + str(self.track_prev_predict_timestamp[track_id][-1]))
        #             return

        # update queue
        alive_track_id_list = []
        for _, tbox in enumerate(msg.tracked_boxes):
            if tbox.box.class_name == "person" or tbox.box.class_name == "obstacle":
                # update buffer to predict
                center3d = [tbox.center3d.x, tbox.center3d.y, tbox.center3d.z]
                track_id = tbox.track_id
                color = [tbox.color.r, tbox.color.g, tbox.color.b]
                if track_id not in self.predict_buf.track_input_queue_dict:
                    self.predict_buf.track_input_queue_dict[track_id] = deque(maxlen=self.input_time)
                self.predict_buf.track_input_queue_dict[track_id].append(center3d)
                self.predict_buf.track_color_dict[track_id] = color
                alive_track_id_list.append(track_id)

                # clear missing time
                if track_id in self.predict_buf.track_id_missing_time_dict:
                    del self.predict_buf.track_id_missing_time_dict[track_id]

                # update buffer for FPS
                if track_id in self.track_prev_predict_timestamp:
                    if rclpy.time.Time.from_msg(tbox.header.stamp) <= self.track_prev_predict_timestamp[track_id][-1]:
                        # rospy.logwarn("skip wrong time order box. box timestamp = " + str(tbox.header.stamp.to_sec())
                        #               + "track_id = " + str(track_id) + ", previous time stamp for track = " + str(self.track_prev_predict_timestamp[track_id][-1]))
                        continue
                    # calculate average FPS in past frames
                    self.track_predict_fps[track_id] = len(self.track_prev_predict_timestamp[track_id])/((rclpy.time.Time.from_msg(tbox.header.stamp)-self.track_prev_predict_timestamp[track_id][0]).nanoseconds/1e9)
                    # self.get_logger().info("track_id = " + str(track_id) + ", FPS = " + str(self.track_predict_fps[track_id]))
                if track_id not in self.track_prev_predict_timestamp:
                    self.track_prev_predict_timestamp[track_id] = deque(maxlen=self.fps_est_time)

                self.track_prev_predict_timestamp[track_id].append(rclpy.time.Time.from_msg(tbox.header.stamp))

        # predict
        track_pos_dict = {}
        track_vel_dict = {}
        for track_id in alive_track_id_list:
            if track_id not in self.predict_buf.track_id_kf_model_dict and len(self.predict_buf.track_input_queue_dict[track_id]) < self.input_time:
                continue

            past = np.array(self.predict_buf.track_input_queue_dict[track_id])[:, :2]

            if track_id not in self.predict_buf.track_id_kf_model_dict:
                tracker = KalmanFilter(dim_x=4, dim_z=2)
                dt = 1.   # time step 1 second
                tracker.F = np.array([[1, dt, 0,  0],
                                      [0,  1, 0,  0],
                                      [0,  0, 1, dt],
                                      [0,  0, 0,  1]])
                tracker.u = 0.
                tracker.H = np.array([[1, 0, 0, 0],
                                      [0, 0, 1, 0]])
                tracker.R = np.eye(2) * 5
                q = Q_discrete_white_noise(dim=2, dt=dt, var=0.05)
                tracker.Q = block_diag(q, q)
                tracker.x = np.array([[past[-1][0], 0, past[-1][1], 0]]).T
                tracker.P = np.eye(4) * 500.
            else:
                tracker = self.predict_buf.track_id_kf_model_dict[track_id]
                # get only last input to update KF
                past = past[-1:, :]

            # update model by inputting past history
            for _, px in enumerate(past):
                tracker.predict()
                tracker.update(px)
            self.predict_buf.track_id_kf_model_dict[track_id] = tracker

            # save position and velocity
            # use raw position
            track_pos_dict[track_id] = past[-1]
            # ues filtered position
            # track_pos_dict[track_id] = tracker.x.reshape(1,4)[0, [0,2]]
            track_vel_dict[track_id] = tracker.x.reshape(1, 4)[0, [1, 3]] * self.track_predict_fps[track_id]
            if track_id not in self.track_vel_hist_dict:
                self.track_vel_hist_dict[track_id] = deque(maxlen=self.fps_est_time)
            self.track_vel_hist_dict[track_id].append((self.track_prev_predict_timestamp[track_id][-1], track_vel_dict[track_id]))

        # clean up missed track if necessary
        missing_track_id_list = set(self.predict_buf.track_input_queue_dict.keys()) - set(alive_track_id_list)
        stop_publish_track_id_list = set()
        now = self.get_clock().now()
        for track_id in missing_track_id_list:
            # update missing time
            if track_id not in self.predict_buf.track_id_missing_time_dict:
                self.predict_buf.track_id_missing_time_dict[track_id] = now

            # if missing long time, stop publishing in people topic
            if (now - self.predict_buf.track_id_missing_time_dict[track_id]).nanoseconds/1000000000 > self.duration_inactive_to_stop_publish:
                stop_publish_track_id_list.add(track_id)

            # if missing long time, delete track
            if (now - self.predict_buf.track_id_missing_time_dict[track_id]).nanoseconds/1000000000 > self.duration_inactive_to_remove:
                if track_id in self.track_predict_fps:
                    del self.track_predict_fps[track_id]
                if track_id in self.track_prev_predict_timestamp:
                    del self.track_prev_predict_timestamp[track_id]

                del self.predict_buf.track_input_queue_dict[track_id]
                del self.predict_buf.track_color_dict[track_id]
                del self.predict_buf.track_id_missing_time_dict[track_id]
                if track_id in self.predict_buf.track_id_kf_model_dict:
                    del self.predict_buf.track_id_kf_model_dict[track_id]

        # predict track which is missing, but not deleted yet
        publish_missing_track_id_list = missing_track_id_list - stop_publish_track_id_list
        for track_id in publish_missing_track_id_list:
            if track_id not in self.predict_buf.track_id_kf_model_dict:
                continue
            tracker = self.predict_buf.track_id_kf_model_dict[track_id]

            last_vel = tracker.x.reshape(1, 4)[0, [1, 3]] * self.track_predict_fps[track_id]

            # save position and velocity
            # use raw position
            track_pos_dict[track_id] = np.array(self.predict_buf.track_input_queue_dict[track_id])[-1, :2]
            # ues filtered position
            # track_pos_dict[track_id] = tracker.x.reshape(1,4)[0, [0,2]]
            track_vel_dict[track_id] = last_vel

        self.pub_result(msg, alive_track_id_list, track_pos_dict, track_vel_dict, self.track_vel_hist_dict)

        self.vis_result(msg, alive_track_id_list, track_pos_dict, track_vel_dict)
