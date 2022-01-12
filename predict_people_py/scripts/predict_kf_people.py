#!/usr/bin/env python3

# Copyright (c) 2021  IBM Corporation
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

import copy
import math
import os
import sys

import rospy
import tf
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Point
from people_msgs.msg import People, Person
from std_msgs.msg import ColorRGBA
from track_people_py.msg import TrackedBox, TrackedBoxes

import numpy as np
from collections import deque
from scipy.linalg import block_diag
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from matplotlib import pyplot as plt


class PredictKfPeopleBuffer():
    def __init__(self, input_time, output_time, n_frames_inactive_to_remove):
        self.n_frames_inactive_to_remove = n_frames_inactive_to_remove
        self.track_input_queue_dict = {}
        self.track_color_dict = {}
        
        self.track_id_kf_model_dict = {}
        self.track_id_missing_time_dict = {}


class PredictKfPeople():
    def __init__(self, input_time, output_time, n_frames_inactive_to_remove, n_frames_inactive_to_stop_publish, fps_est_time, publish_simulator_people):
        # settings for visualization
        self.vis_pred_image = False
        
        # start initialization
        self.input_time = input_time
        self.output_time = output_time
        self.n_frames_inactive_to_remove = n_frames_inactive_to_remove
        self.n_frames_inactive_to_stop_publish = n_frames_inactive_to_stop_publish
        self.fps_est_time = fps_est_time
        self.publish_simulator_people = publish_simulator_people

        # buffer to calculate FPS for each track, in multiple camera mode FPS might be different for each track
        self.track_predict_fps = {}
        self.track_prev_predict_timestamp = {}
        
        # buffers to predict
        self.predict_buf = PredictKfPeopleBuffer(self.input_time, self.output_time, self.n_frames_inactive_to_remove)
        
        # set subscriber, publisher
        self.tracked_boxes_sub = rospy.Subscriber('track_people_py/tracked_boxes', TrackedBoxes, self.tracked_boxes_cb, queue_size=1)
        self.people_pub = rospy.Publisher('people', People, queue_size=1)
        self.vis_marker_array_pub = rospy.Publisher('predict_people_py/visualization_marker_array', MarkerArray, queue_size=1)
        if self.publish_simulator_people:
            self.simulator_people = None
            self.simulator_people_sub = rospy.Subscriber('people_simulator/people', People, self.simulator_people_cb, queue_size=1)

        # buffer to merge people tracking, prediction results before publish
        self.camera_id_predicted_tracks_dict = {}
        self.camera_id_people_dict = {}
        self.camera_id_vis_marker_array_dict = {}
    
    
    def simulator_people_cb(self, msg):
        self.simulator_people = msg.people
    
    
    def pub_simulator_people(self, msg):
        if self.simulator_people is None:
            return
        simulator_people_msg = People()
        simulator_people_msg.header = msg.header
        simulator_people_msg.people = self.simulator_people
        self.people_pub.publish(simulator_people_msg)
    
    
    def pub_result(self, msg, alive_track_id_list, track_pos_dict, track_vel_dict):
        # init People message
        people_msg = People()
        people_msg.header = msg.header
        
        for track_id in track_pos_dict.keys():
            past_center3d = Point()
            past_center3d.x = track_pos_dict[track_id][0]
            past_center3d.y = track_pos_dict[track_id][1]
            past_center3d.z = 0.0

            # create Person message
            person = Person()
            person.name = str(track_id)
            person.position = past_center3d
            person.velocity = Point()
            person.velocity.x = track_vel_dict[track_id][0]
            person.velocity.y = track_vel_dict[track_id][1]
            person.velocity.z = 0.0
            if track_id in alive_track_id_list:
                person.reliability = 1.0
            else:
                person.reliability = 0.9
            people_msg.people.append(person)
        
        # merge people from multiple camera before publish
        self.camera_id_people_dict[msg.camera_id] = copy.copy(people_msg.people)
        for camera_id in self.camera_id_people_dict.keys():
            if camera_id!=msg.camera_id and len(self.camera_id_people_dict[camera_id])>0:
                people_msg.people.extend(self.camera_id_people_dict[camera_id])
        self.people_pub.publish(people_msg)
    
    
    def vis_result(self, msg, alive_track_id_list, track_pos_dict, track_vel_dict):
        input_pose = msg.pose
        
        # publish visualization marker array for rviz
        marker_array = MarkerArray()
        # plot sphere for current position, arrow for current direction
        for track_id in track_pos_dict.keys():
            marker = Marker()
            marker.header = msg.header
            marker.ns = "predict-origin-position"
            marker.id = track_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(0.1)
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.1
            marker.pose.position.x = track_pos_dict[track_id][0]
            marker.pose.position.y = track_pos_dict[track_id][1]
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1
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
            velocity_orientation_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, velocity_orientation)
            marker = Marker()
            marker.header = msg.header
            marker.ns = "predict-origin-direction"
            marker.id = track_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(0.1)
            marker.scale.x = 1.0 * min(velocity_norm, 1.0)
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.pose.position.x = track_pos_dict[track_id][0]
            marker.pose.position.y = track_pos_dict[track_id][1]
            marker.pose.position.z = 0
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

        # merge marker array from multiple camera before publish
        self.camera_id_vis_marker_array_dict[msg.camera_id] = copy.copy(marker_array.markers)
        for camera_id in self.camera_id_vis_marker_array_dict.keys():
            if camera_id!=msg.camera_id and len(self.camera_id_vis_marker_array_dict[camera_id])>0:
                marker_array.markers.extend(self.camera_id_vis_marker_array_dict[camera_id])
        self.vis_marker_array_pub.publish(marker_array)
        
        if self.vis_pred_image:
            # prepare plot
            plt.figure(1)
            plt.cla()
            ax = plt.gca()
            ax.set_title("predict people in global")
            ax.grid(True)
            ax.legend()
            ax.set_xlabel('y')
            ax.set_ylabel('x')
            
            # plot prediction
            plt_x = []
            plt_y = []
            plt_color = []
            for track_id in track_pred_dict.keys():
                track_predict = track_pred_dict[track_id]
                for row_idx, row in enumerate(track_predict):
                    plt_x.append(-track_predict[row_idx][1])
                    plt_y.append(track_predict[row_idx][0])
                    plt_color.append(np.array(self.predict_buf.track_color_dict[track_id]))
            plt.scatter(plt_x, plt_y, c=plt_color, marker='o')
            
            # plot prediction origin
            plt_x = []
            plt_y = []
            plt_color = []
            for track_id in track_pred_dict.keys():
                if len(self.predict_buf.track_input_queue_dict[track_id]) < self.input_time:
                    continue
                
                past = np.array(self.predict_buf.track_input_queue_dict[track_id])[:,:2]
                predict_origin = past[-1,:].copy()
                plt_x.append(-track_predict[row_idx][1])
                plt_y.append(track_predict[row_idx][0])
                plt_color.append(np.array(self.predict_buf.track_color_dict[track_id]))
            plt.scatter(plt_x, plt_y, c=plt_color, marker='s')
            
            plt.scatter([-input_pose.position.y], [input_pose.position.x], c=[np.array([1.0, 0.0, 0.0])], marker='+')
            ax.set_xlim([-input_pose.position.y-20,-input_pose.position.y+20])
            ax.set_ylim([input_pose.position.x-20,input_pose.position.x+20])
            plt.draw()
            plt.pause(0.00000000001)
    
    
    def tracked_boxes_cb(self, msg):
        if self.publish_simulator_people:
            self.pub_simulator_people(msg)
            return

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
            if tbox.box.Class == "person":
                # update buffer to predict
                center3d = [tbox.center3d.x, tbox.center3d.y, tbox.center3d.z]
                track_id = tbox.track_id
                color = [tbox.color.r, tbox.color.g, tbox.color.b]
                if track_id not in self.predict_buf.track_input_queue_dict:
                    self.predict_buf.track_input_queue_dict[track_id] = deque(maxlen=self.input_time)
                self.predict_buf.track_input_queue_dict[track_id].append(center3d)
                self.predict_buf.track_color_dict[track_id] = color
                alive_track_id_list.append(track_id)

                # clear missing counter
                if track_id in self.predict_buf.track_id_missing_time_dict:
                    del self.predict_buf.track_id_missing_time_dict[track_id]
                
                # update buffer for FPS
                if track_id in self.track_prev_predict_timestamp:
                    if tbox.header.stamp.to_sec() < self.track_prev_predict_timestamp[track_id][-1]:
                        # rospy.logwarn("skip wrong time order box. box timestamp = " + str(tbox.header.stamp.to_sec())
                        #               + "track_id = " + str(track_id) + ", previous time stamp for track = " + str(self.track_prev_predict_timestamp[track_id][-1]))
                        continue
                    # calculate average FPS in past frames
                    self.track_predict_fps[track_id] = len(self.track_prev_predict_timestamp[track_id])/(msg.header.stamp.to_sec()-self.track_prev_predict_timestamp[track_id][0])
                    # rospy.loginfo("track_id = " + str(track_id) + ", FPS = " + str(self.track_predict_fps[track_id]))
                if track_id not in self.track_prev_predict_timestamp:
                    self.track_prev_predict_timestamp[track_id] = deque(maxlen=self.fps_est_time)
                self.track_prev_predict_timestamp[track_id].append(msg.header.stamp.to_sec())
        
        # predict
        track_pos_dict = {}
        track_vel_dict = {}
        for track_id in alive_track_id_list:
            if track_id not in self.predict_buf.track_id_kf_model_dict and len(self.predict_buf.track_input_queue_dict[track_id]) < self.input_time:
                continue
            
            past = np.array(self.predict_buf.track_input_queue_dict[track_id])[:,:2]
            
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
                tracker.x = np.array([[0, 0, 0, 0]]).T
                tracker.P = np.eye(4) * 500.
            else:
                tracker = self.predict_buf.track_id_kf_model_dict[track_id]
                # get only last input to update KF
                past = past[-1:,:]
            
            # update model by inputting past history
            for _, px in enumerate(past):
                tracker.predict()
                tracker.update(px)
            self.predict_buf.track_id_kf_model_dict[track_id] = tracker
            
            # save position and velocity
            # use raw position
            track_pos_dict[track_id] = past[-1]
            # ues filtered position
            #track_pos_dict[track_id] = tracker.x.reshape(1,4)[0, [0,2]]
            track_vel_dict[track_id] = tracker.x.reshape(1,4)[0, [1,3]] * self.track_predict_fps[track_id]

        # clean up missed track if necessary
        missing_track_id_list = set(self.predict_buf.track_input_queue_dict.keys()) - set(alive_track_id_list)
        stop_publish_track_id_list = set()
        for track_id in missing_track_id_list:
            # increase missing counter
            if track_id not in self.predict_buf.track_id_missing_time_dict:
                self.predict_buf.track_id_missing_time_dict[track_id] = 0
            self.predict_buf.track_id_missing_time_dict[track_id] += 1

            # if missing long time, stop publishing in people topic
            if self.predict_buf.track_id_missing_time_dict[track_id]>self.n_frames_inactive_to_stop_publish:
                stop_publish_track_id_list.add(track_id)
            
            # if missing long time, delete track
            if self.predict_buf.track_id_missing_time_dict[track_id]>self.n_frames_inactive_to_remove:
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

            last_vel = tracker.x.reshape(1,4)[0, [1,3]] * self.track_predict_fps[track_id]

            # save position and velocity
            # use raw position
            track_pos_dict[track_id] = np.array(self.predict_buf.track_input_queue_dict[track_id])[-1,:2]
            # ues filtered position
            #track_pos_dict[track_id] = tracker.x.reshape(1,4)[0, [0,2]]
            track_vel_dict[track_id] = last_vel
        
        self.pub_result(msg, alive_track_id_list, track_pos_dict, track_vel_dict)
        
        self.vis_result(msg, alive_track_id_list, track_pos_dict, track_vel_dict)


def main():
    rospy.init_node('predict_people_py', anonymous=True)

    publish_simulator_people = rospy.get_param("~publish_simulator_people")

    input_time = 5 # number of frames to start prediction
    output_time = 5 # number of frames to predict
    n_frames_inactive_to_remove = 60 # number of frames for a track to be inactive before removal (this value should be enough long because track_people_py resturns recovered tracks)
    n_frames_inactive_to_stop_publish = 2 # number of frames for a track to be inactive before stop publishing in people topic
    fps_est_time = 100 # number of frames which are used to estimate FPS
    
    predict_people = PredictKfPeople(input_time, output_time, n_frames_inactive_to_remove, n_frames_inactive_to_stop_publish, fps_est_time, publish_simulator_people)
    
    try:
        plt.ion()
        plt.show()
        rospy.spin()
    except KeyboardInterrupt:
      rospy.loginfo("Shutting down")


if __name__=='__main__':
    main()
