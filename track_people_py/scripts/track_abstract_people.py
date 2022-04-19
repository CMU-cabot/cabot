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

from abc import ABCMeta, abstractmethod
from collections import deque
import time

from geometry_msgs.msg import PoseStamped
from matplotlib import pyplot as plt
from message_filters import ApproximateTimeSynchronizer
import message_filters
import numpy as np
import rospy
#from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import ColorRGBA
import tf
import tf2_ros
from track_people_py.msg import BoundingBox, TrackedBox, TrackedBoxes
from visualization_msgs.msg import Marker, MarkerArray
from diagnostic_updater import Updater, DiagnosticTask, HeaderlessTopicDiagnostic, FrequencyStatusParam
from diagnostic_msgs.msg import DiagnosticStatus


class AbsTrackPeople:
    __metaclass__ = ABCMeta
    
    
    def __init__(self, device, minimum_valid_track_duration):
        # settings for visualization
        self.vis_local = False
        self.vis_global = False

        # make sure only one camera is processed
        self.processing_detected_boxes = False
        
        # start initialization
        rospy.init_node('track_people_py', anonymous=True)
        
        self.minimum_valid_track_duration = minimum_valid_track_duration
        
        self.device = device
        self.detected_boxes_sub = rospy.Subscriber('track_people_py/detected_boxes', TrackedBoxes, self.detected_boxes_cb)
        self.tracked_boxes_pub = rospy.Publisher('track_people_py/tracked_boxes', TrackedBoxes, queue_size=10)
        self.visualization_marker_array_pub = rospy.Publisher('track_people_py/visualization_marker_array', MarkerArray, queue_size=10)
        
        self.frame_id = 0
        self.prev_detect_time_sec = 0
    
        self.updater = Updater()
        rospy.Timer(rospy.Duration(1), lambda e: self.updater.update())
        target_fps = rospy.get_param('~target_fps', 0)
        self.htd = HeaderlessTopicDiagnostic("PeopleTrack", self.updater,
                                             FrequencyStatusParam({'min':target_fps, 'max':target_fps}, 0.2, 2))

    @abstractmethod
    def detected_boxes_cb(self, detected_boxes_msg):
        pass
    
    
    def preprocess_msg(self, detected_boxes_msg):
        detect_results = []
        center_bird_eye_global_list = []
        for idx_bbox, bbox in enumerate(detected_boxes_msg.tracked_boxes):
            detect_results.append([bbox.box.xmin, bbox.box.ymin, bbox.box.xmax, bbox.box.ymax])
            center_bird_eye_global_list.append([bbox.center3d.x, bbox.center3d.y, bbox.center3d.z])
        return np.array(detect_results), center_bird_eye_global_list
    
    
    def pub_result(self, detected_boxes_msg, id_list, color_list, tracked_duration):
        self.htd.tick()
        # publish tracked boxes message
        tracked_boxes_msg = TrackedBoxes()
        tracked_boxes_msg.header = detected_boxes_msg.header
        tracked_boxes_msg.camera_id = detected_boxes_msg.camera_id
        tracked_boxes_msg.pose = detected_boxes_msg.pose
        for idx_bbox, bbox in enumerate(detected_boxes_msg.tracked_boxes):
            if tracked_duration[idx_bbox] < self.minimum_valid_track_duration:
                continue
            tracked_box = TrackedBox()
            tracked_box.header = bbox.header
            tracked_box.track_id = id_list[idx_bbox]
            tracked_box.color = ColorRGBA(color_list[idx_bbox][0], color_list[idx_bbox][1], color_list[idx_bbox][2], 0.0)
            tracked_box.box = bbox.box
            tracked_box.center3d = bbox.center3d
            tracked_boxes_msg.tracked_boxes.append(tracked_box)
        self.tracked_boxes_pub.publish(tracked_boxes_msg)

        rospy.loginfo("camera ID = " + detected_boxes_msg.camera_id + ", number of tracked people = " + str(len(tracked_boxes_msg.tracked_boxes)))
    
    
    def vis_result(self, detected_boxes_msg, id_list, color_list, tracked_duration):
        # publish visualization marker array for rviz
        marker_array = MarkerArray()
        for idx_bbox, bbox in enumerate(detected_boxes_msg.tracked_boxes):
            if tracked_duration[idx_bbox] < self.minimum_valid_track_duration:
                continue
            marker = Marker()
            marker.header = bbox.header
            marker.ns = "track-people"
            marker.id = id_list[idx_bbox]
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(0.5)
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.2
            marker.pose.position = bbox.center3d
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1
            marker.color.r = color_list[idx_bbox][0]
            marker.color.g = color_list[idx_bbox][1]
            marker.color.b = color_list[idx_bbox][2]
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.visualization_marker_array_pub.publish(marker_array)
        
        # visualize by 2D plot in global map
        if self.vis_global:
            plt_x = []
            plt_y = []
            plt_color = []
            for idx_bbox, bbox in enumerate(detected_boxes_msg.tracked_boxes):
                if tracked_duration[idx_bbox] < self.minimum_valid_track_duration:
                    continue
                plt_x.append(bbox.x)
                plt_y.append(bbox.y)
                plt_color.append(np.array(color_list[idx_bbox]))
            
            plt.figure(2)
            plt.cla()
            ax = plt.gca()
            ax.set_title("tracked people in global, camera="+detected_boxes_msg.camera_id)
            ax.grid(True)
            ax.legend()
            ax.set_xlabel('y')
            ax.set_ylabel('x')
            plt.scatter(plt_x, plt_y, c=plt_color)
            plt.scatter([-detected_boxes_msg_msg.pose.position.y], [detected_boxes_msg_msg.pose.position.x], c=[np.array([1.0, 0.0, 0.0])], marker='+')
            ax.set_xlim([-detected_boxes_msg_msg.pose.position.y-20,-detected_boxes_msg_msg.pose.position.y+20])
            ax.set_ylim([detected_boxes_msg_msg.pose.position.x-20,detected_boxes_msg_msg.pose.position.x+20])
            plt.draw()
            plt.pause(0.00000000001)
