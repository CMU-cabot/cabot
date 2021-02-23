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
import json
import os
import sys

from matplotlib import pyplot as plt
from matplotlib.path import Path
import math
import numpy as np
from geometry_msgs.msg import Point, Point32, Polygon, PolygonStamped, Pose, PoseStamped
from people_msgs.msg import People, Person
from queue_msgs.msg import Queue
import rospy
from std_msgs.msg import Header, String
import tf
import tf2_ros
import tf2_geometry_msgs 
from visualization_msgs.msg import Marker, MarkerArray

from queue_utils_py import geometry_utils
from queue_utils_py import navigate_utils
from queue_utils_py import file_utils


class DetectQueuePeople():
    def __init__(self, queue_name, frame_id, queue_annotation, queue_velocity_threshold, queue_distance_threshold, queue_adjust_tolerance, dist_interval_queue_navigate_path):
        self.queue_name = queue_name
        self.frame_id = frame_id
        self.queue_annotation = queue_annotation
        self.queue_velocity_threshold = queue_velocity_threshold
        self.queue_distance_threshold = queue_distance_threshold
        self.queue_adjust_tolerance = queue_adjust_tolerance
        self.dist_interval_queue_navigate_path = dist_interval_queue_navigate_path
        rospy.loginfo("initialized DetectQueuePeople, frame_id = " + str(self.frame_id) + ", queue_annotation = " + str(queue_annotation))


    def update_frame_id(self, update_frame_id):
        if self.frame_id==update_frame_id:
            self.queue_expected_path_pose_array = self.queue_annotation["queue_expected_path"]
            self.adjusted_queue_expected_path_pose_array = copy.deepcopy(self.queue_expected_path_pose_array)

            self.queue_obstacle_point_array = self.queue_annotation["queue_obstacle_polygon"]
            self.queue_obstacle_path = Path(self.queue_obstacle_point_array)

            self.queue_obstacle_polygon_msg = Polygon()
            for point in self.queue_obstacle_point_array:
                self.queue_obstacle_polygon_msg.points.append(Point32(x=point[0], y=point[1], z=0.0))

            # create line segments for queue expected path
            self.queue_expected_path_line_segments = []
            self.queue_expected_path_line_segments_length = []
            self.queue_expected_path_line_segments_norm_vec = []
            for idx, queue_expected_path_pose in enumerate(self.queue_expected_path_pose_array):
                if idx<len(self.queue_expected_path_pose_array)-1:
                    point1 = np.array([self.queue_expected_path_pose_array[idx].position.x, self.queue_expected_path_pose_array[idx].position.y])
                    point2 = np.array([self.queue_expected_path_pose_array[idx+1].position.x, self.queue_expected_path_pose_array[idx+1].position.y])
                    self.queue_expected_path_line_segments.append((point1, point2))
                    self.queue_expected_path_line_segments_length.append(np.linalg.norm(point2-point1))
                    self.queue_expected_path_line_segments_norm_vec.append((point2-point1)/np.linalg.norm(point2-point1))
            
            self.adjusted_queue_expected_path_line_segments = copy.deepcopy(self.queue_expected_path_line_segments)
            self.adjusted_queue_expected_path_line_segments_length = copy.deepcopy(self.queue_expected_path_line_segments_length)
            self.adjusted_queue_expected_path_line_segments_norm_vec = copy.deepcopy(self.queue_expected_path_line_segments_norm_vec)
        else:
            self.queue_expected_path_pose_array = None
            self.adjusted_queue_expected_path_pose_array = None

            self.queue_obstacle_point_array = None
            self.queue_obstacle_path = None

            self.queue_obstacle_polygon_msg = None

            self.queue_expected_path_line_segments = None
            self.queue_expected_path_line_segments_length = None
            self.queue_expected_path_line_segments_norm_vec = None
            self.adjusted_queue_expected_path_line_segments = None
            self.adjusted_queue_expected_path_line_segments_length = None
            self.adjusted_queue_expected_path_line_segments_norm_vec = None


    # get head-tail queue position in map frame
    def get_head_tail_from_queue(self, sorted_queue_people_position_list):
        head_tail_position = []
        if len(sorted_queue_people_position_list)==1:
            # get same position as head and tail person position
            for idx in range(2):
                head_tail_position.append(sorted_queue_people_position_list[0])
        elif len(sorted_queue_people_position_list)>1:
            for idx in range(2):
                if idx==0:
                    # get head person position
                    person_idx = 0
                else:
                    # get tail person position
                    person_idx = -1
                head_tail_position.append(sorted_queue_people_position_list[person_idx])
        return head_tail_position


    def people_cb(self, people, map_people_pose_stamped_list):
        if self.queue_expected_path_pose_array is None or self.adjusted_queue_expected_path_pose_array is None:
            rospy.logerr("floor queue data is not specified.")
            return

        # step1 : find people in queue
        # select people who are enogh close to queue expected path
        queue_people_name_list = []
        queue_people_position_list = []
        queue_closest_path_segment_idx_list = []
        queue_closest_path_point_list = []
        for person, map_person_pose_stamped in zip(people, map_people_pose_stamped_list):
            map_person_pos = np.array([map_person_pose_stamped.pose.position.x, map_person_pose_stamped.pose.position.y])

            # confirm person is not in obstacle area
            if not self.queue_obstacle_path.contains_point(map_person_pos):

                # find the closesest queue expected path segment (start from segments far from head point)
                min_dist = None
                min_dist_segment_idx = None
                min_dist_closest_path_point = None
                for segment_idx in range(len(self.queue_expected_path_line_segments)):
                    line_segment = self.queue_expected_path_line_segments[segment_idx]

                    # find the closest point from person to queue expected path segments
                    closest_path_point = geometry_utils.get_closest_point_to_line(map_person_pos, line_segment[0], line_segment[1])
                    
                    # confirm that line from person to the closest point does not intersect with queue obstacle area
                    person_to_line_point_array = []
                    person_to_line_point_array.append(map_person_pos.tolist())
                    person_to_line_point_array.append(closest_path_point.tolist())
                    person_to_line_path = Path(person_to_line_point_array)
                    if not self.queue_obstacle_path.intersects_path(person_to_line_path, filled=False):
                        # get distance from person to queue expected path segments
                        dist = np.linalg.norm(closest_path_point - map_person_pos)
                        if (dist<self.queue_distance_threshold) and ((min_dist is None) or (dist<min_dist)):
                            min_dist = dist
                            min_dist_segment_idx = segment_idx
                            min_dist_closest_path_point = closest_path_point
                
                # queue prson is found
                if (min_dist is not None) and (min_dist_segment_idx is not None) and (min_dist_closest_path_point is not None):
                    # project person velocity vector to closest queue expected path segment
                    vec_person_vel = np.array([person.velocity.x, person.velocity.y])
                    norm_vec_min_dist_line_segment = self.queue_expected_path_line_segments_norm_vec[min_dist_segment_idx]
                    projected_vec_person_vel = np.dot(vec_person_vel, norm_vec_min_dist_line_segment)
                    # subtract person velocity projected to closest queue expected path segment (ignore velocity along queue expected path)
                    sub_projected_vec_person_vel = np.linalg.norm(vec_person_vel) - projected_vec_person_vel
                    # check if person is moving fast to differnt direction to queue path
                    if sub_projected_vec_person_vel < self.queue_velocity_threshold:
                        queue_people_name_list.append(person.name)
                        queue_people_position_list.append(map_person_pose_stamped.pose.position)
                        queue_closest_path_segment_idx_list.append(min_dist_segment_idx)
                        queue_closest_path_point_list.append(min_dist_closest_path_point)
                    else:
                        rospy.loginfo("person is not moving along queue path, name = " + person.name + ", velocity = " + str(sub_projected_vec_person_vel))
                else:
                    rospy.loginfo("person is not close to queue expected path, name = " + person.name)
            else:
                rospy.loginfo("person is in obstacle area, name = " + person.name)

        # step2 : sort people in queue
        # sort people by distance between people's position and queue head point
        if len(queue_people_name_list)>0 and self.queue_expected_path_pose_array:
            dist_from_start_list = []
            for person_idx in range(len(queue_people_name_list)):
                # calculate distance between person in queue and queue head point
                # each person's position is mapped to the closest point in queue expected path
                dist = geometry_utils.get_distance_to_queue_head(queue_closest_path_segment_idx_list[person_idx], queue_closest_path_point_list[person_idx],
                                                                self.queue_expected_path_line_segments, self.queue_expected_path_line_segments_length)
                dist_from_start_list.append(dist)
            sort_queue_people_idx_list = np.argsort(np.array(dist_from_start_list))
            sorted_queue_people_name_list = [queue_people_name_list[idx] for idx in sort_queue_people_idx_list]
            sorted_queue_people_position_list = [queue_people_position_list[idx] for idx in sort_queue_people_idx_list]
            sorted_queue_closest_path_segment_idx_list = [queue_closest_path_segment_idx_list[idx] for idx in sort_queue_people_idx_list]
        else:
            sorted_queue_people_name_list = queue_people_name_list
            sorted_queue_people_position_list = queue_people_position_list
            sorted_queue_closest_path_segment_idx_list = queue_closest_path_segment_idx_list
        
        # step3 : calculate head tail position
        head_tail_position = self.get_head_tail_from_queue(sorted_queue_people_position_list)

        # step4 : calculate adjusted pose list for navigating queue
        navigate_pose_list = navigate_utils.calc_navigate_pose_list(self.queue_expected_path_pose_array, self.dist_interval_queue_navigate_path)

        # step5 : calculate key queue navigate poses using queue_expected_path_pose_msg_array and queue people's positions
        if len(sorted_queue_people_name_list)>0:
            # prepare buffer of people close to path segments
            path_segment_idx_people_name_dict = {}
            path_segment_idx_people_position_dict = {}
            for (person_name, person_position, path_segment_idx) in zip(sorted_queue_people_name_list, sorted_queue_people_position_list, sorted_queue_closest_path_segment_idx_list):
                if path_segment_idx not in path_segment_idx_people_name_dict:
                    path_segment_idx_people_name_dict[path_segment_idx] = [person_name]
                    path_segment_idx_people_position_dict[path_segment_idx] = [person_position]
                else:
                    path_segment_idx_people_name_dict[path_segment_idx].append(person_name)
                    path_segment_idx_people_position_dict[path_segment_idx].append(person_position)
            
            if len(sorted_queue_closest_path_segment_idx_list)>0:
                # select the path segment where tail person is close
                segment_idx = sorted_queue_closest_path_segment_idx_list[-1]

                # if the selected path segment is not the last segment (not close to the casher), move it using people's positions
                if (segment_idx<len(self.adjusted_queue_expected_path_line_segments)-1) and (segment_idx in path_segment_idx_people_name_dict) and (segment_idx in path_segment_idx_people_position_dict):
                    people_position_list = path_segment_idx_people_position_dict[segment_idx]

                    # calculate average pose diff
                    diff_pose_mat = None
                    for person_position in people_position_list:
                        line_segment = self.adjusted_queue_expected_path_line_segments[segment_idx]
                        perpendicular_point = geometry_utils.get_perpendicular_point_to_line(np.array([person_position.x, person_position.y]), line_segment[0], line_segment[1])

                        diff_pose_vec = np.array([[person_position.x-perpendicular_point[0], person_position.y-perpendicular_point[1]]])
                        if diff_pose_mat is None:
                            diff_pose_mat = diff_pose_vec
                        else:
                            diff_pose_mat = np.vstack((diff_pose_mat, diff_pose_vec))
                    if diff_pose_mat.shape[0]==1:
                        pose_move_vec = diff_pose_mat[0]
                    else:
                        pose_move_vec = np.mean(diff_pose_mat, axis=1)
                    
                    # confirm adjusted path segments will move larger than queue_adjust_tolerance 
                    if np.linalg.norm(pose_move_vec)>self.queue_adjust_tolerance:
                        if segment_idx==0:
                            proj_pose_move_vec = pose_move_vec
                        else:
                            prev_line_segment = self.adjusted_queue_expected_path_line_segments[segment_idx-1]
                            vec_prev_line_segment = np.array(prev_line_segment[1]) - np.array(prev_line_segment[0])
                            norm_vec_prev_line_segment = np.linalg.norm(vec_prev_line_segment)
                            projection = np.dot(pose_move_vec, vec_prev_line_segment)/norm_vec_prev_line_segment
                            proj_pose_move_vec = vec_prev_line_segment * (projection/norm_vec_prev_line_segment)
                        moved_pose_position = [self.adjusted_queue_expected_path_pose_array[segment_idx].position.x+proj_pose_move_vec[0], self.adjusted_queue_expected_path_pose_array[segment_idx].position.y+proj_pose_move_vec[1]]

                        # move poses for the path segment where tail person is close
                        next_line_segment = self.adjusted_queue_expected_path_line_segments[segment_idx+1]
                        vec_next_line_segment = np.array(next_line_segment[1]) - np.array(next_line_segment[0])
                        norm_vec_next_line_segment = np.linalg.norm(vec_next_line_segment)
                        projection = np.dot(pose_move_vec, vec_next_line_segment)/norm_vec_next_line_segment
                        proj_pose_move_vec = vec_next_line_segment * (projection/norm_vec_next_line_segment)
                        moved_next_pose_position = [self.adjusted_queue_expected_path_pose_array[segment_idx+1].position.x+proj_pose_move_vec[0], self.adjusted_queue_expected_path_pose_array[segment_idx+1].position.y+proj_pose_move_vec[1]]
                        
                        moved_path_point_array = []
                        moved_path_point_array.append(moved_pose_position)
                        moved_path_point_array.append(moved_next_pose_position)
                        moved_path = Path(moved_path_point_array)
                        # confirm that moved path segments does not intersect with queue obstacle area
                        if not self.queue_obstacle_path.intersects_path(moved_path, filled=False):
                            self.adjusted_queue_expected_path_pose_array[segment_idx].position.x = moved_pose_position[0]
                            self.adjusted_queue_expected_path_pose_array[segment_idx].position.y = moved_pose_position[1]
                            self.adjusted_queue_expected_path_pose_array[segment_idx+1].position.x = moved_next_pose_position[0]
                            self.adjusted_queue_expected_path_pose_array[segment_idx+1].position.y = moved_next_pose_position[1]

            # calculate queue path segments for adjusted queue expected pose lists
            for idx, pose in enumerate(self.adjusted_queue_expected_path_pose_array):
                if idx<len(self.adjusted_queue_expected_path_pose_array)-1:
                    point1 = np.array([self.adjusted_queue_expected_path_pose_array[idx].position.x, self.adjusted_queue_expected_path_pose_array[idx].position.y])
                    point2 = np.array([self.adjusted_queue_expected_path_pose_array[idx+1].position.x, self.adjusted_queue_expected_path_pose_array[idx+1].position.y])
                    self.adjusted_queue_expected_path_line_segments[idx] = (point1, point2)
                    self.adjusted_queue_expected_path_line_segments_length[idx] = np.linalg.norm(point2-point1)
            
            # update key pose orientation as they matches to moved pose lists
            for pose_idx, pose in enumerate(self.adjusted_queue_expected_path_pose_array):
                if pose_idx<len(self.adjusted_queue_expected_path_pose_array)-1:
                    next_pose = self.adjusted_queue_expected_path_pose_array[pose_idx+1]
                    pose_orientation = math.atan2(next_pose.position.y-pose.position.y, next_pose.position.x-pose.position.x)
                    pose_orientation_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, pose_orientation)
                    pose.orientation.x = pose_orientation_quat[0]
                    pose.orientation.y = pose_orientation_quat[1]
                    pose.orientation.z = pose_orientation_quat[2]
                    pose.orientation.w = pose_orientation_quat[3]
        
        # step6 : calculate adjusted pose list for navigating queue
        adjusted_navigate_pose_list = navigate_utils.calc_navigate_pose_list(self.adjusted_queue_expected_path_pose_array, self.dist_interval_queue_navigate_path)
        
        return sorted_queue_people_name_list, sorted_queue_people_position_list, head_tail_position, navigate_pose_list, adjusted_navigate_pose_list


class DetectQueuePeopleNode():
    def __init__(self, detect_queue_people_list, debug_without_mf_localization, debug_queue_annotation_map_frame, n_colors=100):
        self.detect_queue_people_list = detect_queue_people_list
        self.debug_without_mf_localization = debug_without_mf_localization
        self.debug_queue_annotation_map_frame = debug_queue_annotation_map_frame

        # start initialization
        rospy.init_node('detect_queue_people_py', anonymous=True)

        if self.debug_without_mf_localization:
            # when debug without multi floor localization is set, load specified queue annotation
            rospy.loginfo("debug_queue_annotation_map_frame = " + debug_queue_annotation_map_frame)
            for detect_queue_people in self.detect_queue_people_list:
                detect_queue_people.update_frame_id(debug_queue_annotation_map_frame)
            self.current_frame = debug_queue_annotation_map_frame
        else:
            self.current_frame_sub = rospy.Subscriber('/current_frame', String, self.current_frame_cb)
            self.current_frame = None

        # create tf listener
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

        # set subscriber, publisher
        self.people_sub = rospy.Subscriber('/people', People, self.people_cb)
        self.queue_pub = rospy.Publisher('/queue_people_py/queue', Queue, queue_size=1)
        self.vis_marker_array_pub = rospy.Publisher('/detect_queue_people_py/visualization_marker_array', MarkerArray, queue_size=1)
        self.vis_queue_expected_path_pub = rospy.Publisher('/detect_queue_people_py/visualization_queue_expected_path', MarkerArray, queue_size=1)
        self.vis_queue_obstacle_pub = rospy.Publisher('/detect_queue_people_py/visualization_queue_obstacle', PolygonStamped, queue_size=1)
        self.vis_queue_head_tail_pub = rospy.Publisher('/detect_queue_people_py/visualization_queue_head_tail', MarkerArray, queue_size=1)

        # variables for visualization
        self.list_colors = plt.cm.hsv(np.linspace(0, 1, n_colors)).tolist() # list of colors to assign to each track for visualization
        np.random.shuffle(self.list_colors) # shuffle colors


    def current_frame_cb(self, msg):
        self.current_frame = msg.data
        for detect_queue_people in self.detect_queue_people_list:
            detect_queue_people.update_frame_id(msg.data)


    def people_cb(self, msg):
        map_people_pose_stamped_list = []
        for person in msg.people:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = msg.header.frame_id
            pose_stamped.pose.position = person.position
            map_pose_stamped = self.tf2_buffer.transform(pose_stamped, self.current_frame)
            map_people_pose_stamped_list.append(map_pose_stamped)

        for detect_queue_people in self.detect_queue_people_list:
            if detect_queue_people.frame_id==self.current_frame:
                sorted_queue_people_name_list, sorted_queue_people_position_list, head_tail_position, navigate_pose_list, adjusted_navigate_pose_list = detect_queue_people.people_cb(msg.people, map_people_pose_stamped_list)

                self.pub_result(msg, sorted_queue_people_name_list, head_tail_position, navigate_pose_list, adjusted_navigate_pose_list, detect_queue_people.queue_name)
                self.vis_queue(msg, sorted_queue_people_name_list, sorted_queue_people_position_list, navigate_pose_list, adjusted_navigate_pose_list, detect_queue_people.queue_name)
                self.vis_head_tail(msg, head_tail_position, detect_queue_people.queue_name)
                self.vis_queue_settings(msg, detect_queue_people.queue_expected_path_pose_array, detect_queue_people.queue_obstacle_polygon_msg, detect_queue_people.queue_name)


    def pub_result(self, msg, sorted_queue_people_name_list, head_tail_position, navigate_pose_list, adjusted_navigate_pose_list, queue_name):
        # publish queue message
        queue_msg = Queue()
        queue_msg.header.seq = msg.header.seq
        queue_msg.header.stamp = msg.header.stamp
        queue_msg.header.frame_id = msg.header.frame_id
        queue_msg.name = queue_name
        for person_name in sorted_queue_people_name_list:
            queue_msg.people_names.append(person_name)
        for position in head_tail_position:
            queue_msg.head_tail.append(position)
        '''
        for pose in navigate_pose_list:
            queue_msg.navigate_path.append(pose)
        for pose in adjusted_navigate_pose_list:
            queue_msg.adjusted_navigate_path.append(pose)
        '''
        self.queue_pub.publish(queue_msg)

    
    def vis_queue(self, msg, sorted_queue_people_name_list, sorted_queue_people_position_list, navigate_pose_list, adjusted_navigate_pose_list, queue_name):
        # publish queue people marker array for rviz
        marker_array = MarkerArray()

        for idx_person, (person_name, person_position) in enumerate(zip(sorted_queue_people_name_list, sorted_queue_people_position_list)):
            marker = Marker()
            marker.header = msg.header
            marker.header.frame_id = msg.header.frame_id
            marker.ns = "queue-people-point-" + queue_name
            marker.id = int(person_name)
            if idx_person==len(sorted_queue_people_name_list)-1:
                marker.type = Marker.CUBE
            else:
                marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(1.0)
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.2
            marker.pose.position = person_position
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1
            marker.color.r = self.list_colors[int(person_name) % len(self.list_colors)][0]
            marker.color.g = self.list_colors[int(person_name) % len(self.list_colors)][1]
            marker.color.b = self.list_colors[int(person_name) % len(self.list_colors)][2]
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        
        for idx, pose in enumerate(navigate_pose_list):
            marker = Marker()
            marker.header = Header()
            marker.header = msg.header
            marker.header.frame_id = msg.header.frame_id
            marker.ns = "navigate-path-pose-" + queue_name
            marker.id = idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(1.0)
            marker.scale.x = 0.2
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.pose = pose
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        
        for idx, pose in enumerate(adjusted_navigate_pose_list):
            marker = Marker()
            marker.header = Header()
            marker.header = msg.header
            marker.header.frame_id = msg.header.frame_id
            marker.ns = "adjusted-navigate-path-pose-" + queue_name
            marker.id = idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(1.0)
            marker.scale.x = 0.2
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.pose = pose
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        
        self.vis_marker_array_pub.publish(marker_array)


    def vis_head_tail(self, msg, head_tail_position, queue_name):
        # publish queue head-tail marker array for rviz
        marker_array = MarkerArray()
        for idx, position in enumerate(head_tail_position):
            marker = Marker()
            marker.header = Header()
            marker.header = msg.header
            marker.header.frame_id = msg.header.frame_id
            marker.ns = "queue-head-tail-" + queue_name
            marker.id = idx
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(1.0)
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.2
            marker.pose.position = position
            if idx==len(head_tail_position)-1:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.vis_queue_head_tail_pub.publish(marker_array)


    def vis_queue_settings(self, msg, queue_expected_path_pose_array, queue_obstacle_polygon_msg, queue_name):
        marker_array = MarkerArray()
        for idx, pose_msg in enumerate(queue_expected_path_pose_array):
            marker = Marker()
            marker.header = Header()
            marker.header = msg.header
            marker.header.frame_id = msg.header.frame_id
            marker.ns = "queue-expected-path-" + queue_name
            marker.id = idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(1.0)
            marker.scale.x = 0.5
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.pose = pose_msg
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.vis_queue_expected_path_pub.publish(marker_array)

        polygon_stamped_msg = PolygonStamped()
        polygon_stamped_msg.header = Header()
        polygon_stamped_msg.header = msg.header
        polygon_stamped_msg.header.frame_id = msg.header.frame_id
        polygon_stamped_msg.header.stamp = rospy.Time.now()
        polygon_stamped_msg.polygon = queue_obstacle_polygon_msg
        self.vis_queue_obstacle_pub.publish(polygon_stamped_msg)


def main():
    queue_annotation_list_file = rospy.get_param('queue_people_py/queue_annotation_list_file')
    # debug_without_mf_localization is set as true when using debug mode without multi floor localization
    debug_without_mf_localization = rospy.get_param('queue_people_py/debug_without_mf_localization')
    # debug_queue_annotation_map_frame is used for selecting annotation from queue_annotation_list_file when using debug mode
    # when using multi floor localization, queue annotation is automatically selected from queue_annotation_list_file
    debug_queue_annotation_map_frame = rospy.get_param('queue_people_py/debug_queue_annotation_map_frame')

    # velocity of people in queue should be less than queue_velocity_threshold
    queue_velocity_threshold = rospy.get_param('/queue_people_py/queue_velocity_threshold')
    # people in queue should be closer to queue expected than queue_distance_threshold
    queue_distance_threshold = rospy.get_param('/queue_people_py/queue_distance_threshold')
    # adjust queue navigation path if queue will be adjusted larger than queue_adjust_tolerance
    queue_adjust_tolerance = rospy.get_param('/queue_people_py/queue_adjust_tolerance')
    # interval of distance to calculate queue navigation path
    dist_interval_queue_navigate_path = rospy.get_param('/queue_people_py/dist_interval_queue_navigate_path')
    
    queue_annotation_frame_id_dict = file_utils.load_queue_annotation_list_file(file_utils.get_filename(queue_annotation_list_file))

    detect_queue_people_list = []
    for frame_id in queue_annotation_frame_id_dict.keys():
        for idx, queue_annotation in enumerate(queue_annotation_frame_id_dict[frame_id]):
            detect_queue_people = DetectQueuePeople(frame_id+"-"+str(idx), frame_id, queue_annotation, queue_velocity_threshold, queue_distance_threshold,
                                                    queue_adjust_tolerance, dist_interval_queue_navigate_path)
            detect_queue_people_list.append(detect_queue_people)
    detect_queue_people_node = DetectQueuePeopleNode(detect_queue_people_list, debug_without_mf_localization, debug_queue_annotation_map_frame)
    
    try:
        plt.ion()
        plt.show()
        rospy.spin()
    except KeyboardInterrupt:
      rospy.loginfo("Shutting down")


if __name__=='__main__':
    main()
