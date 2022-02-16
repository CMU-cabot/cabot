#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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

import os
import json
import argparse
import ast
import math
from enum import Enum
import numpy as np
import threading

import rospy
import roslaunch
import tf2_ros
import tf_conversions
from std_msgs.msg import String, Int64, Float64
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, Point, Pose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, PointCloud2, LaserScan, NavSatFix, FluidPressure
from nav_msgs.msg import Odometry
from tf2_geometry_msgs import PoseStamped # necessary to use tfBuffer.transform(pose_stamped_msg, frame_id)
from tf2_geometry_msgs import PointStamped, Vector3Stamped

from cartographer_ros_msgs.msg import *
from cartographer_ros_msgs.srv import *

import geoutil
import resource_utils

from wireless_utils import extract_samples
from wireless_rss_localizer import SimpleRSSLocalizer

from mf_localization_msgs.msg import *
from mf_localization_msgs.srv import *

from altitude_manager import AltitudeManager

def json2anchor(jobj):
    return geoutil.Anchor(lat = jobj["lat"],
                        lng = jobj["lng"],
                        rotate = jobj["rotate"],
                        )


class LocalizationMode(Enum):
    INIT = "init"
    TRACK = "track"

    def __str__(self):
        return self.value

def convert_samples_coordinate(samples, from_anchor, to_anchor, floor):
    samples2 = []
    for s in samples:
        s2 = s.copy()
        info = s["information"]
        xy = geoutil.Point(x=info["x"], y= info["y"])
        latlng = geoutil.local2global(xy, from_anchor)
        local_coord = geoutil.global2local(latlng, to_anchor)
        s2["information"]["x"] = local_coord.x
        s2["information"]["y"] = local_coord.y
        s2["information"]["floor"] = floor
        samples2.append(s2)
    return samples2

class FloorManager:
    def __init__(self):
        self.node_id = None
        self.frame_id = None
        self.localizer = None
        self.map_filename = ""

        # publisher
        self.initialpose_pub = None
        self.imu_pub = None
        self.points_pub = None

        # services
        self.get_trajectory_states = None
        self.finish_trajectory = None
        self.start_trajectory = None

class MultiFloorManager:
    def __init__(self):

        # state variables
        self.is_active = True
        self.floor = None # state
        self.area = None # state
        self.current_frame = None # state
        self.mode = None # state
        self.valid_imu = False # state for input validation
        self.valid_points2 = False # state for input validation
        # for optimization detection
        self.map2odom = None # state
        self.optimization_detected = False # state
        self.odom_displacement = 0 # used for print
        # for loginfo
        self.spin_count = 0
        self.prev_spin_count = None

        self.ble_localizer_dict = {}
        self.ble_floor_localizer = None
        self.pressure_available = True
        self.altitude_manager = None

        # area identification
        self.area_floor_const = 10000
        self.area_localizer = None
        self.X_area = None
        self.Y_area = None
        self.previous_area_check_time = None
        self.area_check_interval = 1.0 # [s]

        self.seq_initialpose = 0
        self.transforms = []

        # average floor values
        self.floor_queue_size = 10
        self.floor_queue = [] # state
        self.floor_list = [] # store known floor values

        # failure detection
        self.rmse_threshold = 5.0
        self.loc_queue_min_size = 5
        self.loc_queue_max_size = 10
        self.loc_queue = []
        self.loc_beacon_queue = []

        # auto-relocalization
        self.auto_relocalization = False

        # frames
        self.global_map_frame = "map"
        self.local_map_frame = "map"
        self.odom_frame = "odom"
        self.published_frame = "base_link"
        self.base_link_frame = "base_link"
        self.global_position_frame = "base_link" # frame_id to compute global position

        # publisher
        self.current_floor_pub = rospy.Publisher("current_floor", Int64, latch=True, queue_size=10)
        self.current_floor_raw_pub = rospy.Publisher("current_floor_raw", Float64, latch=True, queue_size=10)
        self.current_frame_pub = rospy.Publisher("current_frame", String, latch=True, queue_size=10)
        self.current_map_filename_pub = rospy.Publisher("current_map_filename", String, latch=True, queue_size=10)
        self.scan_matched_points2_pub = None
        self.resetpose_pub  = rospy.Publisher("resetpose", PoseWithCovarianceStamped, queue_size=10)
        self.global_position_pub = rospy.Publisher("global_position", MFGlobalPosition, queue_size=10)
        self.localize_status_pub = rospy.Publisher("localize_status", MFLocalizeStatus, queue_size=10)
        self.set_localize_status(MFLocalizeStatus.UNKNOWN)

        # Subscriber
        self.scan_matched_points2_sub = None

        # verbosity
        self.verbose = False

        # input data validation
        self.norm_q_tolerance = 0.1 # to block [0,0,0,0] quaternion
        self.norm_acc_threshold = 0.1 # to block [0,0,0] linear_acceleration

        # for local_map_tf_timer_callback
        self.local_map_tf = None

    def set_localize_status(self, status):
        msg = MFLocalizeStatus()
        msg.status = status
        self.localize_status_pub.publish(msg)

    def imu_callback(self, msg):
        # validate imu message
        acc = msg.linear_acceleration
        q = msg.orientation
        acc_vec = np.array([acc.x, acc.y, acc.z])
        q_vec = np.array([q.x, q.y, q.z, q.w])
        norm_acc = np.linalg.norm(acc_vec)
        norm_q = np.linalg.norm(q_vec)
        if self.norm_acc_threshold <= norm_acc and np.abs(norm_q-1.0) < self.norm_q_tolerance:
            self.valid_imu = True
        else:
            self.valid_imu = False

        if not self.valid_imu:
            rospy.loginfo("imu input is invalid. (linear_acceleration="+str(acc_vec)+", orientation="+str(q_vec)+")")

        # use imu data
        if (self.floor is not None) and (self.area is not None) and (self.mode is not None) and self.valid_imu:
            imu_pub = self.ble_localizer_dict[self.floor][self.area][self.mode].imu_pub
            imu_pub.publish(msg)

    def scan_callback(self, msg):
        # Not implemented
        pass

    def points_callback(self, msg):
        # validate points input
        self.valid_points2 = True # set true if points message is coming

        if self.floor is not None and self.area is not None and self.mode is not None:
            points_pub = self.ble_localizer_dict[self.floor][self.area][self.mode].points_pub
            points_pub.publish(msg)

    def odom_callback(self, msg):
        if self.floor is not None and self.area is not None and self.mode is not None:
            odom_pub = self.ble_localizer_dict[self.floor][self.area][self.mode].odom_pub
            odom_pub.publish(msg)

    def scan_matched_points2_callback(self, msg):
        if self.scan_matched_points2_pub is None:
            self.scan_matched_points2_pub = rospy.Publisher("scan_matched_points2", PointCloud2, queue_size=10)
        self.scan_matched_points2_pub.publish(msg)

    def initialpose_callback(self, pose_with_covariance_stamped_msg: PoseWithCovarianceStamped):
        # substitute ROS time to prevent error when gazebo is running and the pose message is published by rviz
        pose_with_covariance_stamped_msg.header.stamp = rospy.Time.now() 

        if self.mode is None:
            self.mode = LocalizationMode.INIT

        if self.floor is None:
            rospy.loginfo("floor is unknown. Set floor by calling /set_current_floor service before publishing the 2D pose estimate.")

        if self.floor is not None and self.mode is not None:
            if self.mode == LocalizationMode.INIT:
                self.set_localize_status(MFLocalizeStatus.LOCATING)

            # transform pose in the message from map frame to a local frame
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header = pose_with_covariance_stamped_msg.header
            pose_stamped_msg.pose = pose_with_covariance_stamped_msg.pose.pose

            # detect area
            x_area = [[pose_stamped_msg.pose.position.x, pose_stamped_msg.pose.position.y, float(self.floor)*self.area_floor_const]] # [x,y,floor]
            area = self.area_localizer.predict(x_area)[0] # [area] area may change.

            if self.verbose:
                rospy.loginfo("multi_floor_manager.initialpose_callback: area="+str(area))

            # get information from floor_manager
            floor_manager = self.ble_localizer_dict[self.floor][area][self.mode]
            frame_id = floor_manager.frame_id
            map_filename = floor_manager.map_filename
            node_id = floor_manager.node_id

            # transform initialpose on the global map frame to the local map frame (frame_id).
            local_pose_stamped = tfBuffer.transform(pose_stamped_msg, frame_id, timeout=rospy.Duration(1.0)) # timeout 1.0 s
            local_pose = local_pose_stamped.pose
            local_pose.position.z = 0.0 # set z = 0 to ensure 2D position on the local map

            # restart trajectory with local_pose
            if self.area is not None:
                self.finish_trajectory() # finish trajectory before updating area value
            self.area = area
            self.start_trajectory_with_pose(local_pose)
            rospy.loginfo("called /"+node_id+"/"+str(self.mode)+"/start_trajectory")

            # publish current floor
            current_floor_msg = Int64()
            current_floor_msg.data = int(self.floor)
            self.current_floor_pub.publish(current_floor_msg)

            # publish current frame
            self.current_frame = frame_id
            current_frame_msg = String()
            current_frame_msg.data = self.current_frame
            self.current_frame_pub.publish(current_frame_msg)

            # publish current map_filename
            self.current_map_filename_pub.publish(map_filename)

            # update scan matched points subscriber
            if self.scan_matched_points2_sub is not None:
                self.scan_matched_points2_sub.unregister()
            self.scan_matched_points2_sub = rospy.Subscriber(node_id+"/"+str(self.mode)+"/"+"scan_matched_points2", PointCloud2, self.scan_matched_points2_callback)

    def restart_floor(self, local_pose: Pose):
        # set z = 0 to ensure 2D position on the local map
        local_pose.position.z = 0.0

        floor_manager = self.ble_localizer_dict[self.floor][self.area][self.mode]
        frame_id = floor_manager.frame_id
        map_filename = floor_manager.map_filename

        # local_pose to pose_cov_stamped
        pose_cov_stamped = PoseWithCovarianceStamped()
        pose_cov_stamped.header.stamp = rospy.Time.now()
        pose_cov_stamped.header.frame_id = frame_id
        pose_cov_stamped.header.seq = self.seq_initialpose
        self.seq_initialpose += 1
        pose_cov_stamped.pose.pose = local_pose
        covariance = np.diag(self.initial_pose_variance)
        pose_cov_stamped.pose.covariance = list(covariance.flatten())

        # start trajectory with local_pose
        self.resetpose_pub.publish(pose_cov_stamped) # publish local_pose for visualization
        status_code_start_trajectory = self.start_trajectory_with_pose(local_pose)
        rospy.loginfo("called /"+ floor_manager.node_id+"/"+str(self.mode)+"/start_trajectory")

        # publish current floor
        current_floor_msg = Int64()
        current_floor_msg.data = int(self.floor)
        self.current_floor_pub.publish(current_floor_msg)

        # publish current frame
        self.current_frame = frame_id
        current_frame_msg = String()
        current_frame_msg.data = self.current_frame
        self.current_frame_pub.publish(current_frame_msg)

        # publish current map_filename
        self.current_map_filename_pub.publish(map_filename)

        # update scan matched points subscriber
        node_id = floor_manager.node_id
        if self.scan_matched_points2_sub is not None:
            self.scan_matched_points2_sub.unregister()
        self.scan_matched_points2_sub = rospy.Subscriber(node_id+"/"+str(self.mode)+"/"+"scan_matched_points2", PointCloud2, self.scan_matched_points2_callback)

        if self.mode == LocalizationMode.INIT:
            self.set_localize_status(MFLocalizeStatus.LOCATING)
        if self.mode == LocalizationMode.TRACK:
            self.set_localize_status(MFLocalizeStatus.TRACKING)

    # simple failure detection based on the root mean square error between tracked and estimated locations
    def check_localization_failure(self, loc_track, loc_est):
        if self.verbose:
            rospy.loginfo("loc_track="+str(loc_track)+", loc_est="+str(loc_est))

        if self.loc_queue_max_size <= len(self.loc_queue):
            self.loc_queue.pop(0)
            self.loc_beacon_queue.pop(0)

        self.loc_queue.append(loc_track)
        self.loc_beacon_queue.append(loc_est)

        failure_detected = False
        if self.loc_queue_min_size <= len(self.loc_queue):
            X1 = np.array(self.loc_queue)
            X2 = np.array(self.loc_beacon_queue)
            rmse = np.sqrt(np.mean(np.sum((X1-X2)**2, axis=1)))
            if self.rmse_threshold <= rmse:
                failure_detected = True
                # clear location lists
                self.loc_queue = []
                self.loc_beacon_queue = []
            if self.verbose:
                rospy.loginfo("rmse="+str(rmse) + ", failure_detected="+str(failure_detected))

        return failure_detected

    def pressure_callback(self, message):
        self.altitude_manager.put_pressure(message)

    def beacons_callback(self, message):
        if self.verbose:
            rospy.loginfo("multi_floor_manager.beacons_callback")

        if not self.is_active:
            # do nothing
            return

        data = json.loads(message.data)
        beacons = data["data"]
        # detect floor
        loc = self.ble_floor_localizer.predict(beacons) # [[x,y,z,floor]]

        if loc is None:
            return

        floor_raw = np.mean(loc[:,3])
        if self.verbose:
            rospy.loginfo("loc = {}".format(str(loc)))
            rospy.loginfo("floor_raw = {}, {}". format(floor_raw, loc[:,3]))

        if len(self.floor_queue) < self.floor_queue_size:
            self.floor_queue.append(floor_raw)
        else:
            self.floor_queue.pop(0)
            self.floor_queue.append(floor_raw)

        # use one of the known floor values closest to the mean value of floor_queue
        mean_floor = np.mean(self.floor_queue)
        self.current_floor_raw_pub.publish(mean_floor)
        
        idx_floor = np.abs(np.array(self.floor_list) - mean_floor).argmin()
        floor = self.floor_list[idx_floor]

        # detect area
        x_area = [[loc[0,0], loc[0,1], floor*self.area_floor_const]] # [x,y,floor]
        area = self.area_localizer.predict(x_area)[0] # [area]

        if self.verbose:
            rospy.loginfo("floor = "+str(floor) + ", area=" + str(area) )

        # check other sensor data before staring a trajectory.
        if not (self.valid_imu and self.valid_points2):
            return # do not start a new trajectory if other sensor data are not ready.

        # switch cartgrapher node
        if self.floor is None:
            self.floor = floor
            self.area = area
            self.mode = LocalizationMode.INIT
            rospy.loginfo("initialize floor = "+str(self.floor))

            # coarse initial localization on local frame (frame_id)
            ble_localizer = self.ble_localizer_dict[self.floor][self.area][LocalizationMode.INIT].localizer
            if ble_localizer is None:
                raise RuntimeError("Unknown floor for BLE localizer "+str(self.floor))

            # local_loc is on the local coordinate on frame_id
            local_loc = ble_localizer.predict(beacons)
            # project loc to sample locations
            local_loc = ble_localizer.find_closest(local_loc)

            # create a local pose instance
            position = Point(local_loc[0,0], local_loc[0,1], local_loc[0,2]) # use the estimated position
            orientation = Quaternion(0.0, 0.0, 0.0, 1.0) # orientation is unknown.
            local_pose = Pose(position, orientation)

            self.restart_floor(local_pose)

        # floor change or init->track
        elif ((self.altitude_manager.is_height_changed() or not self.pressure_available) and self.floor != floor) \
             or (self.mode==LocalizationMode.INIT and self.optimization_detected):            
            if self.floor != floor:
                rospy.loginfo("floor change detected (" + str(self.floor) + " -> " + str(floor) + ")." )
            else:
                rospy.loginfo("optimization_detected. change localization mode init->track (displacement="+str(self.odom_displacement)+")")

            # set temporal variables
            target_floor = floor
            target_area = area
            target_mode = LocalizationMode.TRACK

            # check the availablity of local_pose on the target frame
            floor_manager = self.ble_localizer_dict[target_floor][target_area][target_mode]
            frame_id = floor_manager.frame_id # target frame_id
            local_transform = None
            try:
                # tf from the origin of the target floor to the robot pose
                local_transform = tfBuffer.lookup_transform(frame_id, self.base_link_frame, rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr('LookupTransform Error from '+ frame_id +" to " + self.base_link_frame )

            # update the trajectory only when local_transform is available
            if local_transform is not None:
                if self.floor != floor: # floor change
                    pass # nothing to do
                else:
                    self.optimization_detected = False

                # create local_pose instance
                position = local_transform.transform.translation # Vector3
                orientation = local_transform.transform.rotation # Quaternion
                local_pose = Pose(position, orientation)

                # try to finish the current trajectory before updating state variables
                self.finish_trajectory()

                # update state variables to switch floor
                self.floor = target_floor
                self.area = target_area
                self.mode = target_mode

                # restart trajectory with the updated state variables
                self.restart_floor(local_pose)

        else:
            # check localization failure
            try:
                t = tfBuffer.lookup_transform(self.global_map_frame, self.base_link_frame, rospy.Time(0))
                loc2D_track = np.array([t.transform.translation.x, t.transform.translation.y])
                loc2D_beacon = np.array([loc[0,0], loc[0,1]])
                failure_detected = self.check_localization_failure(loc2D_track, loc2D_beacon)
                if failure_detected and self.auto_relocalization:
                    self.restart_localization()
                    rospy.logerr("Auto-relocalization. (localization failure detected)")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.loginfo('LookupTransform Error from '+ self.global_map_frame +" to " + self.base_link_frame)

    # periodically check and update internal state variables (area and mode)
    def check_and_update_states(self):
        # check interval
        if self.previous_area_check_time is not None:
            now = rospy.get_time()
            if self.area_check_interval <= now - self.previous_area_check_time:
                self.previous_area_check_time = rospy.get_time()
            else:
                return
        else:
            self.previous_area_check_time = rospy.get_time()

        if self.verbose:
            rospy.loginfo("multi_floor_manager.check_and_update_states. (floor="+str(self.floor)+",mode="+str(self.mode))

        if self.floor is not None and self.mode is not None:
            # get robot pose
            try:
                robot_pose = tfBuffer.lookup_transform(self.global_map_frame, self.global_position_frame, rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(e)
                return

            # detect area switching
            x_area = [[robot_pose.transform.translation.x, robot_pose.transform.translation.y, float(self.floor)*self.area_floor_const]] # [x,y,floor]
            
            # find area candidates
            neigh_dist, neigh_ind = self.area_localizer.kneighbors(x_area, n_neighbors=10)
            area_candidates = self.Y_area[neigh_ind]

            # switch area when the detected area is stable
            unique_areas = np.unique(area_candidates)
            if len(unique_areas) == 1:
                area = unique_areas[0]
            else:
                area = self.area

            # if area change detected, switch trajectory
            if self.area != area \
                or (self.mode==LocalizationMode.INIT and self.optimization_detected):

                if self.area != area:
                    rospy.loginfo("area change detected (" + str(self.area) + " -> " + str(area) + ")." )
                else:
                    rospy.loginfo("optimization_detected. change localization mode init->track (displacement="+str(self.odom_displacement)+")")

                # set temporal variables
                target_area = area
                target_mode = LocalizationMode.TRACK
                # check the availablity of local_pose on the target frame
                floor_manager = self.ble_localizer_dict[self.floor][target_area][target_mode]
                frame_id = floor_manager.frame_id # target frame_id
                local_transform = None
                try:
                    # tf from the origin of the target floor to the robot pose
                    local_transform = tfBuffer.lookup_transform(frame_id, self.base_link_frame, rospy.Time(0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr('LookupTransform Error from '+ frame_id +" to " + self.base_link_frame )

                # update the trajectory only when local_transform is available
                if local_transform is not None:

                    if self.optimization_detected:
                        self.optimization_detected = False

                    # create local_pose instance
                    position = local_transform.transform.translation # Vector3
                    orientation = local_transform.transform.rotation # Quaternion
                    local_pose = Pose(position, orientation)
                    # try to finish the current trajectory before updating state variables
                    self.finish_trajectory()
                    # update state variables to switch area
                    self.area = target_area
                    self.mode = target_mode
                    # restart trajectory with the updated state variables
                    self.restart_floor(local_pose)

        return


    # broadcast tf from global_map_frame to each (local) map_frame
    # [deprecated]
    #def transforms_timer_callback(self, timer):
    #    for t in self.transforms:
    #        t.header.stamp = rospy.Time.now() # update timestamp
    #    broadcaster.sendTransform(self.transforms)

    # broadcast tf from global_map_frame to each (local) map_frame
    def send_static_transforms(self):
        for t in self.transforms:
            t.header.stamp = rospy.Time.now() # update timestamp
        static_broadcaster.sendTransform(self.transforms)

    # broadcast tf between the current_frame to local_map_frame
    def local_map_tf_timer_callback(self, timer):
        if self.local_map_tf is None:
            # initialization
            t = TransformStamped()
            t.child_frame_id = self.local_map_frame # static
            t.transform.translation = Vector3(0.0, 0.0, 0.0) # static
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0, 'sxyz')
            rotation = Quaternion(*q)
            t.transform.rotation = rotation # static

            # tentative values
            t.header.stamp = rospy.Time.now()
            t.header.frame_id =  None

            self.local_map_tf = t

        if self.current_frame is not None:
            t = self.local_map_tf

            # send transform only when current_frame changes
            if self.current_frame != t.header.frame_id:
                t.header.stamp = rospy.Time.now()
                t.header.frame_id =  self.current_frame

                transform_list = self.transforms + [t] # to keep self.transforms in static transform

                static_broadcaster.sendTransform(transform_list)


    # publish global position
    def global_position_callback(self, timer):
        averaging_interval = self.global_position_averaging_interval
        try:
            # convert global position on global_map_frame to lat lng
            end_time = tfBuffer.get_latest_common_time(self.global_map_frame, self.global_position_frame) # latest available time
            start_time = end_time - rospy.Duration(averaging_interval)
            trans_pos = tfBuffer.lookup_transform(self.global_map_frame, self.global_position_frame, end_time)
            xy = geoutil.Point(x=trans_pos.transform.translation.x, y=trans_pos.transform.translation.y)
            latlng = geoutil.local2global(xy, self.global_anchor)
            floor = self.floor
            # convert robot rotation to heading
            anchor_rotation = self.global_anchor.rotate # degrees (0 -> north, clock-wise)
            euler_angles = tf_conversions.transformations.euler_from_quaternion([trans_pos.transform.rotation.x, trans_pos.transform.rotation.y, trans_pos.transform.rotation.z, trans_pos.transform.rotation.w], 'sxyz')
            yaw_angle = euler_angles[2] #[roll, pitch, yaw] radien (0 -> x-axis, counter-clock-wise)
            heading = anchor_rotation + 90.0 - 180.0*yaw_angle/math.pi # added 90 degrees to convert y-axis to x-axis
            heading = heading % 360 # clip to the space of heading [0, 2pi]

            # velocity on odom_frame to prevent it from jumping]
            trans_vel_end = tfBuffer.lookup_transform(self.odom_frame, self.global_position_frame, end_time)
            trans_vel_start = tfBuffer.lookup_transform(self.odom_frame, self.global_position_frame, start_time)
            delta_x = trans_vel_end.transform.translation.x - trans_vel_start.transform.translation.x
            delta_y = trans_vel_end.transform.translation.y - trans_vel_start.transform.translation.y
            v_x = delta_x/averaging_interval
            v_y = delta_y/averaging_interval
            v_xy = math.sqrt(v_x**2 + v_y**2)

            # create and publishg a MFGlobalPosition message
            global_position = MFGlobalPosition()
            global_position.header.stamp = end_time
            global_position.header.frame_id = self.global_position_frame
            global_position.latitude = latlng.lat
            global_position.longitude = latlng.lng
            global_position.floor = floor
            global_position.heading = heading
            global_position.speed = v_xy
            self.global_position_pub.publish(global_position)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo('LookupTransform Error '+self.global_map_frame+" -> "+self.global_position_frame)
        except tf2_ros.TransformException as e:
            rospy.loginfo(e)

    def stop_localization_callback(self, data):
        resp = StatusResponse()
        if not self.is_active:
            resp.code = 1
            resp.message = "Stop localization failed. (localization is aleady stopped.)"
            return resp
        try:
            self.is_active = False
            self.finish_trajectory()
            self.reset_states()
            resp.code = 0
            resp.message = "Stopped localization."
        except:
            resp.code = 1
            resp.message = "Stop localization failed."
        return resp

    def start_localization_callback(self, data):
        resp = StatusResponse()
        if self.is_active:
            resp.code = 1
            resp.message = "Start localization failed. (localization is aleady started.)"
            return resp
        self.is_active = True
        resp.code = 0
        resp.message = "Starting localization."
        return resp

    def finish_trajectory(self):
        # try to finish the current trajectory
        floor_manager = self.ble_localizer_dict[self.floor][self.area][self.mode]
        get_trajectory_states = floor_manager.get_trajectory_states
        finish_trajectory = floor_manager.finish_trajectory
        res0 = get_trajectory_states()
        rospy.loginfo(res0)
        last_trajectory_id = res0.trajectory_states.trajectory_id[-1]
        last_trajectory_state = res0.trajectory_states.trajectory_state[-1] # uint8 -> int

        # finish trajectory only if the trajectory is active.
        if last_trajectory_state in [TrajectoryStates.ACTIVE]:
            trajectory_id_to_finish = last_trajectory_id
            res1 = finish_trajectory(trajectory_id_to_finish)
            rospy.loginfo(res1)

    def start_trajectory_with_pose(self, initial_pose: Pose):

        floor_manager = self.ble_localizer_dict[self.floor][self.area][self.mode]
        start_trajectory = floor_manager.start_trajectory

        # start trajectory
        configuration_directory = floor_manager.configuration_directory
        configuration_basename = floor_manager.configuration_basename
        use_initial_pose = True
        relative_to_trajectory_id = 0

        res2 = start_trajectory(configuration_directory,
                                configuration_basename,
                                use_initial_pose,
                                initial_pose,
                                relative_to_trajectory_id
                                )
        rospy.loginfo(res2)
        status_code = res2.status.code

        tfBuffer.clear() # clear buffered tf to avoid the effect of the finished trajectory

        return status_code

    def reset_states(self):
        self.floor = None
        self.area = None
        self.current_frame = None
        self.mode = None
        self.map2odom = None
        self.optimization_detected = False

        self.floor_queue = []

        self.spin_count = 0
        self.prev_spin_count = None

        self.valid_imu = False
        self.valid_points2 = False

        tfBuffer.clear() # clear buffered tf added by finished trajectories
        self.set_localize_status(MFLocalizeStatus.UNKNOWN)

    def restart_localization(self):
        self.is_active = False
        self.finish_trajectory()
        self.reset_states()
        self.is_active = True

    def restart_localization_callback(self, data):
        resp = StatusResponse()
        try:
            self.restart_localization()
            resp.code = 0
            resp.message = "Restarting localization..."
        except:
            resp.code = 1
            resp.message = "Restart localization failed."
        return resp

    def enable_relocalization_callback(self, data):
        resp = StatusResponse()
        self.auto_relocalization = True
        resp.code = 0
        resp.message = "Enabled auto relocalization."
        return resp

    def disable_relocalization_callback(self, data):
        resp = StatusResponse()
        self.auto_relocalization = False
        resp.code = 0
        resp.message = "Disabled auto relocalization."
        return resp

    def set_current_floor_callback(self, data):
        resp = StatusResponse()
        floor = int(data.data)
        if self.floor is None:
            self.floor = floor
            resp.code = 0
            resp.message = "Set floor to " + str(floor) + "."
        else:
            resp.code = 1
            resp.message = "Failed to set floor to " + str(floor) + ". Floor is already set to "+str(self.floor)+"."
        return resp

    # return
    #      MFGlobalPosition global_position
    # input:
    #      MFLocalPosition local_position
    def convert_local_to_global_callback(self, msg):
        averaging_interval = self.global_position_averaging_interval
        try:
            pos = PointStamped()
            vel = Vector3Stamped()
            pos.header = msg.local_position.header
            vel.header = msg.local_position.header
            pos.point = msg.local_position.position
            vel.vector = msg.local_position.velocity

            transformed_position_stamped = tfBuffer.transform(pos, self.global_map_frame, timeout=rospy.Duration(1.0)) # timeout 1.0 s
            transformed_velocity_stamped = tfBuffer.transform(vel, self.global_map_frame, timeout=rospy.Duration(1.0)) # timeout 1.0 s

            # point to latlng
            xy = geoutil.Point(x=transformed_position_stamped.point.x, y=transformed_position_stamped.point.y)
            latlng = geoutil.local2global(xy, self.global_anchor)

            floor = self.floor

            # velocity vector to heading and speed
            speed =  np.sqrt(transformed_velocity_stamped.vector.x**2 + transformed_velocity_stamped.vector.y**2)
            yaw_angle = np.arctan2(transformed_velocity_stamped.vector.y, transformed_velocity_stamped.vector.x) # heading angle
            anchor_rotation = self.global_anchor.rotate # degrees (0 -> north, clock-wise)
            heading = anchor_rotation + 90.0 - 180.0*yaw_angle/math.pi # added 90 degrees to convert y-axis to x-axis
            heading = heading % 360 # clip to the space of heading [0, 2pi]

            # create a
            global_position = MFGlobalPosition()
            global_position.header.stamp = transformed_position_stamped.header.stamp
            global_position.header.frame_id = self.global_position_frame
            global_position.latitude = latlng.lat
            global_position.longitude = latlng.lng
            global_position.floor = floor
            global_position.heading = heading
            global_position.speed = speed
            return global_position
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo('LookupTransform Error '+self.global_map_frame+" -> "+self.global_position_frame)
            return None
        except tf2_ros.TransformException as e:
            return None


class CurrentPublisher:
    def __init__(self, verbose=False):
        self.verbose = False
        self.publish_current_rate = rospy.get_param("~publish_current_rate", 0) # 0 for latch
        self.current_floor = None
        self.current_frame = None
        self.current_map_filename = None
        rospy.Subscriber("current_floor", Int64, self.current_floor_cb)
        rospy.Subscriber("current_frame", String, self.current_frame_cb)
        rospy.Subscriber("current_map_filename", String, self.current_map_filename_cb)
        self.pub_floor = rospy.Publisher("current_floor", Int64, queue_size=max(self.publish_current_rate, 1))
        self.pub_frame = rospy.Publisher("current_frame", String, queue_size=max(self.publish_current_rate, 1))
        self.pub_map = rospy.Publisher("current_map_filename", String, queue_size=max(self.publish_current_rate, 1))

        if self.publish_current_rate == 0:
            if self.verbose:
                rospy.loginfo("node will not publish current regularly (publish_current_rate = 0)")
            return
        self.thread = threading.Thread(target=self.publish_current)
        self.thread.start()

    def current_floor_cb(self, msg):
        self.current_floor = msg

    def current_frame_cb(self, msg):
        self.current_frame = msg

    def current_map_filename_cb(self, msg):
        self.current_map_filename = msg

    def publish_current(self):
        rate = rospy.Rate(self.publish_current_rate)
        if self.verbose:
            rospy.loginfo("node will publish current regularly (publish_current_rate = {})".format(self.publish_current_rate))

        while not rospy.is_shutdown():
            #publish
            if self.current_floor is not None:
                if self.verbose:
                    rospy.loginfo("current_floor = {}".format(self.current_floor.data))
                self.pub_floor.publish(self.current_floor)

            if self.current_frame is not None:
                if self.verbose:
                    rospy.loginfo("current_frame = {}".format(self.current_frame.data))
                self.pub_frame.publish(self.current_frame)

            if self.current_map_filename is not None:
                if self.verbose:
                    rospy.loginfo("current_map_filename = {}".format(self.current_map_filename.data))
                self.pub_map.publish(self.current_map_filename)

            if self.verbose:
                rospy.loginfo("try to publish")
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node('multi_floor_manager')
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    sub_topics = rospy.get_param("~topic_list", ['beacons','wireless/beacons','wireless/wifi'])

    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster = tf2_ros.TransformBroadcaster()
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    multi_floor_manager = MultiFloorManager()
    # load node parameters
    configuration_directory_raw = rospy.get_param("~configuration_directory")
    configuration_file_prefix = rospy.get_param("~configuration_file_prefix")
    temporary_directory_name = rospy.get_param("~temporary_directory_name", "tmp")

    multi_floor_manager.local_map_frame = rospy.get_param("~local_map_frame", "map")
    multi_floor_manager.global_map_frame = rospy.get_param("~global_map_frame", "map")
    multi_floor_manager.odom_frame = rospy.get_param("~odom_frame", "odom")
    multi_floor_manager.published_frame = rospy.get_param("~published_frame", "base_link")
    multi_floor_manager.global_position_frame = rospy.get_param("~global_position_frame", "base_link")
    meters_per_floor = rospy.get_param("~meters_per_floor", 5)
    odom_dist_th = rospy.get_param("~odom_displacement_threshold", 0.1)
    multi_floor_manager.floor_queue_size = rospy.get_param("~floor_queue_size", 3)

    multi_floor_manager.initial_pose_variance = rospy.get_param("~initial_pose_variance", [3, 3, 0.1, 0, 0, 100])
    n_neighbors_floor = rospy.get_param("~n_neighbors_floor", 3)
    n_neighbors_local = rospy.get_param("~n_neighbors_local", 3)
    min_beacons_floor = rospy.get_param("~min_beacons_floor", 3)
    min_beacons_local = rospy.get_param("~min_beacons_local", 3)

    # auto-relocalization parameters
    multi_floor_manager.auto_relocalization = rospy.get_param("~auto_relocalization", False)
    multi_floor_manager.rmse_threshold = rospy.get_param("~rmse_threshold", 5.0)
    multi_floor_manager.loc_queue_min_size = rospy.get_param("~location_queue_min_size", 5)
    multi_floor_manager.loc_queue_max_size = rospy.get_param("~location_queue_max_size", 10)

    # pressure topic parameters
    multi_floor_manager.pressure_available = rospy.get_param("~pressure_available", True)

    multi_floor_manager.verbose = rospy.get_param("~verbose", False)

    # global position parameters
    global_position_interval = rospy.get_param("~global_position_interval", 1.0) # default 1 [s] -> 1 [Hz]
    multi_floor_manager.global_position_averaging_interval = rospy.get_param("~averaging_interval", 1.0) # default 1 [s]

    current_publisher = CurrentPublisher(verbose=rospy.get_param("~verbose", False))

    # configuration file check
    configuration_directory = resource_utils.get_filename(configuration_directory_raw)
    temporary_directory = os.path.join(configuration_directory, temporary_directory_name)
    if not os.path.exists(temporary_directory):
        rospy.logerr("temporary_directory ["+temporary_directory+"] does not exist.")
        raise RuntimeError("temporary_directory ["+temporary_directory+"] does not exist.")

    # resolve topic remapping
    imu_topic_name = rospy.names.resolve_name("imu")
    scan_topic_name = rospy.names.resolve_name("scan")
    points2_topic_name = rospy.names.resolve_name("points2")
    beacons_topic_name = rospy.names.resolve_name("beacons")
    initialpose_topic_name = rospy.names.resolve_name("initialpose")
    odom_topic_name = rospy.names.resolve_name("odom")


    # rss offset parameter
    rssi_offset = 0.0
    robot = rospy.get_param("~robot", "")

    # set from the dictionary
    if rospy.has_param("~rssi_offset_list"):
        rssi_offset_list = rospy.get_param("~rssi_offset_list")
        if robot in rssi_offset_list.keys():
            rssi_offset = rssi_offset_list[robot]

    # overwrite rssi_offset if exists
    if rospy.has_param("~rssi_offset"):
        rssi_offset = rospy.get_param("~rssi_offset")
    rospy.loginfo("rssi_offset="+str(rssi_offset))


    # load the main anchor point
    anchor_dict = rospy.get_param("~anchor")
    map_list = rospy.get_param("~map_list")

    modes = [LocalizationMode.INIT , LocalizationMode.TRACK]

    global_anchor = geoutil.Anchor(lat = anchor_dict["latitude"],
                        lng = anchor_dict["longitude"],
                        rotate = anchor_dict["rotate"]
                        )
    multi_floor_manager.global_anchor = global_anchor

    samples_global_all = []
    floor_set = set()

    for map_dict in map_list:

        floor = float(map_dict["floor"])
        floor_str = str(int(map_dict["floor"]))
        area = int(map_dict["area"]) if "area" in map_dict else 0
        area_str = str(area)
        node_id = map_dict["node_id"]
        frame_id = map_dict["frame_id"]
        anchor = geoutil.Anchor(lat = map_dict["latitude"],
                            lng = map_dict["longitude"],
                            rotate = map_dict["rotate"]
                            )
        load_state_filename =  resource_utils.get_filename(map_dict["load_state_filename"])
        samples_filename = resource_utils.get_filename(map_dict["samples_filename"])
        map_filename = map_dict["map_filename"] if "map_filename" in map_dict else "" # keep the original string without resource resolving. if not found in map_dict, use "".

        floor_set.add(floor)

        with open(samples_filename, "r") as f:
            samples = json.load(f)

        # append area information to the samples
        for s in samples:
            s["information"]["area"] = area

        # extract iBeacon samples
        samples_extracted = extract_samples(samples, key="iBeacon")

        # fit localizer for the floor
        ble_localizer_floor = SimpleRSSLocalizer(n_neighbors=n_neighbors_local, min_beacons=min_beacons_local, rssi_offset=rssi_offset)
        ble_localizer_floor.fit(samples_extracted)

        if not floor in multi_floor_manager.ble_localizer_dict:
            multi_floor_manager.ble_localizer_dict[floor] = {}

        multi_floor_manager.ble_localizer_dict[floor][area] = {}

        # run ros nodes
        for mode in modes:
            namespace = node_id+"/"+str(mode)
            sub_mode = "tracking" if mode == LocalizationMode.TRACK else "rss_localization"

            included_configuration_basename = configuration_file_prefix + "_" + sub_mode + ".lua"
            tmp_configuration_basename = temporary_directory_name + "/" + configuration_file_prefix + "_" + sub_mode + "_" + floor_str + "_" + area_str + ".lua"
            # create  temporary config files
            with open(os.path.join(configuration_directory, tmp_configuration_basename), "w") as f:
                f.write("include \""+included_configuration_basename+"\"")
                f.write("options.map_frame = \""+frame_id+"\"")
                f.write("options.odom_frame = \""+multi_floor_manager.odom_frame+"\"")
                f.write("options.published_frame = \""+multi_floor_manager.published_frame+"\"")
                f.write("return options")

            package1 = "cartographer_ros"
            executable1 = "cartographer_node"

            package2 = "mf_localization"
            executable2 = "trajectory_restarter.py"

            # run cartographer node
            node1 = roslaunch.core.Node(package1, executable1,
                                name="cartographer_node",
                                namespace = namespace,
                                remap_args = [("scan", scan_topic_name), ("points2", points2_topic_name), ("imu", imu_topic_name), ("odom", odom_topic_name)],
                                output = "screen"
                                )
            node1.args = "-configuration_directory " + configuration_directory \
                            + " -configuration_basename " + tmp_configuration_basename \
                            + " -load_state_filename " + load_state_filename \
                            + " -start_trajectory_with_default_topics=false"
            script1 = launch.launch(node1)

            # trajectory restarter
            # set ros parameters before running a node that uses the parameters
            rospy.set_param(namespace+"/trajectory_restarter/configuration_directory", configuration_directory)
            rospy.set_param(namespace+"/trajectory_restarter/configuration_basename", tmp_configuration_basename)
            node2 = roslaunch.core.Node(package2, executable2,
                                namespace = namespace,
                                name="trajectory_restarter",
                                output = "screen")
            script2 = launch.launch(node2)

            # create floor_manager
            floor_manager =  FloorManager()
            floor_manager.configuration_directory = configuration_directory
            floor_manager.configuration_basename = tmp_configuration_basename
            multi_floor_manager.ble_localizer_dict[floor][area][mode] = floor_manager

        # set values to floor_manager
        for mode in modes:
            floor_manager = multi_floor_manager.ble_localizer_dict[floor][area][mode]

            floor_manager.localizer = ble_localizer_floor
            floor_manager.node_id = node_id
            floor_manager.frame_id = frame_id
            floor_manager.map_filename = map_filename
            # publishers
            floor_manager.imu_pub = rospy.Publisher(node_id+"/"+str(mode)+imu_topic_name , Imu, queue_size=4000)
            floor_manager.points_pub = rospy.Publisher(node_id+"/"+str(mode)+points2_topic_name, PointCloud2, queue_size=100)
            floor_manager.initialpose_pub = rospy.Publisher(node_id+"/"+str(mode)+initialpose_topic_name, PoseWithCovarianceStamped, queue_size=10)
            floor_manager.odom_pub = rospy.Publisher(node_id+"/"+str(mode)+odom_topic_name, Odometry, queue_size=100)

            # rospy service
            rospy.wait_for_service(node_id+"/"+str(mode)+'/get_trajectory_states')
            rospy.wait_for_service(node_id+"/"+str(mode)+'/finish_trajectory')
            rospy.wait_for_service(node_id+"/"+str(mode)+'/start_trajectory')
            floor_manager.get_trajectory_states = rospy.ServiceProxy(node_id+"/"+str(mode)+'/get_trajectory_states', GetTrajectoryStates)
            floor_manager.finish_trajectory = rospy.ServiceProxy(node_id+"/"+str(mode)+'/finish_trajectory', FinishTrajectory)
            floor_manager.start_trajectory = rospy.ServiceProxy(node_id+"/"+str(mode)+'/start_trajectory', StartTrajectory)

            multi_floor_manager.ble_localizer_dict[floor][area][mode] = floor_manager

        # convert samples to the coordinate of global_anchor
        samples_global = convert_samples_coordinate(samples, anchor, global_anchor, floor)
        samples_global_all.extend(samples_global)

        # calculate static transform
        xy = geoutil.global2local(anchor, global_anchor)
        yaw = - math.radians(anchor.rotate - global_anchor.rotate)

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id =  multi_floor_manager.global_map_frame
        t.child_frame_id = frame_id

        z = floor * meters_per_floor # for visualization
        trans = Vector3(xy.x, xy.y, z)
        t.transform.translation = trans

        q = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw, 'sxyz')
        rotation = Quaternion(*q)
        t.transform.rotation = rotation

        multi_floor_manager.transforms.append(t)

    multi_floor_manager.floor_list = list(floor_set)

    # a localizer to estimate floor
    samples_global_all_extracted = extract_samples(samples_global_all, key="iBeacon")
    multi_floor_manager.ble_floor_localizer = SimpleRSSLocalizer(n_neighbors=n_neighbors_floor, min_beacons=min_beacons_floor, rssi_offset=rssi_offset)
    multi_floor_manager.ble_floor_localizer.fit(samples_global_all_extracted)

    multi_floor_manager.altitude_manager = AltitudeManager()

    # area localizer
    X_area = []
    Y_area = []
    for s in samples_global_all:
        x_a = float(s["information"]["x"])
        y_a = float(s["information"]["y"])
        f_a = float(s["information"]["floor"])
        f_a = f_a * multi_floor_manager.area_floor_const
        X_area.append([x_a, y_a, f_a])
        area = int(s["information"]["area"])
        Y_area.append(area)
    from sklearn.neighbors import KNeighborsClassifier
    area_classifier = KNeighborsClassifier(n_neighbors=1)
    area_classifier.fit(X_area, Y_area)
    multi_floor_manager.X_area = np.array(X_area)
    multi_floor_manager.Y_area = np.array(Y_area)
    multi_floor_manager.area_localizer = area_classifier

    # global subscribers
    imu_sub = rospy.Subscriber("imu", Imu, multi_floor_manager.imu_callback)
    scan_sub = rospy.Subscriber("scan", LaserScan, multi_floor_manager.scan_callback)
    points2_sub = rospy.Subscriber("points2", PointCloud2, multi_floor_manager.points_callback)
    beacons_sub = rospy.Subscriber("beacons", String, multi_floor_manager.beacons_callback, queue_size=1)
    initialpose_sub = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, multi_floor_manager.initialpose_callback)
    odom_sub = rospy.Subscriber("odom", Odometry, multi_floor_manager.odom_callback)
    pressure_sub = rospy.Subscriber("pressure", FluidPressure, multi_floor_manager.pressure_callback)

    # services
    stop_localization_service = rospy.Service("stop_localization", StopLocalization, multi_floor_manager.stop_localization_callback)
    start_localization_service = rospy.Service("start_localization", StartLocalization, multi_floor_manager.start_localization_callback)
    restart_localization_service = rospy.Service("restart_localization", RestartLocalization, multi_floor_manager.restart_localization_callback)
    enable_relocalization_service = rospy.Service("enable_auto_relocalization", MFTrigger, multi_floor_manager.enable_relocalization_callback)
    disable_relocalization_service = rospy.Service("disable_auto_relocalization", MFTrigger, multi_floor_manager.disable_relocalization_callback)
    set_current_floor_service = rospy.Service("set_current_floor", MFSetInt, multi_floor_manager.set_current_floor_callback)
    convert_local_to_global_service = rospy.Service("convert_local_to_global", ConvertLocalToGlobal, multi_floor_manager.convert_local_to_global_callback)

    # publish map->local_map by /tf_static
    multi_floor_manager.send_static_transforms()

    # timers
    timer_duration = 0.01 # 100 Hz
    # publish current_frame -> map when map_frame is not defined by global_map_frame
    # if global_map_frame == local_map_frame, local_map_frame is used to represent the origin of the global map
    # if global_map_frame != local_map_frame, local_map_frame is used to represent the origin of the local map corresponding to the origin of current_frame
    if multi_floor_manager.local_map_frame != multi_floor_manager.global_map_frame:
        local_map_tf_timer = rospy.Timer(rospy.Duration(timer_duration), multi_floor_manager.local_map_tf_timer_callback)
    # global position
    global_position_timer = rospy.Timer(rospy.Duration(global_position_interval), multi_floor_manager.global_position_callback)


    # detect optimization
    multi_floor_manager.map2odom = None

    # ros spin
    spin_rate = 10 # 10 Hz
    r = rospy.Rate(spin_rate)

    # for loginfo
    log_interval = spin_rate # loginfo at about 1 Hz

    while not rospy.is_shutdown():
        # detect odom movement
        try:
            t = tfBuffer.lookup_transform(multi_floor_manager.global_map_frame, multi_floor_manager.odom_frame, rospy.Time(0))
            if multi_floor_manager.is_active:
                if multi_floor_manager.map2odom is not None:
                    map2odom = multi_floor_manager.map2odom # local variable
                    dx = map2odom.transform.translation.x - t.transform.translation.x
                    dy = map2odom.transform.translation.y - t.transform.translation.y
                    dz = map2odom.transform.translation.z - t.transform.translation.z
                    dist = np.sqrt(dx**2 + dy**2 + dy**2)
                    if odom_dist_th < dist:
                        multi_floor_manager.optimization_detected = True
                        multi_floor_manager.odom_displacement = dist
                multi_floor_manager.map2odom = t
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            if (multi_floor_manager.prev_spin_count is None
                    or multi_floor_manager.spin_count - multi_floor_manager.prev_spin_count > log_interval):
                multi_floor_manager.prev_spin_count = multi_floor_manager.spin_count
                rospy.loginfo('LookupTransform Error '+multi_floor_manager.global_map_frame+" -> "+multi_floor_manager.odom_frame)

        # check and update area and mode
        multi_floor_manager.check_and_update_states()

        multi_floor_manager.spin_count += 1

        r.sleep()
