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
import orjson
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
import message_filters
from std_msgs.msg import String, Int64, Float64
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, Point, Pose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import TwistWithCovarianceStamped # gnss_fix_velocity
from sensor_msgs.msg import Imu, PointCloud2, LaserScan, NavSatFix, NavSatStatus, FluidPressure
from nav_msgs.msg import Odometry
from tf2_geometry_msgs import PoseStamped # necessary to use tfBuffer.transform(pose_stamped_msg, frame_id)
from tf2_geometry_msgs import PointStamped, Vector3Stamped

from cartographer_ros_msgs.msg import *
from cartographer_ros_msgs.srv import *

import geoutil
import resource_utils

from wireless_utils import extract_samples
from wireless_rss_localizer import create_wireless_rss_localizer

from mf_localization_msgs.msg import *
from mf_localization_msgs.srv import *

from altitude_manager import AltitudeManager

from diagnostic_updater import Updater, FunctionDiagnosticTask
from diagnostic_msgs.msg import DiagnosticStatus

def json2anchor(jobj):
    return geoutil.Anchor(lat = jobj["lat"],
                        lng = jobj["lng"],
                        rotate = jobj["rotate"],
                        )

def toTransmat(x,y,yaw):
    Tmat = np.array([[np.cos(yaw), -np.sin(yaw), x],
                    [np.sin(yaw), np.cos(yaw), y],
                    [0.0, 0.0, 1.0]])
    return Tmat

class LocalizationMode(Enum):
    INIT = "init"
    TRACK = "track"

    def __str__(self):
        return self.value

class IndoorOutdoorMode(Enum):
    UNKNOWN = "unknown"
    INDOOR = "indoor"
    OUTDOOR = "outdoor"

    def __str__(self):
        return self.value

class RSSType(Enum):
    iBeacon = 0
    WiFi = 1

def convert_samples_coordinate_slow(samples, from_anchor, to_anchor, floor):
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

def convert_samples_coordinate(samples, from_anchor, to_anchor, floor):
    # convert from_anchor point to to_anchor coordinate
    xy = geoutil.Point(x=0.0, y=0.0)
    latlng = geoutil.local2global(xy, from_anchor)
    local_coord = geoutil.global2local(latlng, to_anchor)
    X0 = local_coord.x
    Y0 = local_coord.y

    # calculate from_anchor to to_anchor rotation
    rad_from = - np.deg2rad(from_anchor.rotate)
    rad_to = - np.deg2rad(to_anchor.rotate)
    theta = rad_to - rad_from

    # convert samples coordinate X to to_anchor coordinate
    X = np.array([[s["information"]["x"], s["information"]["y"]] for s in samples]) # create [[sample.x, sample.y]] array
    R = np.array([[np.cos(theta), np.sin(theta)],
                    [-np.sin(theta), np.cos(theta)]])
    X2 = X @ R.T + np.array([X0, Y0])

    # create converted samples
    samples2 = []
    for i, s in enumerate(samples):
        s2 = s.copy()
        s2["information"]["x"] = X2[i,0]
        s2["information"]["y"] = X2[i,1]
        s2["information"]["floor"] = floor
        samples2.append(s2)

    return samples2

class FloorManager:
    def __init__(self):
        self.node_id = None
        self.frame_id = None
        self.localizer = None
        self.wifi_localizer = None
        self.map_filename = ""

        # publisher
        self.initialpose_pub = None
        self.imu_pub = None
        self.points_pub = None
        self.odom_pub = None
        self.fix_pub = None

        # services
        self.get_trajectory_states = None
        self.finish_trajectory = None
        self.start_trajectory = None

        # variables
        self.previous_fix_local_published = None

    def reset_states(self):
        self.previous_fix_local_published = None

class TFAdjuster:
    def __init__(self, frame_id: str, map_frame_adjust: str, adjust_tf: bool):
        # for gnss adjust
        # constant
        self.frame_id = frame_id
        self.map_frame_adjust = map_frame_adjust
        self.adjust_tf = adjust_tf

        # variable
        self.gnss_adjust_x = 0.0
        self.gnss_adjust_y = 0.0
        self.gnss_adjust_yaw = 0.0
        self.gnss_fix_list = []
        self.local_odom_list = []
        self.gnss_total_count = 0
        self.zero_adjust_uncertainty = 0.0

    def reset(self):
        self.gnss_adjust_x = 0.0
        self.gnss_adjust_y = 0.0
        self.gnss_adjust_yaw = 0.0
        self.gnss_fix_list = []
        self.local_odom_list = []
        self.gnss_total_count = 0
        self.zero_adjust_uncertainty = 0.0

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
        self.valid_beacon = False # state for input validation
        self.valid_wifi = False # state for input validation
        # for optimization detection
        self.map2odom = None # state
        self.optimization_detected = False # state
        self.odom_displacement = 0 # used for print
        # for initial pose estimation timeout
        self.init_time = None
        self.init_timeout = 60 # seconds
        self.init_timeout_detected = False
        # for loginfo
        self.spin_count = 0
        self.prev_spin_count = None

        self.ble_localizer_dict = {}
        self.ble_floor_localizer = None
        self.wifi_floor_localizer = None
        self.pressure_available = True
        self.altitude_manager = None

        # ble wifi localization
        self.use_ble = True
        self.use_wifi = False

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
        # unknown frame to temporarily cut a TF tree
        self.unknown_frame = "unknown"

        # publisher
        self.current_floor_pub = rospy.Publisher("current_floor", Int64, latch=True, queue_size=10)
        self.current_floor_raw_pub = rospy.Publisher("current_floor_raw", Float64, latch=True, queue_size=10)
        self.current_floor_smoothed_pub = rospy.Publisher("current_floor_smoothed", Float64, latch=True, queue_size=10)
        self.current_frame_pub = rospy.Publisher("current_frame", String, latch=True, queue_size=10)
        self.current_map_filename_pub = rospy.Publisher("current_map_filename", String, latch=True, queue_size=10)
        self.scan_matched_points2_pub = None
        self.resetpose_pub  = rospy.Publisher("resetpose", PoseWithCovarianceStamped, queue_size=10)
        self.global_position_pub = rospy.Publisher("global_position", MFGlobalPosition, queue_size=10)
        self.localize_status_pub = rospy.Publisher("localize_status", MFLocalizeStatus, latch=True, queue_size=10)
        self.localize_status = MFLocalizeStatus.UNKNOWN

        # Subscriber
        self.scan_matched_points2_sub = None

        # verbosity
        self.verbose = False

        # input data validation
        self.norm_q_tolerance = 0.1 # to block [0,0,0,0] quaternion
        self.norm_acc_threshold = 0.1 # to block [0,0,0] linear_acceleration

        # for local_map_tf_timer_callback
        self.local_map_tf = None

        # for gnss localization
        # parameters
        self.gnss_position_covariance_threshold = 0.2*0.2
        self.gnss_status_threshold = NavSatStatus.STATUS_GBAS_FIX # gnss status threshold for fix constraints
        self.gnss_fix_motion_filter_distance = 0.1 # [meters]
        self.gnss_track_error_threshold = 5.0
        self.gnss_track_yaw_threshold = np.radians(30)
        self.gnss_track_error_adjust = 0.1
        self.gnss_n_max_correspondences = 20
        self.gnss_n_min_correspondences_stable = 10
        self.gnss_odom_jump_threshold = 2.0
        self.gnss_odom_small_threshold = 0.5
        self.gnss_localization_interval = 10
        # variables
        self.gnss_adjuster_dict = {}
        self.prev_navsat_msg = None
        self.indoor_outdoor_mode = IndoorOutdoorMode.INDOOR
        self.gnss_is_active = False
        self.prev_publish_map_frame_adjust_timestamp = None
        self.gnss_fix_local_pub = None
        self.gnss_localization_time = None

        self.updater = Updater()
        rospy.Timer(rospy.Duration(1), lambda e: self.updater.update())
        def localize_status(stat):
            if self.valid_imu:
                stat.add("IMU input", "valid")
            else:
                stat.add("IMU input", "invalid")
            if self.valid_points2:
                stat.add("PointCloud2 input", "valid")
            else:
                stat.add("PointCloud2 input", "invalid")
            if self.valid_beacon:
                stat.add("Beacon input", "valid")
            else:
                stat.add("Beacon input", "invalid")
            if self.valid_wifi:
                stat.add("WiFi input", "valid")
            else:
                stat.add("WiFi input", "invalid")
            if self.floor:
                stat.add("Floor", self.floor)
            else:
                stat.add("Floor", "invalid")

            if self.localize_status == MFLocalizeStatus.UNKNOWN:
                stat.summary(DiagnosticStatus.WARN, "Unknown")
            if self.localize_status == MFLocalizeStatus.LOCATING:
                stat.summary(DiagnosticStatus.WARN, "Locating")
            if self.localize_status == MFLocalizeStatus.TRACKING:
                stat.summary(DiagnosticStatus.OK, "Tracking")
            if self.localize_status == MFLocalizeStatus.UNRELIABLE:
                stat.summary(DiagnosticStatus.WARN, "Unreliable")
        self.updater.add(FunctionDiagnosticTask("Localize Status", localize_status))

    def set_localize_status(self, status):
        self.localize_status = status
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
        self.valid_beacon = True
        if self.verbose:
            rospy.loginfo("multi_floor_manager.beacons_callback")

        data = json.loads(message.data)
        beacons = data["data"]

        if self.use_ble:
            self.rss_callback(beacons, rss_type=RSSType.iBeacon)

    def wifi_callback(self, message):
        self.valid_wifi = True
        if self.verbose:
            rospy.loginfo("multi_floor_manager.wifi_callback")

        data = json.loads(message.data)
        beacons = data["data"]

        if self.use_wifi:
            self.rss_callback(beacons, rss_type = RSSType.WiFi)

    def rss_callback(self, beacons, rss_type=RSSType.iBeacon):
        if not self.is_active:
            # do nothing
            return

        # detect floor
        floor_localizer = None
        if rss_type == RSSType.iBeacon:
            floor_localizer = self.ble_floor_localizer
        elif rss_type == RSSType.WiFi:
            floor_localizer = self.wifi_floor_localizer

        loc = floor_localizer.predict(beacons) # [[x,y,z,floor]]

        if loc is None:
            return

        floor_raw = np.mean(loc[:,3])
        if self.verbose:
            rospy.loginfo("loc = {}".format(str(loc)))
            rospy.loginfo("floor_raw = {}, {}". format(floor_raw, loc[:,3]))

        # extract latest floor_raw values from floor_queue to calculate the moving average
        now = rospy.get_time()
        self.floor_queue = [elem for elem in self.floor_queue if now - elem[0] < self.floor_queue_size]
        self.floor_queue.append([now, floor_raw])
        floor_values = [elem[1] for elem in self.floor_queue]
        if self.verbose:
            rospy.loginfo("floor_queue = "+str(floor_values))

        # calculate mean (smoothed) floor
        mean_floor = np.mean(floor_values)
        self.current_floor_raw_pub.publish(floor_raw)
        self.current_floor_smoothed_pub.publish(mean_floor)

        # use one of the known floor values closest to the mean value of floor_queue
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

        # do not start/switch trajectories when gnss is active and the robot is not in indoor environments.
        if self.gnss_is_active \
            and self.indoor_outdoor_mode != IndoorOutdoorMode.INDOOR:
            return

        # switch cartgrapher node
        if self.floor is None:
            self.floor = floor
            self.area = area
            self.mode = LocalizationMode.INIT
            rospy.loginfo("initialize floor = "+str(self.floor))

            # coarse initial localization on local frame (frame_id)
            localizer = None
            if rss_type == RSSType.iBeacon:
                localizer = self.ble_localizer_dict[self.floor][self.area][LocalizationMode.INIT].localizer
            elif rss_type == RSSType.WiFi:
                localizer = self.ble_localizer_dict[self.floor][self.area][LocalizationMode.INIT].wifi_localizer

            # local_loc is on the local coordinate on frame_id
            local_loc = localizer.predict(beacons)

            # project loc to sample locations
            local_loc = localizer.find_closest(local_loc)

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
                or (self.mode==LocalizationMode.INIT and self.optimization_detected) \
                or (self.mode==LocalizationMode.INIT and self.init_timeout_detected):

                if self.area != area:
                    rospy.loginfo("area change detected (" + str(self.area) + " -> " + str(area) + ")." )
                elif (self.mode==LocalizationMode.INIT and self.optimization_detected):
                    rospy.loginfo("optimization_detected. change localization mode init->track (displacement="+str(self.odom_displacement)+")")
                elif (self.mode==LocalizationMode.INIT and self.init_timeout_detected):
                    rospy.loginfo("optimization timeout detected. change localization mode init->track")

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

                    if self.init_timeout_detected:
                        self.init_timeout_detected = False

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
            global_position.floor = int(floor)
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

        # reset floor_manager
        floor_manager.reset_states()

        # reset gnss adjuster and publish
        self.gnss_adjuster_dict[self.floor][self.area].reset()
        self.publish_map_frame_adjust_tf()

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
        self.init_time = None
        self.init_timeout_detected = False

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

    def mf_navsat_callback(self, msg: MFNavSAT):
        self.prev_navsat_msg = msg

    def estimateRt(self, X, Y):
        """
        estimate rotation (R) and translation (t) from 2D point corespondances (y = [R|t]x)
        input: X(n_samples, 2), Y(n_samples, 2)
        """
        Xmean = np.mean(X, axis=0)
        Ymean = np.mean(Y, axis=0)

        Xdev = X - Xmean
        Ydev = Y - Ymean

        H = np.dot(np.transpose(Xdev), Ydev)
        U, S, Vt = np.linalg.svd(H)

        # rotation
        R =  np.dot(Vt.T, U.T)
        #M = np.eye(len(R))
        if np.linalg.det(R)<0.0: # check reflection
            #M[len(R)-1, len(R)-1] = -1.0
            Vt[-1,:] = -Vt[-1,:]
            S[-1] = -S[-1]

        R = np.dot(Vt.T, U.T)

        # translation
        t = - np.dot(R, Xmean.T) + Ymean.T

        return R, t


    # input:
    #      NavSatFix gnss_fix
    #      TwistWithCovarianceStamped gnss_fix_velocity
    def gnss_fix_callback(self, fix: NavSatFix, fix_velocity: TwistWithCovarianceStamped):
        # read message
        now = rospy.Time.now()
        stamp = fix.header.stamp
        gnss_frame = fix.header.frame_id
        latitude = fix.latitude
        longitude = fix.longitude
        position_covariance = fix.position_covariance

        floor_raw = 0.0 # outdoor
        idx_floor = np.abs(np.array(self.floor_list) - floor_raw).argmin()
        floor = self.floor_list[idx_floor] # select from floor_list to prevent using an unregistered value.

        # calculate moving direction from fix_velocity
        vel_e = fix_velocity.twist.twist.linear.x
        vel_n = fix_velocity.twist.twist.linear.y
        speed =  np.sqrt(vel_e**2 + vel_n**2)
        heading = np.arctan2(vel_n, vel_e) # yaw angle
        heading_degree = 90.0 - 180.0*heading/math.pi # added 90 degrees to convert y-axis to x-axis
        heading_degree = heading_degree % 360 # clip to the space of heading [0, 2pi]

        frame_id = self.global_map_frame
        anchor = self.global_anchor

        # lat,lng -> x,y
        latlng = geoutil.Latlng(lat=latitude, lng=longitude)
        gnss_xy = geoutil.global2local(latlng, anchor)

        # heading -> yaw
        anchor_rotation = anchor.rotate
        yaw_degrees = anchor_rotation + 90.0 - heading_degree
        yaw_degrees = yaw_degrees % 360
        gnss_yaw = np.radians(yaw_degrees)

        # pose covariance
        covariance_matrix = np.zeros((6,6))
        covariance_matrix[0:3,0:3] = np.reshape(position_covariance, (3,3))
        # TODO: orientation covariance

        # x,y,yaw -> PoseWithCovarianceStamped message
        pose_with_covariance_stamped = PoseWithCovarianceStamped()
        pose_with_covariance_stamped.header.stamp = now
        pose_with_covariance_stamped.header.frame_id = frame_id
        pose_with_covariance_stamped.pose.pose.position.x = gnss_xy.x
        pose_with_covariance_stamped.pose.pose.position.y = gnss_xy.y
        pose_with_covariance_stamped.pose.pose.position.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, gnss_yaw, 'sxyz')
        pose_with_covariance_stamped.pose.pose.orientation = Quaternion(*q)
        pose_with_covariance_stamped.pose.covariance = covariance_matrix.flatten()

        fix_local = [now.to_sec(), gnss_xy.x, gnss_xy.y, gnss_yaw] # timestamp, x, y, yaw

        # publish gnss fix in local frame
        self.gnss_fix_local_pub.publish(pose_with_covariance_stamped)

        # update indoor / outdoor status by using navsat status
        if self.prev_navsat_msg is not None:
            sv_status = self.prev_navsat_msg.sv_status
            if self.indoor_outdoor_mode == IndoorOutdoorMode.UNKNOWN: # at start up
                if sv_status == MFNavSAT.STATUS_INACTIVE:
                    self.indoor_outdoor_mode = IndoorOutdoorMode.INDOOR
                elif sv_status == MFNavSAT.STATUS_INTERMEDIATE:
                    self.indoor_outdoor_mode = IndoorOutdoorMode.INDOOR
                else: # sv_status == MFNavSAT.STATUS_ACTIVE:
                    self.indoor_outdoor_mode = IndoorOutdoorMode.OUTDOOR
            elif self.indoor_outdoor_mode == IndoorOutdoorMode.INDOOR:
                if sv_status == MFNavSAT.STATUS_ACTIVE:
                    self.indoor_outdoor_mode =  IndoorOutdoorMode.OUTDOOR
            else: # self.indoor_outdoor_mode == IndoorOutdoorMode.OUTDOOR:
                if sv_status == MFNavSAT.STATUS_INACTIVE:
                    self.indoor_outdoor_mode =  IndoorOutdoorMode.INDOOR
            self.prev_navsat_msg = None

        # disable gnss adjust in indoor invironments
        if self.indoor_outdoor_mode == IndoorOutdoorMode.INDOOR:
            # reset all gnss adjust
            for floor in self.gnss_adjuster_dict.keys():
                for area in self.gnss_adjuster_dict[floor].keys():
                    self.gnss_adjuster_dict[floor][area].reset()

        # do not use unreliable gnss fix
        fix_rejection_position_covariance = self.gnss_position_covariance_threshold
        if fix.status.status == NavSatStatus.STATUS_NO_FIX \
            or fix_rejection_position_covariance < fix.position_covariance[0]:
            # return if gnss fix is unreliable
            return
        else:
            # set outdoor mode if gnss fix is reliable
            self.gnss_outdoor_mode = IndoorOutdoorMode.OUTDOOR

        # check other sensor data before starting a new trajectory
        if not (self.valid_imu and self.valid_points2):
            return

        # do not start trajectories when gnss is not active
        if not self.gnss_is_active:
            return

        # publish gnss fix to localizer node
        if self.floor is not None and self.area is not None and self.mode is not None:
            floor_manager= self.ble_localizer_dict[self.floor][self.area][self.mode]
            if fix.status.status >= self.gnss_status_threshold:
                fix_pub = floor_manager.fix_pub
                fix.header.stamp = now # replace gnss timestamp with ros timestamp for rough synchronization

                # publish fix topic only when the distance travelled exceeds a certain level to avoid adding too many constraints
                if floor_manager.previous_fix_local_published is None:
                    fix_pub.publish(fix)
                    floor_manager.previous_fix_local_published = fix_local
                else:
                    prev_fix_local = floor_manager.previous_fix_local_published
                    distance_fix_local = np.sqrt((fix_local[1] - prev_fix_local[1])**2 + (fix_local[2] - prev_fix_local[2])**2)
                    if self.gnss_fix_motion_filter_distance <= distance_fix_local:
                        fix_pub.publish(fix)
                        floor_manager.previous_fix_local_published = fix_local

                # check initial pose optimization timeout in reliable gnss fix loop
                if self.init_time is not None:
                    if now - self.init_time > rospy.Duration(self.init_timeout):
                        self.init_time = None
                        self.init_timeout_detected = True

        # Forcibly prevent the tracked trajectory from going far away from the gnss position history
        track_error_detected = False
        # Prevent too large adjust
        large_adjust_detected = False

        update_gnss_adjust = False
        gnss_adjust_x = None
        gnss_adjust_y = None
        gnss_adjust_yaw = None
        may_stable_Rt = False

        tf_available = False

        # lookup tf
        try:
            # position on global_map_frame
            end_time = tfBuffer.get_latest_common_time(self.global_map_frame, gnss_frame) # latest available time
            # robot pose
            trans_pos = tfBuffer.lookup_transform(self.global_map_frame, self.global_position_frame, end_time)
            # gnss pose
            transform_gnss = tfBuffer.lookup_transform(self.global_map_frame, gnss_frame, end_time)


            # get tf required to compute gnss adjust
            # gnss adjuster
            gnss_adjuster = self.gnss_adjuster_dict[self.floor][self.area]
            tf_global2local = tfBuffer.lookup_transform(self.global_map_frame, gnss_adjuster.frame_id, end_time)
            tf_adjust2odom = tfBuffer.lookup_transform(gnss_adjuster.map_frame_adjust, self.odom_frame, end_time)
            tf_odom2gnss = tfBuffer.lookup_transform(self.odom_frame, gnss_frame, end_time)

            tf_available = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo('LookupTransform Error '+self.global_map_frame+" -> " + gnss_frame)
        except tf2_ros.TransformException as e:
            rospy.loginfo(e)

        if tf_available:
            # robot pose
            euler_angles_pos = tf_conversions.transformations.euler_from_quaternion([trans_pos.transform.rotation.x, trans_pos.transform.rotation.y, trans_pos.transform.rotation.z, trans_pos.transform.rotation.w], 'sxyz')
            yaw_pos = euler_angles_pos[2] #[roll, pitch, yaw] radien (0 -> x-axis, counter-clock-wise

            # gnss pose
            euler_angles_gnss = tf_conversions.transformations.euler_from_quaternion([transform_gnss.transform.rotation.x, transform_gnss.transform.rotation.y, transform_gnss.transform.rotation.z, transform_gnss.transform.rotation.w], 'sxyz')
            yaw_angle = euler_angles_gnss[2] #[roll, pitch, yaw] radien (0 -> x-axis, counter-clock-wise)

            # calculate error
            xy_error = np.linalg.norm([transform_gnss.transform.translation.x - gnss_xy.x,transform_gnss.transform.translation.y - gnss_xy.y])
            cosine = np.cos(gnss_yaw)*np.cos(yaw_angle) + np.sin(gnss_yaw)*np.sin(yaw_angle)
            cosine = np.clip(cosine, -1.0, 1.0)
            yaw_error = np.arccos(cosine)
            #rospy.loginfo("xy_error="+str(xy_error)+", yaw_error="+str(yaw_error))

            # gnss adjuster
            gnss_adjuster = self.gnss_adjuster_dict[self.floor][self.area]

            # estimate map_local -> map_adjust
            # global_map_frame -> local_map_frame (gnss_adjuster.frame_id) -> map_frame_adjust -> odom_frame -> base_link -> ... -> gnss

            euler_global2local = tf_conversions.transformations.euler_from_quaternion([tf_global2local.transform.rotation.x, tf_global2local.transform.rotation.y, tf_global2local.transform.rotation.z, tf_global2local.transform.rotation.w], 'sxyz')
            euler_adjust2odom =  tf_conversions.transformations.euler_from_quaternion([tf_adjust2odom.transform.rotation.x, tf_adjust2odom.transform.rotation.y, tf_adjust2odom.transform.rotation.z, tf_adjust2odom.transform.rotation.w], 'sxyz')
            euler_odom2gnss = tf_conversions.transformations.euler_from_quaternion([tf_odom2gnss.transform.rotation.x, tf_odom2gnss.transform.rotation.y, tf_odom2gnss.transform.rotation.z, tf_odom2gnss.transform.rotation.w], 'sxyz')

            global2local = [stamp.to_sec(), tf_global2local.transform.translation.x, tf_global2local.transform.translation.y, euler_global2local[2]]
            adjust2odom = [stamp.to_sec(), tf_adjust2odom.transform.translation.x, tf_adjust2odom.transform.translation.y, euler_adjust2odom[2]]
            odom2gnss = [stamp.to_sec(), tf_odom2gnss.transform.translation.x, tf_odom2gnss.transform.translation.y, euler_odom2gnss[2]]

            Tglobal2local = toTransmat(global2local[1], global2local[2], global2local[3])
            Tadjust2odom = toTransmat(adjust2odom[1], adjust2odom[2], adjust2odom[3])

            # jump detection and small motion filtering
            if len(gnss_adjuster.gnss_fix_list) > 0:
                diff = np.linalg.norm([gnss_adjuster.local_odom_list[-1][1] -  odom2gnss[1],
                                        gnss_adjuster.local_odom_list[-1][2] -  odom2gnss[2]])
                #rospy.loginfo("diff="+str(diff))
                if diff > self.gnss_odom_jump_threshold: # if the local slam is jumped
                    rospy.loginfo("detected local slam jump. diff="+str(diff)+".")
                    gnss_adjuster.gnss_fix_list = []
                    gnss_adjuster.local_odom_list = []
                    gnss_adjuster.gnss_fix_list.append(fix_local)
                    gnss_adjuster.local_odom_list.append(odom2gnss)
                    gnss_adjuster.gnss_total_count += 1
                    update_gnss_adjust = True
                elif diff <= self.gnss_odom_small_threshold: # if the movement is too small.
                    pass
                else:
                    gnss_adjuster.gnss_fix_list.append(fix_local)
                    gnss_adjuster.local_odom_list.append(odom2gnss)
                    gnss_adjuster.gnss_total_count += 1
                    update_gnss_adjust = True
            else:
                gnss_adjuster.gnss_fix_list.append(fix_local)
                gnss_adjuster.local_odom_list.append(odom2gnss)
                gnss_adjuster.gnss_total_count += 1
                update_gnss_adjust = True

            def apply_deadzone(x, deadzone):
                x_new = 0.0
                if deadzone <= x:
                    x_new = x - deadzone
                elif x <= -deadzone:
                    x_new = x + deadzone
                return x_new

            # estimate gnss_adjust
            if update_gnss_adjust:
                if 2 <= len(gnss_adjuster.gnss_fix_list): # use at least two points to calculate R, t

                    W = [] # odom_frame -> gnss_frame
                    Z = [] # global_map_frame -> gnss_frame

                    for idx in range(np.min([self.gnss_n_max_correspondences, len(gnss_adjuster.gnss_fix_list)])):
                        W.append([gnss_adjuster.local_odom_list[-idx-1][1], gnss_adjuster.local_odom_list[-idx-1][2], 1])
                        Z.append([gnss_adjuster.gnss_fix_list[-idx-1][1], gnss_adjuster.gnss_fix_list[-idx-1][2], 1])

                    W = np.array(W)
                    Z = np.array(Z)

                    X = W @ Tadjust2odom.transpose()
                    Y = (np.linalg.inv(Tglobal2local) @ Z.transpose()).transpose()

                    R, t = self.estimateRt(X[:,:2], Y[:,:2])

                    gnss_adjust_x = t[0]
                    gnss_adjust_y = t[1]
                    gnss_adjust_yaw = np.arctan2( R[1,0], R[0,0]) # [-pi, pi]

                    # apply dead zone
                    gnss_adjust_x = apply_deadzone(gnss_adjust_x, self.gnss_track_error_adjust)
                    gnss_adjust_y = apply_deadzone(gnss_adjust_y, self.gnss_track_error_adjust)

                    # apply zero adjust weight
                    if gnss_adjuster.zero_adjust_uncertainty < 1.0:
                        alpha = gnss_adjuster.zero_adjust_uncertainty
                        gnss_adjust_x = (1.0-alpha)*0.0 + alpha*gnss_adjust_x
                        gnss_adjust_y = (1.0-alpha)*0.0 + alpha*gnss_adjust_y
                        gnss_adjust_yaw = (1.0-alpha)*0.0 + alpha*gnss_adjust_yaw
                        # update zero_adjust_uncertainty
                        gnss_adjuster.zero_adjust_uncertainty += 1.0/self.gnss_n_max_correspondences
                        gnss_adjuster.zero_adjust_uncertainty = np.min([gnss_adjuster.zero_adjust_uncertainty, 1.0]) # clipping

                    # update gnss adjust values when the uncertainty of those values is high (initial stage) or those values are very stable (tracking stage).
                    may_stable_Rt = self.gnss_n_min_correspondences_stable <= len(gnss_adjuster.gnss_fix_list)
                    if gnss_adjuster.gnss_total_count < self.gnss_n_min_correspondences_stable \
                            or may_stable_Rt:
                        if gnss_adjuster.adjust_tf:
                            gnss_adjuster.gnss_adjust_x = gnss_adjust_x
                            gnss_adjuster.gnss_adjust_y = gnss_adjust_y
                            gnss_adjuster.gnss_adjust_yaw = gnss_adjust_yaw
                            rospy.loginfo("gnss_adjust updated: gnss_adjust_x="+str(gnss_adjust_x)+", gnss_adjust_y="+str(gnss_adjust_y)+", gnss_adjust_yaw="+str(gnss_adjust_yaw))
                        else:
                            rospy.loginfo("gnss_adjust NOT updated: gnss_adjust_x="+str(gnss_adjust_x)+", gnss_adjust_y="+str(gnss_adjust_y)+", gnss_adjust_yaw="+str(gnss_adjust_yaw))

            if np.sqrt(position_covariance[0]) + self.gnss_track_error_threshold <= xy_error:
                rospy.loginfo("gnss tracking error detected.")
                track_error_detected = True

            if may_stable_Rt:
                if np.sqrt(position_covariance[0]) + self.gnss_track_error_threshold <= np.sqrt(gnss_adjuster.gnss_adjust_x**2 + gnss_adjuster.gnss_adjust_y**2) \
                        or self.gnss_track_yaw_threshold < np.abs(gnss_adjuster.gnss_adjust_yaw):
                    rospy.loginfo("gnss map adjustment becomes too large.")
                    large_adjust_detected = True

            # update pose for reset
            if may_stable_Rt:
                q = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw_pos, 'sxyz')
                pose_with_covariance_stamped.pose.pose.orientation = Quaternion(*q)


        # publish (possibly) updated map adjust
        reset_trajectory = False
        reset_zero_adjust_uncertainty = False # set zero adjust uncertainty to 1 (unknown) when gnss adjust is completely unknown (e.g. initialization, large error with estimated gnss adjust)

        if self.floor is None: # run one time
            # set floor before initialpose_callback
            self.floor = floor
            reset_trajectory = True
            reset_zero_adjust_uncertainty = True
        elif track_error_detected:
            reset_trajectory = True
            reset_zero_adjust_uncertainty = True
        elif large_adjust_detected:
            reset_trajectory = True
        else:
            reset_trajectory = False

        if not reset_trajectory:
            return

        if self.gnss_localization_time is not None:
            if now - self.gnss_localization_time < rospy.Duration(self.gnss_localization_interval):
                return

        # start localization
        target_mode = None
        if self.mode is None:
            target_mode = LocalizationMode.INIT
            self.mode = target_mode
            self.init_time = now
        elif self.mode == LocalizationMode.INIT \
            and may_stable_Rt: # change mode INIT -> TRACK if mode has not been updated by optimization
            target_mode = LocalizationMode.TRACK

        # if floor, area, and mode are active, try to finish the trajectory.
        if self.area is not None:
            # preprocessing to prevent TF tree from jumping in a short moment after publishing map -> map_adjust and before resetting a trajectory.

            # finish trajectory to stop publishing map_adjust -> ... -> published_frame TF
            self.finish_trajectory()

            gnss_adjuster = self.gnss_adjuster_dict[self.floor][self.area]

            # temporarily cut TF tree between map_adjust -> odom_frame
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id =  multi_floor_manager.unknown_frame
            t.child_frame_id = multi_floor_manager.odom_frame
            t.transform.translation = Vector3(0.0, 0.0, 0.0)
            t.transform.rotation = Quaternion(0, 0, 0, 1)
            broadcaster.sendTransform([t])

        if target_mode is not None:
            self.mode = target_mode # set mode after finishing old trajectory

        # reset all gnss adjust
        for floor in self.gnss_adjuster_dict.keys():
            for area in self.gnss_adjuster_dict[floor].keys():
                gnss_adjuster = self.gnss_adjuster_dict[floor][area]
                gnss_adjuster.reset()
                if reset_zero_adjust_uncertainty:
                    gnss_adjuster.zero_adjust_uncertainty = 1.0

        # self.publish_map_frame_adjust_tf()
        # here, map_adjust -> ... -> published_frame TF must be disabled.
        self.initialpose_callback(pose_with_covariance_stamped) # reset pose on the global frame
        self.gnss_localization_time = now

    def publish_map_frame_adjust_tf(self):
        # prevent publishing redundant tf
        stamp = rospy.Time.now()
        if self.prev_publish_map_frame_adjust_timestamp is not None:
            if self.prev_publish_map_frame_adjust_timestamp == stamp:
                return

        transform_list = []
        for floor in self.ble_localizer_dict.keys():
            for area in self.ble_localizer_dict[floor].keys():
                gnss_adjuster = self.gnss_adjuster_dict[floor][area]

                parent_frame_id = gnss_adjuster.frame_id
                child_frame_id = gnss_adjuster.map_frame_adjust

                t = TransformStamped()
                t.header.stamp = stamp
                t.header.frame_id = parent_frame_id
                t.child_frame_id = child_frame_id
                t.transform.translation = Vector3(gnss_adjuster.gnss_adjust_x, gnss_adjuster.gnss_adjust_y, 0.0)
                q = tf_conversions.transformations.quaternion_from_euler(0, 0, gnss_adjuster.gnss_adjust_yaw, 'sxyz')
                t.transform.rotation = Quaternion(*q)

                transform_list.append(t)

        broadcaster.sendTransform(transform_list)
        self.prev_publish_map_frame_adjust_timestamp = stamp

    def map_frame_adjust_callback(self, timer):
        self.publish_map_frame_adjust_tf()

    # input:
    #      MFLocalPosition global_position
    def global_localizer_global_pose_callback(self, msg):
        rospy.loginfo("received global_pose from global_localizer: global_pose="+str(msg))

        stamp = msg.header.stamp
        latitude = msg.latitude
        longitude = msg.longitude
        floor_val = msg.floor
        heading = msg.heading

        frame_id = self.global_map_frame
        anchor = self.global_anchor

        # lat,lng -> x,y
        latlng = geoutil.Latlng(lat=latitude, lng=longitude)
        xy = geoutil.global2local(latlng, anchor)

        # heading -> yaw
        anchor_rotation = anchor.rotate
        yaw_degrees = anchor_rotation + 90.0 - heading
        yaw_degrees = yaw_degrees % 360
        yaw = np.radians(yaw_degrees)

        # x,y,yaw -> PoseWithCovarianceStamped message
        pose_with_covariance_stamped = PoseWithCovarianceStamped()
        pose_with_covariance_stamped.header.stamp = stamp
        pose_with_covariance_stamped.header.frame_id = frame_id
        pose_with_covariance_stamped.pose.pose.position.x = xy.x
        pose_with_covariance_stamped.pose.pose.position.y = xy.y
        pose_with_covariance_stamped.pose.pose.position.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw, 'sxyz')
        pose_with_covariance_stamped.pose.pose.orientation = Quaternion(*q)

        # set floor before initialpose_callback
        self.floor = floor

        # start localization
        self.initialpose_callback(pose_with_covariance_stamped)
        self.is_active = True

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

class CartographerParameterConverter:
    # yaml parameter structure
    #
    # cartographer:
    #   key: value
    #   init:
    #     key: value
    #   track:
    #     key: value
    # map_list:
    #   - node_id: node_id
    #     ...
    #     cartographer:
    #       key: value
    #       init:
    #         key: value
    #       track:
    #         key: value

    def __init__(self, all_params):
        modes = [LocalizationMode.INIT , LocalizationMode.TRACK]
        modes_str = [str(mode) for mode in modes]

        map_list = all_params.get("map_list")

        cartographer_dict_global = all_params.get("cartographer", None)

        parameter_dict = {}

        for map_dict in map_list:
            node_id = map_dict["node_id"]
            parameter_dict[node_id] = {}
            cartographer_dict_map = map_dict.get("cartographer", None)

            for mode in modes_str:
                cartographer_parameters = {}
                if cartographer_dict_global is not None:
                    for key in cartographer_dict_global.keys():
                        if not key in modes_str:
                            cartographer_parameters[key] = cartographer_dict_global[key]

                    if mode in cartographer_dict_global.keys():
                        cartographer_dict_global_mode = cartographer_dict_global[mode]
                        for key in cartographer_dict_global_mode.keys():
                            if not key in modes_str:
                                cartographer_parameters[key] = cartographer_dict_global_mode[key]

                if cartographer_dict_map is not None:
                    for key in cartographer_dict_map.keys():
                        if not key in modes_str:
                            cartographer_parameters[key] = cartographer_dict_map[key]

                    if mode in cartographer_dict_map.keys():
                        cartographer_dict_map_mode = cartographer_dict_map[mode]
                        for key in cartographer_dict_map_mode.keys():
                            if not key in modes_str:
                                cartographer_parameters[key] = cartographer_dict_map_mode[key]

                parameter_dict[node_id][mode] = cartographer_parameters

        self.parameter_dict = parameter_dict

    def get_parameters(self, node_id, mode):
        return self.parameter_dict.get(node_id).get(str(mode))


def extend_node_parameter_dictionary(all_params: dict) -> dict:
    """If specific parameters are undefined, calculate and add them to the parameter dictionary."""
    all_params_new = all_params.copy()

    map_list = all_params_new.get("map_list")
    floor_count = {}  # count floor for assigning area
    for map_dict in map_list:
        # read floor
        floor = float(map_dict["floor"])
        floor_str = str(int(map_dict["floor"]))
        floor_count.setdefault(floor, 0)

        # automatically assign area if undefined
        area = int(map_dict["area"]) if "area" in map_dict else None
        if area is None:
            area = floor_count[floor]
            map_dict["area"] = area
        floor_count[floor] += 1
        area_str = str(area)

        # automatically assign node_id if undefined
        node_id = map_dict["node_id"] if "node_id" in map_dict else None
        if node_id is None:
            node_id = "carto_"+floor_str+"_"+area_str
            map_dict["node_id"] = node_id

        # automatically assign floor_id if undefined
        frame_id = map_dict["frame_id"] if "frame_id" in map_dict else None
        if frame_id is None:
            frame_id = "map_"+node_id
            map_dict["frame_id"] = frame_id

    all_params_new["map_list"] = map_list
    return all_params_new


if __name__ == "__main__":
    rospy.init_node('multi_floor_manager')
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    all_params = rospy.get_param("~")

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
    multi_floor_manager.floor_queue_size = rospy.get_param("~floor_queue_size", 3) # [seconds]
    multi_floor_manager.init_timeout = rospy.get_param("~initial_pose_optimization_timeout", 60) # [seconds]

    multi_floor_manager.initial_pose_variance = rospy.get_param("~initial_pose_variance", [3, 3, 0.1, 0, 0, 100])
    n_neighbors_floor = rospy.get_param("~n_neighbors_floor", 3)
    n_neighbors_local = rospy.get_param("~n_neighbors_local", 3)
    min_beacons_floor = rospy.get_param("~min_beacons_floor", 3)
    min_beacons_local = rospy.get_param("~min_beacons_local", 3)
    floor_localizer_type = rospy.get_param("~floor_localizer", "SimpleRSSLocalizer")
    local_localizer_type = rospy.get_param("~local_localizer", "SimpleRSSLocalizer")

    # external localizer parameters
    use_gnss = rospy.get_param("~use_gnss", False)
    use_global_localizer = rospy.get_param("~use_global_localizer", False)

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
    fix_topic_name = rospy.names.resolve_name("gnss_fix")
    fix_velocity_topic_name = rospy.names.resolve_name("gnss_fix_velocity")


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

    # extend parameter dictionary
    all_params = extend_node_parameter_dictionary(all_params)

    # load the main anchor point
    anchor_dict = all_params.get("anchor")
    map_list = all_params.get("map_list")

    modes = [LocalizationMode.INIT , LocalizationMode.TRACK]

    global_anchor = geoutil.Anchor(lat = anchor_dict["latitude"],
                        lng = anchor_dict["longitude"],
                        rotate = anchor_dict["rotate"]
                        )
    multi_floor_manager.global_anchor = global_anchor

    samples_global_all = []
    floor_set = set()

    # load use_ble and use_wifi
    multi_floor_manager.use_ble = rospy.get_param("~use_ble", True)
    multi_floor_manager.use_wifi = rospy.get_param("~use_wifi", False)

    # load cartographer parameters
    cartographer_parameter_converter = CartographerParameterConverter(all_params)

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
        environment =  map_dict["environment"] if "environment" in map_dict else "indoor"
        use_gnss_adjust = map_dict["use_gnss_adjust"] if "use_gnss_adjust" in map_dict else False

        # check value
        if not environment in ["indoor", "outdoor"]:
            raise RuntimeError("unknown environment ("+environment+") is in the site configuration file")

        floor_set.add(floor)

        with open(samples_filename, "rb") as f:
            samples = orjson.loads(f.read())

        # append area information to the samples
        for s in samples:
            s["information"]["area"] = area

        # BLE beacon localizer
        ble_localizer_floor = None
        if multi_floor_manager.use_ble:
            # extract iBeacon samples
            samples_extracted = extract_samples(samples, key="iBeacon")
            # fit localizer for the floor
            ble_localizer_floor = create_wireless_rss_localizer(local_localizer_type, n_neighbors=n_neighbors_local, min_beacons=min_beacons_local, rssi_offset=rssi_offset)
            ble_localizer_floor.fit(samples_extracted)

        # WiFi localizer
        wifi_localizer_floor = None
        if multi_floor_manager.use_wifi:
            # extract wifi samples
            samples_wifi = extract_samples(samples, key="WiFi")
            # fit wifi localizer for the floor
            wifi_localizer_floor = create_wireless_rss_localizer(local_localizer_type, n_neighbors=n_neighbors_local, min_beacons=min_beacons_local)
            wifi_localizer_floor.fit(samples_wifi)

        if not floor in multi_floor_manager.ble_localizer_dict:
            multi_floor_manager.ble_localizer_dict[floor] = {}

        multi_floor_manager.ble_localizer_dict[floor][area] = {}

        # gnss adjust
        map_frame_adjust = frame_id + "_adjust"
        if not floor in multi_floor_manager.gnss_adjuster_dict:
            multi_floor_manager.gnss_adjuster_dict[floor] = {}
        multi_floor_manager.gnss_adjuster_dict[floor][area] = TFAdjuster(frame_id, map_frame_adjust, use_gnss_adjust)

        # run ros nodes
        for mode in modes:
            namespace = node_id+"/"+str(mode)
            sub_mode = "tracking" if mode == LocalizationMode.TRACK else "rss_localization"
            if environment != "indoor":
                sub_mode = sub_mode + "_" + environment

            included_configuration_basename = configuration_file_prefix + "_" + sub_mode + ".lua"
            tmp_configuration_basename = temporary_directory_name + "/" + configuration_file_prefix + "_" + sub_mode + "_" + floor_str + "_" + area_str + ".lua"

            # update config variables if needed
            options_map_frame = frame_id
            if use_gnss:
                options_map_frame = map_frame_adjust

            # load cartographer_parameters
            cartographer_parameters = cartographer_parameter_converter.get_parameters(node_id, mode)

            # create  temporary config files
            with open(os.path.join(configuration_directory, tmp_configuration_basename), "w") as f:
                f.write("include \""+included_configuration_basename+"\""+"\n")
                f.write("options.map_frame = \""+options_map_frame+"\""+"\n")
                f.write("options.odom_frame = \""+multi_floor_manager.odom_frame+"\""+"\n")
                f.write("options.published_frame = \""+multi_floor_manager.published_frame+"\""+"\n")

                # overwrite cartographer parameters if exist
                for cartographer_param_key in cartographer_parameters.keys():
                    if type(cartographer_parameters[cartographer_param_key]) is bool:
                        f.write(cartographer_param_key + " = " + str(cartographer_parameters[cartographer_param_key]).lower() + "\n")
                    else:
                        f.write(cartographer_param_key + " = " + str(cartographer_parameters[cartographer_param_key]) + "\n")

                # end of the config file
                f.write("return options")

            package1 = "cartographer_ros"
            executable1 = "cartographer_node"

            # run cartographer node
            node1 = roslaunch.core.Node(package1, executable1,
                                name="cartographer_node",
                                namespace = namespace,
                                remap_args = [("scan", scan_topic_name),
                                                ("points2", points2_topic_name),
                                                ("imu", imu_topic_name),
                                                ("odom", odom_topic_name),
                                                ("fix", "/"+namespace+fix_topic_name)],
                                output = "screen"
                                )
            node1.args = "-configuration_directory " + configuration_directory \
                            + " -configuration_basename " + tmp_configuration_basename \
                            + " -load_state_filename " + load_state_filename \
                            + " -start_trajectory_with_default_topics=false"
            script1 = launch.launch(node1)

            # create floor_manager
            floor_manager =  FloorManager()
            floor_manager.configuration_directory = configuration_directory
            floor_manager.configuration_basename = tmp_configuration_basename
            multi_floor_manager.ble_localizer_dict[floor][area][mode] = floor_manager

        # set values to floor_manager
        for mode in modes:
            floor_manager = multi_floor_manager.ble_localizer_dict[floor][area][mode]

            floor_manager.localizer = ble_localizer_floor
            floor_manager.wifi_localizer = wifi_localizer_floor
            floor_manager.node_id = node_id
            floor_manager.frame_id = frame_id
            floor_manager.map_filename = map_filename
            # publishers
            floor_manager.imu_pub = rospy.Publisher(node_id+"/"+str(mode)+imu_topic_name , Imu, queue_size=4000)
            floor_manager.points_pub = rospy.Publisher(node_id+"/"+str(mode)+points2_topic_name, PointCloud2, queue_size=100)
            floor_manager.initialpose_pub = rospy.Publisher(node_id+"/"+str(mode)+initialpose_topic_name, PoseWithCovarianceStamped, queue_size=10)
            floor_manager.odom_pub = rospy.Publisher(node_id+"/"+str(mode)+odom_topic_name, Odometry, queue_size=100)
            floor_manager.fix_pub = rospy.Publisher(node_id+"/"+str(mode)+fix_topic_name, NavSatFix, queue_size=10)

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

    # wait for services after launching all nodes to reduce waiting time
    for map_dict in map_list:
        floor = float(map_dict["floor"])
        area = int(map_dict["area"]) if "area" in map_dict else 0
        node_id = map_dict["node_id"]
        for mode in modes:
            floor_manager = multi_floor_manager.ble_localizer_dict[floor][area][mode]
            #rospy service
            rospy.wait_for_service(node_id+"/"+str(mode)+'/get_trajectory_states')
            rospy.wait_for_service(node_id+"/"+str(mode)+'/finish_trajectory')
            rospy.wait_for_service(node_id+"/"+str(mode)+'/start_trajectory')
            floor_manager.get_trajectory_states = rospy.ServiceProxy(node_id+"/"+str(mode)+'/get_trajectory_states', GetTrajectoryStates)
            floor_manager.finish_trajectory = rospy.ServiceProxy(node_id+"/"+str(mode)+'/finish_trajectory', FinishTrajectory)
            floor_manager.start_trajectory = rospy.ServiceProxy(node_id+"/"+str(mode)+'/start_trajectory', StartTrajectory)

    multi_floor_manager.floor_list = list(floor_set)

    # a localizer to estimate floor
    # ble floor localizer
    if multi_floor_manager.use_ble:
        samples_global_all_extracted = extract_samples(samples_global_all, key="iBeacon")
        multi_floor_manager.ble_floor_localizer = create_wireless_rss_localizer(floor_localizer_type, n_neighbors=n_neighbors_floor, min_beacons=min_beacons_floor, rssi_offset=rssi_offset)
        multi_floor_manager.ble_floor_localizer.fit(samples_global_all_extracted)
    # wifi floor localizer
    if multi_floor_manager.use_wifi:
        samples_global_all_wifi = extract_samples(samples_global_all, key="WiFi")
        multi_floor_manager.wifi_floor_localizer = create_wireless_rss_localizer(floor_localizer_type, n_neighbors=n_neighbors_floor, min_beacons=min_beacons_floor)
        multi_floor_manager.wifi_floor_localizer.fit(samples_global_all_wifi)

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
    wifi_sub = rospy.Subscriber("wifi", String, multi_floor_manager.wifi_callback, queue_size=1)
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

    # external localizer
    if use_gnss:
        multi_floor_manager.gnss_is_active = True
        multi_floor_manager.indoor_outdoor_mode = IndoorOutdoorMode.UNKNOWN
        gnss_fix_sub = message_filters.Subscriber("gnss_fix", NavSatFix)
        gnss_fix_velocity_sub = message_filters.Subscriber("gnss_fix_velocity", TwistWithCovarianceStamped)
        time_synchronizer = message_filters.TimeSynchronizer([gnss_fix_sub, gnss_fix_velocity_sub], 10)
        time_synchronizer.registerCallback(multi_floor_manager.gnss_fix_callback)
        mf_navsat_sub = rospy.Subscriber("mf_navsat", MFNavSAT, multi_floor_manager.mf_navsat_callback)
        multi_floor_manager.gnss_fix_local_pub = rospy.Publisher("gnss_fix_local", PoseWithCovarianceStamped, queue_size=10)

        #map_frame_adjust_time = rospy.Timer(rospy.Duration(0.1), multi_floor_manager.map_frame_adjust_callback) # 10 Hz

    if use_global_localizer:
        multi_floor_manager.is_active = False # deactivate multi_floor_manager
        global_localizer_global_pose_sub = rospy.Subscriber("/global_localizer/global_pose", MFGlobalPosition, multi_floor_manager.global_localizer_global_pose_callback)
        # call external global localization service
        rospy.loginfo("wait for service /global_localizer/request_localization")
        rospy.wait_for_service('/global_localizer/request_localization')
        global_localizer_request_localization = rospy.ServiceProxy('/global_localizer/request_localization', MFSetInt)
        n_compute_global_pose = 1 # request computing global_pose once
        resp = global_localizer_request_localization(n_compute_global_pose)
        rospy.loginfo("called /global_localizer/request_localization")

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

    multi_floor_manager.set_localize_status(MFLocalizeStatus.UNKNOWN)
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

        if use_gnss:
            multi_floor_manager.publish_map_frame_adjust_tf()

        multi_floor_manager.spin_count += 1

        r.sleep()
