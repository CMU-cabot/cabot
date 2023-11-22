#!/usr/bin/env python

# Copyright (c) 2023  Carnegie Mellon University, IBM Corporation, and others
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
"""
record_data.py

Records the accumulated occupancy gridmap and robot pose inorder to evaluate the intersection detector.
"""

import rosbag
import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import LaserScan

from tf.transformations import quaternion_multiply, euler_from_quaternion
import traceback

import math
import datetime

class data_recorder():
    def __init__(self):
        self._distance_threshold = 0.5 #[m]
        self._angle_threshold = math.pi / 6 #[rad]

        self._global_map_name = rospy.get_param("~global_map_name", "map")
        self._file_name = rospy.get_param("~bagfile", "")
        if not self._file_name:
            self._file_name = "explore-%s"%(datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))

        if not self._file_name.endswith(".bag"):
            self._file_name += ".bag"
        
        self._map = None
        self._scan = None
        self._prev_local_odometry = None

        self._tf2_buffer = tf2_ros.Buffer()
        self._tf2_listener = tf2_ros.TransformListener(self._tf2_buffer)

        rospy.Subscriber("/map", OccupancyGrid, self._map_callback, None)
        rospy.Subscriber("/cabot_explore/scan", LaserScan, self._scan_callback, None)
        rospy.Subscriber("/odom", Odometry, self._odometry_callback, None)

        with rosbag.Bag(self._file_name, 'w') as outbag:
            pass

    def get_current_pose(self):
        while not rospy.is_shutdown():
            try:
                t = self._tf2_buffer.lookup_transform("map", "base_footprint", rospy.Time(0), rospy.Duration(1.0))

                current_pose = Pose()
                current_pose.position.x = t.transform.translation.x
                current_pose.position.y = t.transform.translation.y
                current_pose.position.z = t.transform.translation.z
                current_pose.orientation.x = t.transform.rotation.x
                current_pose.orientation.y = t.transform.rotation.y
                current_pose.orientation.z = t.transform.rotation.z
                current_pose.orientation.w = t.transform.rotation.w
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.loginfo("Transform from map to robot is not ready")
            rospy.sleep(0.01)
        return current_pose

    def _record_data(self, map, scan, odometry):
        pose = self.get_current_pose()
        rospy.loginfo('record now ' + datetime.datetime.now().strftime(' Time %H hour %M min %S sec'))
        with rosbag.Bag(self._file_name, 'a') as outbag:
            outbag.write("/map", map, odometry.header.stamp)
            outbag.write("/cabot_explore/scan", scan, odometry.header.stamp)
            outbag.write("/odom", odometry, odometry.header.stamp)
            outbag.write("/world_pose", pose, odometry.header.stamp)

    def _map_callback(self, msg: OccupancyGrid):
        self._map = msg

    def _scan_callback(self, msg: LaserScan):
        self._scan = msg

    def _odometry_callback(self, msg: Odometry):
        now_local_odometry = msg

        if self._prev_local_odometry is None:
            self._prev_local_odometry = now_local_odometry

        if self._diff_distance(now_local_odometry, self._prev_local_odometry) > self._distance_threshold or self._diff_angle(now_local_odometry, self._prev_local_odometry) > self._angle_threshold:
            self._record_data(self._map, self._scan, now_local_odometry)
            self._prev_local_odometry = now_local_odometry

    def _diff_distance(self, p1: Odometry, p2: Odometry):
        deltaX_squared = (p1.pose.pose.position.x - p2.pose.pose.position.x) ** 2
        deltaY_squared = (p1.pose.pose.position.y - p2.pose.pose.position.y) ** 2
        deltaZ_squared = (p1.pose.pose.position.z - p2.pose.pose.position.z) ** 2
        distance = math.sqrt(deltaX_squared + deltaY_squared + deltaZ_squared)
        return distance

    def _diff_angle(self, p1: Odometry, p2: Odometry):
        quat1 = [p1.pose.pose.orientation.x, p1.pose.pose.orientation.y, p1.pose.pose.orientation.z, p1.pose.pose.orientation.w]
        quat2 = [p2.pose.pose.orientation.x, p2.pose.pose.orientation.y, p2.pose.pose.orientation.z, p2.pose.pose.orientation.w]
        quat3 = quaternion_multiply(quat2, self._q_inverse(quat1))
        _, _, yaw1 = euler_from_quaternion(quat3)
        return abs(yaw1)

    def _q_inverse(self, q):
        return [q[0], q[1], q[2], -q[3]]


if __name__ == "__main__":
    rospy.init_node("data_recorder", log_level=rospy.DEBUG)
    try:
        data_recorder()
    except:
        rospy.logerr(traceback.format_exc())
    rospy.spin()
