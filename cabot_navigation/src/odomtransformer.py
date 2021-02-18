#!/usr/bin/env python

# Copyright (c) 2020 Carnegie Mellon University
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
import numpy as np
import math
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg
from copy import deepcopy

# constants
ALPHA = 0.1
TIME_WINDOW = 0.2

# global variables
odoms = []
position = np.array([0, 0, 0])
orientation = np.array([0, 0, 0, 1])


# interporate two quaternion
def getSlerp(o1, o2):
        temp = tf.transformations.quaternion_slerp(o1, o2, ALPHA)
        return np.array(temp)


# convert ros messgae into numpy array
def toArray(v):
        if isinstance(v, geometry_msgs.msg.Quaternion):
                return np.array([v.x, v.y, v.z, v.w])
        if isinstance(v, geometry_msgs.msg.Point):
                return np.array([v.x, v.y, v.z])
        return None


# filter pose
def filterPose(pose):
        global position, orientation
        position = position * (1-ALPHA) + toArray(pose.position) * ALPHA
        orientation = getSlerp(orientation, toArray(pose.orientation))


# calculate velocity
def getVelocity(odoms):
        odom1 = odoms[0]
        odom2 = odoms[-1]

        # difference of time
        dt = odom2.header.stamp.to_sec() - odom1.header.stamp.to_sec()

        pose1 = odom1.pose.pose
        pose2 = odom2.pose.pose

        # calculate difference between two points
        pos1 = toArray(pose1.position)
        pos2 = toArray(pose2.position)
        dp = pos2 - pos1

        # calculate difference of angle q2 = qr * q1
        q1_inv = toArray(pose1.orientation) * np.array([1, 1, 1, -1])
        q2 = toArray(pose2.orientation)
        qr = tf.transformations.quaternion_multiply(q2, q1_inv)
        # convert quaternion to rpy
        rpy = tf.transformations.euler_from_quaternion(qr)

        # create Twist message
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = round(np.linalg.norm(dp) / dt, 2)
        twist.angular.z = round(rpy[2] / dt, 2)
        # assumes zero for others
        return twist


def publishTransform(data, odomInput, tfOutput):
        tfBroadcaster = tf2_ros.TransformBroadcaster()

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = odomInput
        t.child_frame_id = tfOutput
        t.transform.translation = data.pose.pose.position
        t.transform.rotation = data.pose.pose.orientation

        tfBroadcaster.sendTransform(t)


def callback(data, args):
        global odoms, position, orientation
        (odomInput, tfOutput, odomPublisher) = args
        
        publishTransform(data, odomInput, tfOutput)

        # append the latest odometry
        odoms.append(deepcopy(data))
        if len(odoms) > 40 * TIME_WINDOW:
                odoms.pop(0)

        # filter it
        filterPose(data.pose.pose)

        # use filtered position for odometry
        data.pose.pose.position = geometry_msgs.msg.Point(*position)
        data.pose.pose.orientation = data.pose.pose.orientation
        if len(odoms) > 1:
                data.twist.twist = getVelocity(odoms)
                odomPublisher.publish(data)


if __name__ == "__main__":
        rospy.init_node("odomtransformer")

        odomInput = rospy.get_param("~odom_input")
        odomOutput = rospy.get_param("~odom_output")
        tfOutput = rospy.get_param("~tf_output")

        odomPublisher = rospy.Publisher(odomOutput, nav_msgs.msg.Odometry,
                                        queue_size=10)
        rospy.Subscriber(odomInput, nav_msgs.msg.Odometry, callback,
                         [odomInput, tfOutput, odomPublisher])


rospy.spin()
