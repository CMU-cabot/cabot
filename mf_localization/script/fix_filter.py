#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2022  IBM Corporation
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

from enum import IntEnum
import numpy as np
import rospy

from std_msgs.msg import Int8, Int64
from sensor_msgs.msg import NavSatFix, NavSatStatus

class FixFilterNode:
    def __init__(self, status_threshold, stdev_threshold):
        self.status_threshold = status_threshold
        self.stdev_threshold = stdev_threshold
        self.fix_sub = rospy.Subscriber("fix", NavSatFix, self.fix_callback)
        self.fix_pub = rospy.Publisher("fix_filtered", NavSatFix, queue_size=10)

    def fix_callback(self, msg: NavSatFix):
        navsat_status = msg.status
        status = navsat_status.status
        position_covariance = np.reshape(msg.position_covariance, (3,3))
        stdev = np.sqrt(position_covariance[0,0])

        if self.status_threshold <= status and stdev <= self.stdev_threshold:
            msg.header.stamp = rospy.Time.now()
            msg.altitude = 0.0
            self.fix_pub.publish(msg)
def main():
    rospy.init_node("fix_filter")
    status_threshold = rospy.get_param("~status_threshold",2)
    stdev_threshold = rospy.get_param("~stdev_threshold",0.1)

    fix_filter_node = FixFilterNode(status_threshold, stdev_threshold)

    rospy.spin()

if __name__ == "__main__":
    main()
