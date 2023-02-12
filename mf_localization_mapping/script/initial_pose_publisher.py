#!/usr/bin/env python
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

import numpy as np

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped


class TrajectoryRestarter:
    def __init__(self):
        self._count_initialize = 0  # default = 0
        self._count = 0
        self.cov2stdev = True

        self.pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=100)

    def pose_fix_callback(self, message):
        # print("pose_fix_callback")
        if self._count == self._count_initialize:
            print("cov2stdev="+str(self.cov2stdev))
            if self.cov2stdev:
                message.pose.covariance = np.sqrt(message.pose.covariance)

            self.pub.publish(message)
            print("published /initialpose")
        self._count += 1


if __name__ == "__main__":
    rospy.init_node("trajectory_restarter")
    trajectory_restarter = TrajectoryRestarter()

    trajectory_restarter.cov2stdev = rospy.get_param("~covariance2stdev", False)

    sub = rospy.Subscriber("/pose_fix", PoseWithCovarianceStamped, trajectory_restarter.pose_fix_callback)

    print("start trajectory restarter")

    rospy.spin()
