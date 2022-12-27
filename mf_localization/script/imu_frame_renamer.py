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

import rospy
from sensor_msgs.msg import Imu


class IMUFrameRenamer:
    def __init__(self, frame_id):
        self._frame_id = frame_id
        self._prev_time = None
        self.pub = None
        self.sub = None

    def imu_callback(self, msg):
        msg.header.frame_id = self._frame_id
        if self._prev_time is None:
            self._prev_time = msg.header.stamp
        elif msg.header.stamp <= self._prev_time:
            return # skip
        self.pub.publish(msg)
        self._prev_time = msg.header.stamp

def main():
    rospy.init_node("imu_frame_renamer")
    frame_id = rospy.get_param("~frame_id", "imu")
    imu_renamer = IMUFrameRenamer(frame_id)
    imu_renamer.pub = rospy.Publisher("imu_out", Imu, queue_size=4000)
    imu_renamer.sub = rospy.Subscriber("imu_in", Imu, imu_renamer.imu_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
