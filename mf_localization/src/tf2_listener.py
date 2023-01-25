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

import csv
import numpy as np

import rospy
import tf2_ros
import tf_conversions
from tf2_msgs.msg import TFMessage

class TFMapper:
    def __init__(self):
        self.previous_timestamp = None
        self.data_list = []

        self.previous_error_time = 0
        self.error_interval = 1.0

    def tf_callback(self, msg):
        try:
            t = tfBuffer.lookup_transform(frame_id, child_frame_id, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rostime = rospy.get_time()

            if self.error_interval < rostime - self.previous_error_time:
                rospy.logerr('LookupTransform Error')
                self.previous_error_time = rostime
            return

        timestamp = float(t.header.stamp.secs) + float(t.header.stamp.nsecs)*1.0e-9

        # check if frame_id -> child_frame_id was updated.
        if self.previous_timestamp is None:
            self.previous_timestamp = timestamp
        else:
            if self.previous_timestamp == timestamp:
                # frame_id -> child_frame_id was not updated.
                return
            else:
                self.previous_timestamp = timestamp
                
        x = t.transform.translation.x
        y = t.transform.translation.y
        z = t.transform.translation.z
        qx = t.transform.rotation.x
        qy = t.transform.rotation.y
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w

        data = [timestamp, x, y, z, qx, qy, qz, qw]
        self.data_list.append(data)

        rospy.loginfo(data)

if __name__ == "__main__":
    rospy.init_node("tf2_listener")

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    frame_id = rospy.get_param("~frame_id", "map")
    child_frame_id = rospy.get_param("~child_frame_id", "base_link")
    output = rospy.get_param("~output", None)

    tf_mapper = TFMapper()
    tf_sub = rospy.Subscriber("/tf", TFMessage, tf_mapper.tf_callback)

    def shutdown_hook():
        if output is not None and 0<len(tf_mapper.data_list):
            with open(output, "w") as f:
                writer = csv.writer(f, lineterminator="\n")
                writer.writerow(["timestamp","x","y","z","qx","qy","qz","qw"])
                writer.writerows(tf_mapper.data_list)

    rospy.on_shutdown(shutdown_hook)

    rospy.spin()