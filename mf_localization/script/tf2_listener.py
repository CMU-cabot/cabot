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

import rclpy
import rclpy.time
import rclpy.duration
import tf2_ros
from tf2_msgs.msg import TFMessage
import traceback


class TFMapper:
    def __init__(self):
        self.previous_timestamp = None
        self.data_list = []

        self.previous_error_time = rclpy.time.Time(seconds=0, nanoseconds=0, clock_type=node.get_clock().clock_type)
        self.error_interval = 1.0

    def tf_callback(self, msg):
        try:
            t = tfBuffer.lookup_transform(frame_id, child_frame_id, node.get_clock().now())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rostime = node.get_clock().now()

            if rclpy.duration.Duration(seconds=self.error_interval) < rostime - self.previous_error_time:
                node.get_logger().error('LookupTransform Error')
                self.previous_error_time = rostime
            return

        timestamp = float(t.header.stamp.sec) + float(t.header.stamp.nanosec)*1.0e-9

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

        node.get_logger().info(str(data))


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("tf2_listener")

    tfBuffer = tf2_ros.Buffer(node=node)
    listener = tf2_ros.TransformListener(tfBuffer, node)

    frame_id = node.declare_parameter("frame_id", "map").value
    child_frame_id = node.declare_parameter("child_frame_id", "base_link").value
    output = node.declare_parameter("output", "").value

    tf_mapper = TFMapper()
    tf_sub = node.create_subscription(TFMessage, "/tf", tf_mapper.tf_callback, 10)

    def shutdown_hook():
        if output is not None and 0 < len(tf_mapper.data_list):
            print("wirinting to output")
            with open(output, "w") as f:
                writer = csv.writer(f, lineterminator="\n")
                writer.writerow(["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"])
                writer.writerows(tf_mapper.data_list)

    try:
        rclpy.spin(node)
    except:  # noqa: E722
        traceback.print_exc()
        shutdown_hook()
