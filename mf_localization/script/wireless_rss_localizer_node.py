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

import json
import ast

import numpy as np

import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Point, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from mf_localization.wireless_utils import extract_samples

from wireless_rss_localizer import SimpleRSSLocalizer


class RSSLocalizerNode:
    def __init__(self, node, wifi_localizer, ble_localizer):
        self.node = node
        self._wifi_localizer = wifi_localizer
        self._ble_localizer = ble_localizer

        self._seq_pose = 0
        self._wifi_pose_pub = self.node.create_publisher(PoseWithCovarianceStamped, '/pose_fix', 10)

    def beacons_callback(self, message):
        # p rint("/wireless/beacons")
        data = json.loads(message.data)
        self.process_beacons(data)

    def wifi_callback(self, message):
        # print("/wireless/wifi")
        data = json.loads(message.data)
        self.process_wifi(data)

    def process_wifi(self, data):

        loc = self._wifi_localizer.predict(data["data"])
        if loc is None:
            return

        point = Point()
        point.x = loc[0, 0]
        point.y = loc[0, 1]
        point.z = loc[0, 2]

        orientation = Quaternion()
        orientation.x = 0
        orientation.y = 0
        orientation.z = 0
        orientation.w = 1

        pose_cov_stamped = PoseWithCovarianceStamped()
        pose_cov_stamped.header.seq = self._seq_pose
        pose_cov_stamped.header.stamp = self.clock.now()
        pose_cov_stamped.header.frame_id = "map"

        pose_cov_stamped.pose.pose.position = point
        pose_cov_stamped.pose.pose.orientation = orientation

        covariance = 7.0*7.0*np.eye(6)
        covariance[2, 2] = 0.1  # z
        covariance[3, 3] = 0.0  # x-axis
        covariance[4, 4] = 0.0  # y-axis
        covariance[5, 5] = 100.0  # z-axis

        pose_cov_stamped.pose.covariance = list(covariance.flatten())

        self._wifi_pose_pub.publish(pose_cov_stamped)
        self._seq_pose += 1

    def process_beacons(self, data):

        loc = self._ble_localizer.predict(data["data"])
        if loc is None:
            return

        point = Point()
        point.x = loc[0, 0]
        point.y = loc[0, 1]
        point.z = loc[0, 2]

        orientation = Quaternion()
        orientation.x = 0
        orientation.y = 0
        orientation.z = 0
        orientation.w = 1

        pose_cov_stamped = PoseWithCovarianceStamped()
        pose_cov_stamped.header.seq = self._seq_pose
        pose_cov_stamped.header.stamp = self.clock.now()
        pose_cov_stamped.header.frame_id = "map"

        pose_cov_stamped.pose.pose.position = point
        pose_cov_stamped.pose.pose.orientation = orientation

        covariance = 3.0*3.0*np.eye(6)
        covariance[2, 2] = 0.1  # z
        covariance[3, 3] = 0.0  # x-axis
        covariance[4, 4] = 0.0  # y-axis
        covariance[5, 5] = 100.0  # z-axis

        pose_cov_stamped.pose.covariance = list(covariance.flatten())

        self._wifi_pose_pub.publish(pose_cov_stamped)
        self._seq_pose += 1


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node('wireless_rss_localizer')

    sub_topics_str = node.declare_parameter("topics", "['/beacons','/wireless/beacons','/wireless/wifi']").value
    sub_topics = ast.literal_eval(sub_topics_str)

    samples_file = node.declare_parameter("samples_filename", '').value

    with open(samples_file) as f:
        samples = json.load(f)

    beacons_samples = extract_samples(samples, key="iBeacon")
    wifi_samples = extract_samples(samples, key="WiFi")

    wifi_localizer = SimpleRSSLocalizer(n_neighbors=3)
    wifi_localizer.fit(wifi_samples)

    ble_localizer = SimpleRSSLocalizer(n_neighbors=3)
    ble_localizer.fit(beacons_samples)

    localizer = RSSLocalizerNode(node, wifi_localizer, ble_localizer)

    for sub_topic in sub_topics:
        if sub_topic[0] == "/":
            sub_topic = sub_topic[1:]

        if sub_topic in ["wireless/beacons", "beacons"]:
            beacons_sub = node.create_subscription(String, sub_topic, localizer.beacons_callback, 10)
        elif sub_topic in ["wireless/wifi"]:
            wifi_sub = node.create_subscription(String, sub_topic, localizer.wifi_callback, 10)
        else:
            raise RuntimeError("unknown topic (" + sub_topic + ") is requested to be subscribed.")

    try:
        rclpy.spin(node)
    except:  # noqa: E722
        pass
