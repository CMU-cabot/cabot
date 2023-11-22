#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

import rospy
from visualization_msgs.msg import Marker, MarkerArray


def create_rviz_marker(ns, id, marker_type, position=None, scale_x=0.2, scale_y=0.2, scale_z=0.2, color_a=1.0, color_r=0.0, color_g=0.0, color_b=0.0, text=None):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = id
    marker.type = marker_type
    marker.action = Marker.MODIFY
    if text is not None:
        marker.text = text
    if position is not None:
        marker.pose.position = position
    marker.scale.x = scale_x
    marker.scale.y = scale_y
    marker.scale.z = scale_z
    marker.color.a = color_a
    marker.color.r = color_r
    marker.color.g = color_g
    marker.color.b = color_b
    return marker