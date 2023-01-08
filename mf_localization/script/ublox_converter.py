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

import numpy as np
import rclpy

from std_msgs.msg import Int8, Int64
from ublox_msgs.msg import NavSAT
from mf_localization_msgs.msg import MFNavSAT


class UbloxConverter:
    def __init__(self, min_cno, min_elev, num_sv_threshold_low, num_sv_threshold_high):
        self.min_cno = min_cno
        self.min_elev = min_elev
        self.num_sv_threshold_low = num_sv_threshold_low
        self.num_sv_threshold_high = num_sv_threshold_high

    def count_active_sv(self, sv):
        count = 0
        for s in sv:
            cno = s.cno
            elev = s.elev
            if self.min_cno <= cno and self.min_elev <= np.abs(elev):
                count += 1
        return count

    def convert_count_to_status(self, count):
        if count < self.num_sv_threshold_low:
            return MFNavSAT.STATUS_INACTIVE
        elif count < self.num_sv_threshold_high:
            return MFNavSAT.STATUS_INTERMEDIATE
        else:
            return MFNavSAT.STATUS_ACTIVE


class UbloxConverterNode:
    def __init__(self, node, min_cno, min_elev, num_sv_threshold_low, num_sv_threshold_high):
        self.node = node
        self.ublox_converter = UbloxConverter(min_cno, min_elev, num_sv_threshold_low, num_sv_threshold_high)
        self.navsat_sub = self.node.create_subscription(NavSAT, "navsat", self.navsat_callback, 10)
        self.num_active_sv_pub = self.node.create_publisher(Int64, "num_active_sv", 10)
        self.status_pub = self.node.create_publisher(Int8, "sv_status", 10)
        self.mf_navsat_pub = self.node.create_publisher(MFNavSAT, "mf_navsat", 10)

    def navsat_callback(self, msg: NavSAT):
        num_sv = msg.numSvs

        num_active_sv = self.ublox_converter.count_active_sv(msg.sv)
        sv_status = self.ublox_converter.convert_count_to_status(num_active_sv)

        count_msg = Int64(num_active_sv)
        self.num_active_sv_pub.publish(count_msg)

        status_msg = Int8(sv_status)
        self.status_pub.publish(status_msg)

        mf_navsat_msg = MFNavSAT()
        mf_navsat_msg.num_sv = num_sv
        mf_navsat_msg.num_active_sv = num_active_sv
        mf_navsat_msg.sv_status = sv_status
        self.mf_navsat_pub.publish(mf_navsat_msg)


def main():
    rclpy.init()
    node = rclpy.create_node("ublox_converter")
    min_cno = node.declare_parameter("min_cno", 30).value
    min_elev = node.declare_parameter("min_elev", 15).value
    num_sv_threshold_low = node.declare_parameter("num_sv_threshold_low", 5).value
    num_sv_threshold_high = node.declare_parameter("num_sv_threshold_high", 10).value

    UbloxConverterNode(node, min_cno, min_elev, num_sv_threshold_low, num_sv_threshold_high)

    rclpy.spin(node)

import sys
import signal
def receiveSignal(signal_num, frame):
    print("Received:", signal_num)
    sys.exit(0)

signal.signal(signal.SIGINT, receiveSignal)

if __name__ == "__main__":
    main()
