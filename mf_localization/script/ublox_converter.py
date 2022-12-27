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
    def __init__(self, min_cno, min_elev, num_sv_threshold_low, num_sv_threshold_high):
        self.ublox_converter = UbloxConverter(min_cno, min_elev, num_sv_threshold_low, num_sv_threshold_high)
        self.navsat_sub = rospy.Subscriber("navsat", NavSAT, self.navsat_callback)
        self.num_active_sv_pub = rospy.Publisher("num_active_sv", Int64, queue_size=10)
        self.status_pub = rospy.Publisher("sv_status", Int8, queue_size=10)
        self.mf_navsat_pub = rospy.Publisher("mf_navsat", MFNavSAT, queue_size=10)

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
    rospy.init_node("ublox_converter")
    min_cno = rospy.get_param("~min_cno", 30)
    min_elev = rospy.get_param("~min_elev", 15)
    num_sv_threshold_low = rospy.get_param("~num_sv_threshold_low", 5)
    num_sv_threshold_high = rospy.get_param("~num_sv_threshold_high", 10)

    ublox_converter_node = UbloxConverterNode(min_cno, min_elev, num_sv_threshold_low, num_sv_threshold_high)

    rospy.spin()

if __name__ == "__main__":
    main()