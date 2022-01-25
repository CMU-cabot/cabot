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
import argparse
import copy

import rosbag


def main():
    parser = argparse.ArgumentParser("convert imu frame name to 'imu'")
    parser.add_argument("-i","--input", required=True, help="input bag file")
    parser.add_argument("-o","--output", required=True, help="output bag file")
    parser.add_argument("--imu_topic", default="/imu/data")
    parser.add_argument("--imu_frame", default="imu")
    parser.add_argument("--fix_imu_timestamp", default=False, action="store_true")
    args = parser.parse_args()

    input_bag = args.input
    output_bag = args.output
    imu_topic = args.imu_topic
    imu_frame = args.imu_frame
    fix_imu_timestamp = args.fix_imu_timestamp

    if input_bag == output_bag:
        raise RuntimeError("input == output")
        return

    outbag = rosbag.Bag(output_bag, 'w')

    with rosbag.Bag(input_bag) as bag:
        for topic, msg, t in bag.read_messages():

            if topic == imu_topic:
                # replace imu_frame if necessary
                if imu_frame != "":
                    msg.header.frame_id = imu_frame

                # imu value check
                ax = msg.linear_acceleration.x
                ay = msg.linear_acceleration.y
                az = msg.linear_acceleration.z
                amp = (ax**2 + ay**2 + az**2)**0.5
                if amp == 0.0:
                    print("skipped invalid imu value. (linear_accleration==0)")
                    continue

            if topic in ["/velodyne_points","/beacons", "/wireless/beacons", "/wireless/wifi"]:
                outbag.write(topic, msg, t)
            elif topic ==  imu_topic:
                # replace imu message timestamps with header timestamps if necessary
                if fix_imu_timestamp:
                    outbag.write(topic, msg, msg.header.stamp)
                else:
                    outbag.write(topic, msg, t)

    outbag.close()

if __name__ == "__main__":
    main()
