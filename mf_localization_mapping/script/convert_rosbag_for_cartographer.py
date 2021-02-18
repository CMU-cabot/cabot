#!/usr/bin/env python

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
# -*- coding: utf-8 -*-

import json
import argparse
import copy

import rosbag


def main():
    parser = argparse.ArgumentParser("convert imu frame name to 'imu'")
    parser.add_argument("-i","--input", required=True)
    parser.add_argument("-o","--output", required=True)
    args = parser.parse_args()

    input_bag = args.input
    output_bag = args.output

    if input_bag == output_bag:
        raise RuntimeError("input == output")
        return

    outbag = rosbag.Bag(output_bag, 'w')

    with rosbag.Bag(input_bag) as bag:
        for topic, msg, t in bag.read_messages():

            if topic == "/imu/data":
                msg.header.frame_id = "imu"

            if topic in ["/velodyne_points", "/imu/data", "/beacons", "/wireless/beacons", "/wireless/wifi"]:
                outbag.write(topic, msg, t)
    outbag.close()

if __name__ == "__main__":
    main()
