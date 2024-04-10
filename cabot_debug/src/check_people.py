#!/usr/bin/env python3

# Copyright (c) 2020, 2023  Carnegie Mellon University
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


import math
import os
import sys
from optparse import OptionParser

import rclpy.time
from cabot_common.rosbag2 import BagReader
from matplotlib import pyplot as plt
from tf_bag import BagTfTransformer

parser = OptionParser(
    usage="""
Plot odometry data

Example
{0} -f <bag file>                        # plot odometry in x-y coordinates
{0} -f <bag file> -t                     # plot odometry distance in timeline
""".format(
        sys.argv[0]
    )
)

parser.add_option("-f", "--file", type=str, help="bag file to plot")
parser.add_option("-t", "--timeline", action="store_true", help="plot distance and time")
parser.add_option("-s", "--start", type=float, help="start time from the begining", default=0.0)
parser.add_option("-d", "--duration", type=float, help="duration from the start time", default=99999999999999)
parser.add_option("-c", "--cmd_vel", action="store_true", help="plot cmd_vel (only with --timeline)")


(options, args) = parser.parse_args()

if not options.file:
    parser.print_help()
    sys.exit(0)

bagfilename = options.file
reader = BagReader(bagfilename)

reader2 = BagReader(bagfilename)
btf = BagTfTransformer(reader2)

reader.set_filter_by_topics(
    [
        "/people",
        "/rs1/depth/metadata",
        "/rs2/depth/metadata",
        "/rs3/depth/metadata",
    ]
)
reader.set_filter_by_options(options)  # filter by start and duration

last_time = {"people": 0, "rs1": 0, "rs2": 0, "rs3": 0}

bag_duration = reader.bag_duration()

print("bag_duration:", bag_duration)

while reader.has_next():
    (topic, msg, t, st) = reader.serialize_next()
    if not topic:
        continue

    if topic == "/people":
        last_time["people"] = st
    elif topic == "/rs1/depth/metadata":
        last_time["rs1"] = st
    elif topic == "/rs2/depth/metadata":
        last_time["rs2"] = st
    elif topic == "/rs3/depth/metadata":
        last_time["rs3"] = st


print(last_time)

result = {"people": None, "rs1": None, "rs2": None, "rs3": None}

for key, value in last_time.items():
    if bag_duration - value < 5.0:
        result[key] = "OK"
    elif value != 0:
        result[key] = "Down"

print(result)
