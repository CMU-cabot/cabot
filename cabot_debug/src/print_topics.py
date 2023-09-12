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


from optparse import OptionParser
import math
import os
import sys
import rclpy.time
from matplotlib import pyplot as plt

from cabot_common.rosbag2 import BagReader
from tf_bag import BagTfTransformer
from rosidl_runtime_py import message_to_csv

import logging
logging.basicConfig(level=logging.INFO)

parser = OptionParser(usage="""
Plot odometry data

Example
{0} -f <bag file>                        # plot IMU
""".format(sys.argv[0]))

parser.add_option('-f', '--file', type=str, help='bag file to plot')
parser.add_option('-s', '--start', type=float, help='start time from the begining', default=0.0)
parser.add_option('-d', '--duration', type=float, help='duration from the start time', default=99999999999999)
parser.add_option('-t', '--topic', type=str, action='append', default=[], help='topics to be printed')

(options, args) = parser.parse_args()

if not options.file or not options.topic:
    parser.print_help()
    sys.exit(0)

logging.info(options)
bagfilename = options.file
reader = BagReader(bagfilename)

reader.set_filter_by_topics(options.topic)
reader.set_filter_by_options(options)  # filter by start and duration

while reader.has_next():
    (topic, msg, t, st) = reader.serialize_next()
    if not topic:
        continue

    print(f"[{topic}] {t:.2f}({st:.2f}): {message_to_csv(msg)}")
