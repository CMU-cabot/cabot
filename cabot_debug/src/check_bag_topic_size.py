#!/usr/bin/env python3

###############################################################################
# Copyright (c) 2019, 2022  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
###############################################################################

import multiprocessing
import os
import re
import sys
import traceback
from bisect import bisect
from optparse import OptionParser

import numpy
import yaml
from cabot_common.rosbag2 import BagReader
from matplotlib import pyplot as plt

parser = OptionParser(
    usage="""
Example
{0} -f <bag file>                       # bagfile
{0} -f <bag file> -v                    # output all topics
{0} -f <bag file> -t                    # analyze tf
""".format(
        sys.argv[0]
    )
)

parser.add_option("-f", "--file", type=str, help="bag file to be processed")
parser.add_option("-v", "--verbose", action="store_true", help="output all topics")
parser.add_option("-c", "--count", action="store_true", help="sort by count")
parser.add_option("-t", "--tf", action="store_true", help="analyze tf")

(options, args) = parser.parse_args()

if not options.file:
    parser.print_help()
    sys.exit(0)

bagfilename = options.file
reader = BagReader(bagfilename)

if options.tf:
    reader.set_filter_by_topics(["/tf"])

    tf_count = {}
    max_diff = 0
    reader.set_filter_by_options(options)  # filter by start and duration

    while reader.has_next():
        (topic, msg, t, st) = reader.serialize_next()

        for tf in msg.transforms:
            tft = tf.header.stamp.sec + tf.header.stamp.nanosec / 1e9
            if max_diff < (t - tft):
                print(f"{t - tft} {tf.header.frame_id}-{tf.child_frame_id}")
                max_diff = t - tft
            key = f"{tf.header.frame_id}-{tf.child_frame_id}"
            if key not in tf_count:
                tf_count[key] = 0
            tf_count[key] += 1
    print(f"max_diff={max_diff}")
    for i, (k, v) in enumerate(tf_count.items()):
        print(f"{k}: {v}")

    sys.exit(0)


total = 0
sizes = {}
counts = {}
start = {}
end = {}

while reader.has_next():
    (topic, msg_data, t) = reader.read_next()
    if "__all__" not in start:
        start["__all__"] = t
    end["__all__"] = t
    if topic not in sizes:
        if topic not in start:
            start[topic] = t
        sizes[topic] = 0
        counts[topic] = 0
    end[topic] = t
    sizes[topic] += len(msg_data)
    counts[topic] += 1
    total += len(msg_data)

if options.count:
    sorted_dict = sorted(counts.items(), key=lambda item: item[1], reverse=True)
else:
    sorted_dict = sorted(sizes.items(), key=lambda item: item[1], reverse=True)

total = total / 1024 / 1024
duration = (end["__all__"] - start["__all__"]) / 1e9
rate = total / duration
print(f"{total:10.2f} MB in {duration:.2f} secs, {rate:10.2f} MB/s")
for k, v in sorted_dict:
    v = sizes[k]
    c = counts[k]
    d = (end[k] - start[k]) / 1e9
    if d == 0:
        d = 1

    if 1024 * 1024 < v:
        f = 1024 * 1024
        u = "MB"
    elif 1024 < v:
        f = 1024
        u = "KB"
    else:
        f = 1
        u = "B"

    if f == 1024 * 1024 or options.verbose:
        print(f"{v/f:10.2f} {u} \t{c:10d}\t{d:8.2f}\t{c/d:8.2f}Hz\t{k}")
