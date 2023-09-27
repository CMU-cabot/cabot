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
from rosidl_runtime_py import message_to_yaml

from datetime import datetime, timedelta
import pytz

import logging
logging.basicConfig(level=logging.INFO)

parser = OptionParser(usage="""
print/plot topics
Example
{0} -f <bag file> -i                      # show bag info (similar to ros2 bag info)
{0} -f <bag file> -t <topic>              # print topic
{0} -f <bag file> -t <topic> -1           # print topic once
{0} -f <bag file> -t <topic> -y           # print topic in yaml format (default csv)
{0} -f <bag file> -t <topic> -p data      # print topic and plot "data" of the message (support only one topic)
""".format(sys.argv[0]))

parser.add_option('-f', '--file', type=str, help='bag file to print')
parser.add_option('-s', '--start', type=float, help='start time from the begining', default=0.0)
parser.add_option('-d', '--duration', type=float, help='duration from the start time', default=99999999999999)
parser.add_option('-t', '--topic', type=str, action='append', default=[], help='topics to be printed')
parser.add_option('-p', '--plot', type=str, default="", help='plot data')
parser.add_option('-i', '--info', action='store_true', help='print info')
parser.add_option('-1', '--once', action='store_true', help='print only one message')
parser.add_option('-y', '--yaml', action='store_true', help='print message in yaml')
parser.add_option('-r', '--raw', action='store_true', help='print message only without topic name and time')
parser.add_option('-T', '--timezone', type=int, help='set timezone default=0', default=0)


(options, args) = parser.parse_args()

if not options.file:
    parser.print_help()
    sys.exit(0)

logging.info(options)
bagfilename = options.file
reader = BagReader(bagfilename)

if options.info:
    for info in sorted(reader.topic_types, key=lambda x: x.name):
        print(f"{info.name:80s}{info.type}")

    meta = reader.info
    print(f"Bag Size:   {meta.bag_size / 1024 / 1024:.2f} MB")
    print(f"Start Time: {meta.starting_time}")
    print(f"Duration:   {meta.duration} ({meta.duration.seconds:.2f} seconds)")
    
    sys.exit(0)

if not options.topic:
    parser.print_help()
    sys.exit(0)

reader.set_filter_by_topics(options.topic)
reader.set_filter_by_options(options)  # filter by start and duration

ts = []
ds = []

while reader.has_next():
    try:
        (topic, msg, t, st) = reader.serialize_next()
    except:
        continue
    if not topic:
        continue
    dt_object_utc = datetime.utcfromtimestamp(t).replace(tzinfo=pytz.utc)
    dt_object_jst = dt_object_utc + timedelta(hours=options.timezone)

    if options.raw:
        if options.yaml:
            print(f"{message_to_yaml(msg)}")
        else:
            print(f"{message_to_csv(msg)}")
    else:
        if options.yaml:
            print(f"[{topic}] {dt_object_jst} {t:.2f}({st:.2f}): \n{message_to_yaml(msg)}")
        else:
            print(f"[{topic}] {dt_object_jst} {t:.2f}({st:.2f}): {message_to_csv(msg)}")


    if options.once:
        break

    if options.plot:
        if hasattr(msg, options.plot):
            ts.append(st)
            ds.append(getattr(msg, options.plot))
        else:
            logging.error(f"cannot get attribute {options.plog} in {msg}")


if options.plot:
    plt.plot(ts, ds)
    plt.show()
