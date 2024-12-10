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


import rclpy
from rclpy.qos import QoSProfile
import yaml
from optparse import OptionParser
import math
import os
import sys
import rclpy.time
from matplotlib import pyplot as plt
import functools

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
parser.add_option('-p', '--plot', type=str, action='append', default=[], help='plot data')
parser.add_option('-P', '--publish', action='store_true', help='publish topic')
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
        print(f"{info.name:80s}{reader.message_counts[info.name]:10d} {info.type}")

    meta = reader.info
    print(f"Bag Size:   {meta.bag_size / 1024 / 1024:.2f} MB")
    print(f"Start Time: {meta.starting_time}")
    if hasattr(meta.duration, "nanoseconds"):
        print(f"Duration:   {meta.duration} ({meta.duration.nanoseconds/1e9:.2f} seconds)")
    else:   # back compatibility for galactic
        print(f"Duration:   {meta.duration} ({meta.duration.seconds:.2f} seconds)")
    
    sys.exit(0)

if not options.topic:
    parser.print_help()
    sys.exit(0)

def import_class(input_str):
    import importlib
    # Split the input string and form module and class strings
    module_str, class_str = input_str.rsplit('/', 1)
    module_str = module_str.replace('/', '.')
    # Import the module dynamically
    module = importlib.import_module(module_str)
    return getattr(module, class_str)

def get_nested_attr(obj, attr):
    def _getattr(obj, attr):
        return getattr(obj, attr)
    return functools.reduce(_getattr, [obj] + attr.split('.'))

node = None
pubs = {}
if options.publish:
    rclpy.init()
    node = rclpy.node.Node("print_topics")
    for topic in options.topic:
        for info in reader.topic_types:
            if topic == info.name:
                qos = QoSProfile(history=0, depth=10)
                if len(info.offered_qos_profiles) > 0:
                    offered_qos = yaml.safe_load(info.offered_qos_profiles)[0]
                    print(offered_qos)
                    qos = QoSProfile(
                        history=offered_qos['history'],
                        depth=offered_qos['depth'],
                        durability=offered_qos['durability'],
                        reliability=offered_qos['reliability'],
                        )
                pubs[topic] = node.create_publisher(import_class(info.type), topic, qos_profile=qos)

reader.set_filter_by_topics(options.topic)
reader.set_filter_by_options(options)  # filter by start and duration

NUM_OF_DATA = 100
ts = tuple([[] for i in range(NUM_OF_DATA)])
ds = tuple([[] for i in range(NUM_OF_DATA)])
while reader.has_next():
    try:
        (topic, msg, t, st) = reader.serialize_next()
    except:
        continue
    if not topic:
        continue
    dt_object_utc = datetime.utcfromtimestamp(t).replace(tzinfo=pytz.utc)
    dt_object_jst = dt_object_utc + timedelta(hours=options.timezone)

    if options.publish:
        if topic not in pubs:
            pubs[topic] = node.create_publisher(type(msg), topic, 10)
        pubs[topic].publish(msg)
        print(f"publishing a {topic} message")
    else:
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
        if options.publish:
            rclpy.spin_once(node, timeout_sec=1.0)
        break

    if options.plot:
        for index, plot in enumerate(options.plot):
            val = get_nested_attr(msg, plot)
            if val is not None:
                ts[index].append(st)
                ds[index].append(val)
            else:
                logging.error(f"cannot get attribute {options.plot} in {msg}")


if options.plot:
    plt.figure(figsize=(24, 18))
    for index, plot in enumerate(options.plot):
        plt.plot(ts[index], ds[index], label=plot)
    plt.legend()
    plt.show()

if options.publish:
    rclpy.spin(node)
