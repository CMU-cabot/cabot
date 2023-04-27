#!/usr/bin/env python

# Copyright (c) 2020  Carnegie Mellon University
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
import os
import sys
import rosbag2_py
import rclpy.time
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import numpy, numpy.linalg
from matplotlib import pyplot as plt
from tf_bag import BagTfTransformer


parser = OptionParser(usage="""
Example
{0} -f <bag file>                        # show a list of process whose maximum usage is over 50%
""".format(sys.argv[0]))

parser.add_option('-f', '--file', type=str, help='bag file to plot')

def get_rosbag_options(path, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    return storage_options, converter_options

(options, args) = parser.parse_args()

if not options.file:
    parser.print_help()
    sys.exit(0)

bagfilename = options.file
filename=os.path.splitext(os.path.split(bagfilename)[-1])[0]
storage_options, converter_options = get_rosbag_options(bagfilename)
reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

topic_types = reader.get_all_topics_and_types()
type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

storage_filter = rosbag2_py.StorageFilter(topics=["/cabot/odom", "/cabot/odom_raw", "/cabot/odom_hector", "/cabot/odometry/filtered"])
reader.set_filter(storage_filter)

data = []
times = []
summary = tuple([[] for i in range(30)])

pidindex = 9
pidmap = {}

maxcpu = 0
maxmem = 0
count = 0
prev = 0

btf = BagTfTransformer(bagfilename)

ts=[[],[],[],[],[]]
xs=[[],[],[],[],[]]
ys=[[],[],[],[],[]]


while reader.has_next():
    (topic, msg_data, t) = reader.read_next()
    msg_type = get_message(type_map[topic])
    msg = deserialize_message(msg_data, msg_type)

    if topic == "/cabot/odom":
        ts[0].append(t)
        xs[0].append(msg.pose.pose.position.x)
        ys[0].append(msg.pose.pose.position.y)
    if topic == "/cabot/odom_raw":
        ts[1].append(t)
        xs[1].append(msg.pose.pose.position.x)
        ys[1].append(msg.pose.pose.position.y)
    if topic == "/cabot/odom_hector":
        ts[2].append(t)
        xs[2].append(msg.pose.pose.position.x)
        ys[2].append(msg.pose.pose.position.y)
    if topic == "/cabot/odometry/filtered":
        ts[3].append(t)
        xs[3].append(msg.pose.pose.position.x)
        ys[3].append(msg.pose.pose.position.y)

    try:
        transform = btf.lookupTransform("odom", "base_footprint", rclpy.time.Time(nanoseconds=t))
        ts[4].append(t)
        xs[4].append(transform.transform.translation.x)
        ys[4].append(transform.transform.translation.y)
    except:
        import traceback
        traceback.print_exc()
        break



plt.plot(xs[0], ys[0], color='blue', label="odom")
plt.plot(xs[1], ys[1], color='red', label="odom raw")
plt.plot(xs[2], ys[2], color='green', label="odom hector")
plt.plot(xs[3], ys[3], color='orange', label="odom filtered")
plt.plot(xs[4], ys[4], color='black', label="odom tf")

plt.legend()
plt.show()
