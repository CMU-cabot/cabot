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

import os
import sys
import re
import numpy
import traceback
import multiprocessing
import yaml

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

from optparse import OptionParser
from matplotlib import pyplot as plt
from bisect import bisect

parser = OptionParser(usage="""
Example
{0} -f <bag file>                       # bagfile
""".format(sys.argv[0]))

parser.add_option('-f', '--file', type=str, help='bag file to be processed')
parser.add_option('-o', '--odom', action='store_true', help='output odom')



def get_rosbag_options(path, serialization_format='cdr'):
    data = yaml.safe_load(open(path+"/metadata.yaml"))
    storage_options = rosbag2_py.StorageOptions(
        uri=path,
        storage_id=data['rosbag2_bagfile_information']['storage_identifier'])

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

storage_filter = rosbag2_py.StorageFilter(
    topics=[
        "/cmd_vel",
        "/cabot/cmd_vel_adapter",
        "/cabot/cmd_vel",
        "/cabot/motorTarget",
        "/odom",
        "/cabot/odom_raw",
        "/cabot/odometry/filtered",
    ])
reader.set_filter(storage_filter)


data = tuple([[] for i in range(100)])
indexes = {}
index = 0


def getIndex(name, increment=0):
    global indexes, index
    if name not in indexes:
        indexes[name] = index
        index += increment
    return indexes[name]


while reader.has_next():
    (topic, msg_data, t) = reader.read_next()
    msg_type = get_message(type_map[topic])
    msg = deserialize_message(msg_data, msg_type)

    if topic in [
            "/cmd_vel",
            "/cabot/cmd_vel_adapter",
            "/cabot/cmd_vel"]:
        i = getIndex(topic, 3)
        data[i].append(t)
        data[i+1].append(msg.linear.x)
        data[i+2].append(msg.angular.z)
    elif topic == "/cabot/motorTarget":
        i = getIndex(topic, 3)
        data[i].append(t)
        data[i+1].append(msg.spd_left)
        data[i+2].append(msg.spd_right)
    elif topic in [
            "/cabot/odom_raw",
            "/cabot/odometry/filtered",
            "/odom"]:
        i = getIndex(topic, 3)
        data[i].append(t)
        data[i+1].append(msg.twist.twist.linear.x)
        data[i+2].append(msg.twist.twist.angular.z)


def plot_cmd_vel(name, linestyle):
    i = getIndex(name)
    plt.plot(data[i], data[i+1], "red", linestyle=linestyle, label=f'{name}.l')
    plt.plot(data[i], data[i+2], "blue", linestyle=linestyle, label=f'{name}.r')


def plot_odom(name, linestyle):
    i = getIndex(name)
    plt.plot(data[i], data[i+1], "purple", linestyle=linestyle, label=f'{name}.l')
    plt.plot(data[i], data[i+2], "green", linestyle=linestyle, label=f'{name}.r')


plt.figure(figsize=(20, 10))
if options.odom:
    plot_odom("/cabot/odom_raw", ':')
    plot_odom("/cabot/odometry/filtered", '--')
    plot_odom("/odom", '-')
else:
    plot_cmd_vel("/cmd_vel", '-')
    plot_cmd_vel("/cabot/cmd_vel_adapter", '--')
    plot_cmd_vel("/cabot/cmd_vel", ':')
plt.legend(bbox_to_anchor=(1.00, 1), loc='upper left')

plt.show()
