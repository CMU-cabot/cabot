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


parser = OptionParser(
    usage="""
Plot odometry data

Example
{0} -f <bag file>                        # plot IMU
""".format(
        sys.argv[0]
    )
)

parser.add_option("-f", "--file", type=str, help="bag file to plot")
parser.add_option("-s", "--start", type=float, help="start time from the begining", default=0.0)
parser.add_option("-d", "--duration", type=float, help="duration from the start time", default=99999999999999)


(options, args) = parser.parse_args()

if not options.file:
    parser.print_help()
    sys.exit(0)

bagfilename = options.file
reader = BagReader(bagfilename)

# TODO: seek interface is available from humble, so make another reader instead
# btf = BagTfTransformer(reader)
# reader.seek(0)
reader2 = BagReader(bagfilename)
btf = BagTfTransformer(reader2)

NUM_OF_DATA = 2
ts = tuple([[] for i in range(NUM_OF_DATA)])
xs = tuple([[] for i in range(NUM_OF_DATA)])
ys = tuple([[] for i in range(NUM_OF_DATA)])
zs = tuple([[] for i in range(NUM_OF_DATA)])


def getPos(points):
    x = 0
    y = 0
    for p in points:
        x += p.x
        y += p.y
    return x / len(points), y / len(points)


reader.set_filter_by_topics(
    [
        "/cabot/imu/data",
        "/odom",
    ]
)
reader.set_filter_by_options(options)  # filter by start and duration

while reader.has_next():
    (topic, msg, t, st) = reader.serialize_next()
    if not topic:
        continue

    if topic == "/cabot/imu/data":
        ts[0].append(st)
        xs[0].append(msg.linear_acceleration.x)
        ys[0].append(msg.linear_acceleration.y)
        zs[0].append(msg.linear_acceleration.z)
    if topic == "/odom":
        ts[1].append(st)
        xs[1].append(msg.twist.twist.linear.x)

plt.figure(figsize=(10, 10))

plt.plot(ts[0], xs[0], color="blue", label="x")
plt.plot(ts[0], ys[0], color="red", label="y")
plt.plot(ts[0], zs[0], color="green", label="z")
plt.plot(ts[1], xs[1], color="black", label="linear speed")

plt.legend()
plt.show()
