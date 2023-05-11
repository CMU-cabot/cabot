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


parser = OptionParser(usage="""
Plot odometry data

Example
{0} -f <bag file>                        # plot odometry in x-y coordinates
{0} -f <bag file> -t                     # plot odometry distance in timeline
""".format(sys.argv[0]))

parser.add_option('-f', '--file', type=str, help='bag file to plot')
parser.add_option('-t', '--timeline', action='store_true', help='plot distance and time')
parser.add_option('-s', '--start', type=float, help='start time from the begining', default=0.0)
parser.add_option('-d', '--duration', type=float, help='duration from the start time', default=99999999999999)
parser.add_option('-c', '--cmd_vel', action='store_true', help='plot cmd_vel (only with --timeline)')


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

NUM_OF_DATA = 7
ts = tuple([[] for i in range(NUM_OF_DATA)])
xs = tuple([[] for i in range(NUM_OF_DATA)])
ys = tuple([[] for i in range(NUM_OF_DATA)])
ds = tuple([[] for i in range(NUM_OF_DATA)])


def getPos(points):
    x = 0
    y = 0
    for p in points:
        x += p.x
        y += p.y
    return x/len(points), y/len(points)


reader.set_filter_by_topics([
    "/odom",
    "/cabot/odom_raw",
    "/cabot/odom_hector",
    "/cabot/odometry/filtered",
    "/cabot/cmd_vel",
    "/local_costmap/published_footprint"
])
reader.set_filter_by_options(options)  # filter by start and duration

while reader.has_next():
    (topic, msg, t, st) = reader.serialize_next()
    if not topic:
        continue

    if topic == "/odom":
        ts[0].append(st)
        xs[0].append(msg.pose.pose.position.x)
        ys[0].append(msg.pose.pose.position.y)
    elif topic == "/cabot/odom_raw":
        ts[1].append(st)
        xs[1].append(msg.pose.pose.position.x)
        ys[1].append(msg.pose.pose.position.y)
    elif topic == "/cabot/odom_hector":
        ts[2].append(st)
        xs[2].append(msg.pose.pose.position.x)
        ys[2].append(msg.pose.pose.position.y)
    elif topic == "/cabot/odometry/filtered":
        ts[3].append(st)
        xs[3].append(msg.pose.pose.position.x)
        ys[3].append(msg.pose.pose.position.y)
    elif topic == "/local_costmap/published_footprint":
        ts[5].append(st)
        # t2 = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        # ts[5].append((t2 - start_time) - options.start)
        x, y = getPos(msg.polygon.points)
        xs[5].append(x)
        ys[5].append(y)
    elif topic == "/cabot/cmd_vel":
        ts[6].append(st)
        xs[6].append(msg.linear.x)
        ys[6].append(msg.angular.z)

    try:
        transform = btf.lookupTransform("odom", "base_footprint", rclpy.time.Time(nanoseconds=t*1e9))
        ts[4].append(st)
        xs[4].append(transform.transform.translation.x)
        ys[4].append(transform.transform.translation.y)
    except:
        import traceback
        traceback.print_exc()
        break


def dist(ts, xs, ys):
    if len(ts) == 0:
        return []
    ds = [0]
    for i in range(0, len(ts)-1):
        d = math.sqrt(math.pow(xs[0]-xs[i+1], 2) + math.pow(ys[0]-ys[i+1], 2))
        ds.append(d)
    return ds


plt.figure(figsize=(10, 10))

if options.timeline:
    for i in range(0, 6):
        ds[i].extend(dist(ts[i], xs[i], ys[i]))
        # print(F"{len(ds[i])}, {len(ts[i])}")
    plt.plot(ts[0], ds[0], color='blue', label="odom")
    plt.plot(ts[1], ds[1], color='red', label="odom raw")
    plt.plot(ts[2], ds[2], color='green', label="odom hector")
    plt.plot(ts[3], ds[3], color='orange', label="odom filtered")
    plt.plot(ts[4], ds[4], color='black', label="odom tf")
    plt.plot(ts[5], ds[5], color='gray', label="local/published_footprint")
    if options.cmd_vel:
        plt.plot(ts[6], xs[6], color='purple', label="cmd_vel.l")
        plt.plot(ts[6], ys[6], color='brown', label="cmd_vel.r")
    plt.legend()
    plt.show()

else:
    plt.plot(xs[0], ys[0], color='blue', label="odom")
    plt.plot(xs[1], ys[1], color='red', label="odom raw")
    plt.plot(xs[2], ys[2], color='green', label="odom hector")
    plt.plot(xs[3], ys[3], color='orange', label="odom filtered")
    plt.plot(xs[4], ys[4], color='black', label="odom tf")
    plt.plot(xs[5], ys[5], color='purple', label="local/published_footprint")
    plt.legend()
    plt.show()
