#!/usr/bin/env python3

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

import sys
import yaml
import os
import os.path
from matplotlib import pyplot as plt
import matplotlib.ticker as ticker
from tf_transformations import euler_from_quaternion
from optparse import OptionParser
from cabot_common.rosbag2 import BagReader
from pathlib import Path


parser = OptionParser(
    usage="""
Example
{0} -f <bag file>                        # show a list of process whose maximum usage is over 50%
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
filepath = Path(bagfilename)
reader = BagReader(bagfilename)

reader.set_filter_by_topics(
    ["/cabot/raw_cmd_vel", "/cabot/cmd_vel", "/cabot/odometry/filtered", "/cabot/odom_raw", "/cabot/odom_hector", "/cabot/motorTarget", "/cabot/map_speed", "/cabot/motorStatus"]
)
reader.set_filter_by_options(options)  # filter by start and duration

data = tuple([[] for i in range(30)])

init_t = None
last_t = None
inityaw = None

start_time = None

while reader.has_next():
    (topic, msg, t, st) = reader.serialize_next()
    if not topic:
        continue

    if init_t is None:
        init_t = t
    last_t = t
    now = t - init_t

    if topic == "/cabot/raw_cmd_vel":
        data[0].append(now)
        data[1].append(msg.linear.x)
        data[2].append(msg.angular.z)
    if topic == "/cabot/cmd_vel":
        data[3].append(now)
        data[4].append(msg.linear.x)
        data[5].append(msg.angular.z)
    if topic == "/cabot/odometry/filtered":
        data[6].append(now)
        data[7].append(msg.twist.twist.linear.x)
        data[8].append(msg.twist.twist.angular.z)
        po = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([po.x, po.y, po.z, po.w])
        if inityaw is None:
            inityaw = yaw
        data[23].append((yaw - inityaw) / 3.14)
        prevyaw = yaw
    if topic == "/cabot/odom_raw":
        data[9].append(now)
        data[10].append(msg.twist.twist.linear.x)
        data[11].append(msg.twist.twist.angular.z)
    if topic == "/cabot/odom_hector":
        data[20].append(now)
        data[21].append(msg.twist.twist.linear.x)
        data[22].append(msg.twist.twist.angular.z)
    if topic == "/cabot/motorTarget":
        data[12].append(now)
        # data[13].append((msg.spd_left+msg.spd_right)/2)
        # data[14].append((msg.spd_right-msg.spd_left)/bias)
        data[13].append(msg.spd_left)
        data[14].append(msg.spd_right)
    if topic == "/cabot/motorStatus":
        data[15].append(now)
        # data[16].append((msg.spdLeft+msg.spdRight)/2)
        # data[17].append((msg.spdRight-msg.spdLeft)/bias)
        data[16].append(msg.spd_left)
        data[17].append(msg.spd_right)
    if topic == "/cabot/map_speed":
        data[18].append(now)
        data[19].append(msg.data)


p = None
for d in zip(data[6], data[23]):
    if p is None:
        p = d
        continue

    if d[0] - p[0] > 0.1:
        data[24].append(d[0])
        data[25].append(3.14 * (d[1] - p[1]) / (d[0] - p[0]))
        p = d


duration = last_t - init_t
interval = 5
print("duration", duration)

if not filepath.exists():
    print(f"something wrong the filepath should be a directory for rosbag2 {filepath}")
    sys.exit(1)

for i in range(0, int(duration) - 4):
    plt.figure(figsize=(40, 20))
    ax = plt.subplot(1, 1, 1)

    ax.xaxis.set_major_locator(ticker.MultipleLocator(0.5))
    ax.xaxis.set_minor_locator(ticker.MultipleLocator(0.1))

    # plt.plot(data[0], data[1], "red", linestyle='--', label='raw_cmd_vel.l')
    # plt.plot(data[0], data[2], "blue", linestyle='--', label='raw_cmd_vel.r')

    ax.plot(data[3], data[4], "red", linestyle="-", label="cmd_vel.l")
    ax.plot(data[3], data[5], "blue", linestyle="-", label="cmd_vel.r")

    ax.plot(data[6], data[23], "black", linestyle="--", label="pose.orientation.z")
    # plt.plot(data[24], data[25], "gray", linestyle='--', label='pose.orientation.z diff')
    # plt.plot(data[6], data[7], "black", linestyle='--', label='odom.l')
    ax.plot(data[6], data[8], "gray", linestyle="--", label="odom.r")

    # ax.plot(data[9], data[10], "black", linestyle='-', label='odom_raw.l')
    # plt.plot(data[9], data[11], "gray", linestyle='-', label='odom_raw.r')

    # plt.plot(data[20], data[21], "black", linestyle=':', label='odom_hector.l')
    # plt.plot(data[20], data[22], "gray", linestyle=':', label='odom_hector.r')

    ax.plot(data[12], data[13], "purple", linestyle="-", label="target.l")
    ax.plot(data[12], data[14], "green", linestyle="-", label="target.r")

    ax.plot(data[15], data[16], "purple", linestyle=":", label="status.l")
    ax.plot(data[15], data[17], "green", linestyle=":", label="status.r")

    # plt.plot(data[18], data[19], "black", linestyle=':', label='map_speed')
    ax.set_xlim([i, i + 5])
    ax.set_ylim([-1.5, 1.5])
    ax.legend(bbox_to_anchor=(1.00, 1), loc="upper left")
    # ax.subplots_adjust(right=0.7)
    ax.minorticks_on()
    ax.tick_params(which="minor", length=10)
    ax.grid(linestyle="-", linewidth=0.5, which="major")
    ax.grid(linestyle=":", linewidth=0.5, which="minor")
    plt.savefig(str(filepath / f"{i}.png"))
    plt.close()
