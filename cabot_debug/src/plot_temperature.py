#!/usr/bin/env python3

###############################################################################
# Copyright (c) 2019, 2023  Carnegie Mellon University
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
from optparse import OptionParser
from matplotlib import pyplot as plt
from cabot_common.rosbag2 import BagReader


parser = OptionParser(
    usage="""
Example
{0} -f <bag file>                        # plot cpu clocks
{0} -f <bag file> -t                     # plot cpu clocks and temperature
""".format(
        sys.argv[0]
    )
)

parser.add_option("-f", "--file", type=str, help="bag file to plot")
parser.add_option("-t", "--temp", action="store_true", help="plot temperature")
parser.add_option("-o", "--output", type=str, help="output with filename")
parser.add_option("-g", "--gpu", action="store_true", help="plot gpu temperature")
parser.add_option("-b", "--bme", action="store_true", help="plot bme temperature")

(options, args) = parser.parse_args()

if not options.file:
    parser.print_help()
    sys.exit(0)

bagfilename = options.file
reader = BagReader(bagfilename)

reader.set_filter_by_topics(
    [
        "/sar",
        "/nvidia_smi_dmon",
        "/cabot/temperature",
    ]
)
reader.set_filter_by_options(options)  # filter by start and duration

(options, args) = parser.parse_args()

print(options)

if not options.file:
    parser.print_help()
    sys.exit(0)

data = tuple([[] for i in range(10000)])

tempmap = {}

prev = 0
while reader.has_next():
    (topic, msg, t, st) = reader.serialize_next()
    if not topic:
        continue

    now = st
    if now <= prev:
        continue
    prev = now

    if topic == "/sar":
        lines = msg.data.split("\n")
        items = re.split(" +", lines[1])
        if len(items) == 0:
            continue

        try:
            data[1].append(float(items[2]))
            data[0].append(now)

            tempidx = lines.index("")
            for i in range(tempidx + 2, tempidx + 6):  # (16,25):
                items2 = re.split(" +", lines[i])
                data[i - tempidx].append(float(items2[2]))
                temp = "{}-{}".format(items2[1], items2[4])
                tempmap[temp] = i - tempidx
        except:
            print("warning: error parsing {}".format(msg.data))
            traceback.print_exc()
            sys.exit(0)

    if topic == "/nvidia_smi_dmon":
        items = re.split(" +", msg.data)
        try:
            temp = int(items[2])
            data[12].append(now)
            data[13].append(temp)
        except:
            pass

    if topic == "/cabot/temperature":
        data[14].append(now)
        data[15].append(msg.temperature)


from pylab import rcParams

rcParams["figure.figsize"] = 40, 20

fig, ax = plt.subplots()

ax.plot(data[0], data[1], "k-", label="CPU clock")
ax.set_ylim([0, 5000])
ax.set_yticks(numpy.arange(0, 5000, step=250))
ax.grid(True, which="both", axis="both")
ax.legend(loc=2)

if options.temp or options.gpu or options.bme:
    ax2 = ax.twinx()
    ax2.set_ylim([0, 100])
    ax2.set_yticks(numpy.arange(0, 100, step=5))
    ax2.grid(True, which="both", axis="both")

if options.temp:
    ax2.plot(data[0], data[10], label="wifi temp")

if options.gpu:
    ax2.plot(data[12], data[13], label="gpu temp")

if options.bme:
    ax2.plot(data[14], data[15], label="bme temp")

if options.temp or options.gpu:
    ax2.legend(loc=1)

if options.output:
    plt.savefig(options.output)
else:
    plt.show()
