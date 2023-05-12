#!/usr/bin/env python3

# Copyright (c) 2023  Carnegie Mellon University
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
import numpy
from rosbags.highlevel import AnyReader
from pathlib import Path
from matplotlib import pyplot as plt
from optparse import OptionParser
import struct
from cabot_common.rosbag2 import BagReader

parser = OptionParser(usage="""
Example
{0} -f <bag file>                        # show a list of process whose maximum usage is over 50%
{0} -f <bag file> -a                     # analyze invalid time stamps
{0} -f <bag file> -a -n                  # do not plot
""".format(sys.argv[0]))

parser.add_option('-f', '--file', type=str, help='bag file to plot')
parser.add_option('-a', '--analyze', action='store_true', help='analyze invalid time stamps')
parser.add_option('-p', '--packets', action='store_true', help='analyze packets')
parser.add_option('-n', '--no-plot', action='store_true', help='do not plot')
parser.add_option('-v', '--verbose', action='store_true', help='verbose output')

(options, args) = parser.parse_args()
if not options.file:
    parser.print_help()
    sys.exit(0)

bagfilename = options.file
reader = BagReader(bagfilename)
reader.set_filter_by_topics([
    "/velodyne_points",
    "/velodyne_packets",
])
reader.set_filter_by_options(options)  # filter by start and duration

count = 0
last = 0
invalid_data = ([], [])
data_count = 0

prev_data = []
all_data = []
prevt = 0

while reader.has_next():
    (topic, msg, t, st) = reader.serialize_next()
    if not topic:
        continue

    msg_stamp = t

    if options.packets:
        count += 1
        if topic == '/velodyne_packets':
            total = 0
            for p in msg.packets:
                total += len(p.data)
                pt = p.stamp.sec+p.stamp.nanosec/1e9
                if pt < prevt:
                    print(F"ERROR {count} {prevt} {pt}")
                prevt = pt
            mt = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9;
            all_data.append(t-mt)
            print(f"{count} {t-mt}")
        continue
    else:
        if topic == '/velodyne_packets':
            continue

    count += 1

    data = []

    if options.verbose:
        print(F"{msg.height} x {msg.width}")
        print(F"{msg.width} x {msg.point_step} = {msg.width * msg.point_step} = {msg.row_step} = {len(msg.data)}")
        print(msg)

    for f in msg.fields:
        if options.verbose:
            print(f)

    for i in range(0, msg.width):
        j = i * 22 + 18
        offset = struct.unpack('f', msg.data[j:j+4].tobytes())[0]
        if options.verbose and i % 100 == 0:
            print(F"{i:5d}/{msg.width} offset = {offset}")
        data.append(msg_stamp + offset)
        all_data.append(msg_stamp + offset)

    if options.analyze:
        invalid = 0
        for i in range(0, len(data)):
            data_count += 1
            if data[i] < last:
                invalid += 1
                invalid_data[0].append(data_count)
                invalid_data[1].append(offset)

        last = numpy.max(data)
        if invalid:
            print(F"{count:5d} Invalid time stamp found {data[0]} <= {last}, at most {invalid} points will be removed")
            if not options.no_plot:
                plt.plot(range(0, len(prev_data)), prev_data)
                plt.plot(range(len(prev_data), len(data)+len(prev_data)), data)
                plt.show()
        else:
            print(F"{count:5d} Last time stamp is {data[-1]}")
    else:
        if not options.no_plot:
            print(F"data length = {len(data)}")
            plt.plot(data)
            print((numpy.min(data), numpy.max(data)))
            plt.ylim(numpy.min(data), numpy.max(data))
            plt.show()

    prev_data = data

plt.plot(all_data)
print((numpy.min(all_data), numpy.max(all_data)))
plt.ylim(numpy.min(all_data), numpy.max(all_data))
plt.show()


print(F"invalid data count = {len(invalid_data[0])}")
plt.plot(invalid_data[0], invalid_data[1])
plt.show()
