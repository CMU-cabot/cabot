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

import sys
from optparse import OptionParser
from matplotlib import pyplot as plt
from cabot_common.rosbag2 import BagReader
from pathlib import Path
import re
import os
from rosidl_runtime_py import message_to_csv
from datetime import datetime, timedelta
import pytz

parser = OptionParser(usage="""
Example
{0} -f <bag file>                       # bagfile
""".format(sys.argv[0]))

parser.add_option('-f', '--file', type=str, help='bag file to be processed')
parser.add_option('-o', '--odom', action='store_true', help='output odom')
parser.add_option('-s', '--start', type=float, help='start time from the begining', default=0.0)
parser.add_option('-d', '--duration', type=float, help='duration from the start time', default=99999999999999)
parser.add_option('-T', '--timezone', type=int, help='set timezone default=0', default=0)


(options, args) = parser.parse_args()

def find_file(dir, pat):
    path = Path(dir)
    for file in path.rglob(pat):
            return file

file = find_file(os.getcwd(), "cabot_serial*")

pattern = r'\[(\d+\.\d+)\]'
val_ptr = r'\,(\d+)'

result = tuple([[] for i in range(100)])
with open(file, "r") as f:
    for line in f:
         if "CAP12xx" in line:
              line_strip = line.strip()
              match = re.search(pattern, line_strip)
              val_match = re.search(val_ptr, line_strip)
              result[0].append(match.group(1))
              result[1].append(val_match.group(1))
              result[2].append(val_match.group(2))
              result[3].append(val_match.group(3))
            #   result.append([match.group(1), val_match.group(1), val_match.group(2), val_match.group(3)])

if not options.file:
    parser.print_help()
    sys.exit(0)

bagfilename = options.file
reader = BagReader(bagfilename)

reader.set_filter_by_topics([
    "/cmd_vel",
    "/cabot/touch",
    "/cabot/touch_raw",
])
reader.set_filter_by_options(options)  # filter by start and duration


data = tuple([[] for i in range(100)])
indexes = {}
index = 0


def getIndex(name, increment=0):
    global indexes, index
    if name not in indexes:
        indexes[name] = index
        index += increment
    return indexes[name]

tmp_data = []
rsl_ind = 0
first = True
base_time = 0

while reader.has_next():
    (topic, msg, t, st) = reader.serialize_next()
    if not topic:
        continue
    dt_object_utc = datetime.utcfromtimestamp(t).replace(tzinfo=pytz.utc)
    dt_object_jst = dt_object_utc + timedelta(hours=options.timezone)

    # while first and round(t, 2) > float(result[rsl_ind][0]):
    #     rsl_ind += 1

    if first:
        base_time = t-st
        first = False

    # if not first and round(t, 2) > float(result[rsl_ind][0]):
    #     print(result[rsl_ind][1])
    #     rsl_ind += 1

    if topic == "/cmd_vel":
        i = getIndex(topic, 2)
        data[i].append(st)
        data[i+1].append(msg.linear.x)
    elif topic in [
            "/cabot/touch",
            "/cabot/touch_raw"]:
        i = getIndex(topic, 2)
        data[i].append(st)
        data[i+1].append(msg.data)


def plot(name, linestyle):
    i = getIndex(name)
    plt.plot(data[i], data[i+1], "red", linestyle=linestyle, label=f'{name}.l')


plt.figure(figsize=(20, 10))
# plot("/cmd_vel", '-')
plot("/cabot/touch", '--')
plot("/cabot/touch_raw", ':')
plt.plot(result[0], result[1], "green", linestyle=':', label=f'general status')
plt.plot(data[i], data[i+1], "blue", linestyle=':', label=f'noise flag')
plt.plot(data[i], data[i+1], "purple", linestyle=':', label=f'calibration')

plt.legend(bbox_to_anchor=(1.00, 1), loc='upper left')

plt.show()
