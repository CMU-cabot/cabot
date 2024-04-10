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

import sys
from matplotlib import pyplot as plt
from optparse import OptionParser
from cabot_common.rosbag2 import BagReader

parser = OptionParser(
    usage="""
Example
{0} -f <bag file>                        # plot pressure
""".format(
        sys.argv[0]
    )
)

parser.add_option("-f", "--file", type=str, help="bag file to plot")
parser.add_option("-t", "--temp", action="store_true", help="plot temperature")
parser.add_option("-n", "--namespace", type=str, default="/cabot", help="namespace")

(options, args) = parser.parse_args()

ns = options.namespace

if not options.file:
    parser.print_help()
    sys.exit(0)

bagfilename = options.file
reader = BagReader(bagfilename)

reader.set_filter_by_topics(
    [
        ns + "/pressure",
        ns + "/temperature",
        ns + "/imu/data",
    ]
)

data = tuple([[] for i in range(30)])

init_t = None
last_t = None
inityaw = None

while reader.has_next():
    (topic, msg, t, st) = reader.serialize_next()
    if not topic:
        continue

    if topic == ns + "/pressure":
        data[0].append(st)
        data[1].append(msg.fluid_pressure)
    if topic == ns + "/temperature":
        data[3].append(st)
        data[4].append(msg.temperature)
    if topic == ns + "/imu/data":
        data[6].append(st)
        data[7].append(msg.linear_acceleration.z)

fig, ax1 = plt.subplots(figsize=(20, 10))

for i in range(0, len(data[1])):
    p = data[1][i]
    p0 = 101325
    t = data[4][i]
    a = (pow(p0 / p, 1.0 / 5.257) - 1.0) * (t + 273.15) / 0.0065
    # print(p, t, a)
    data[5].append(a)


ax1.scatter(data[0], data[5], c="blue", marker=".", label="pressure")
ax1.legend(bbox_to_anchor=(1.00, 1), loc="upper left")

ax2 = ax1.twinx()

ax2.scatter(data[6], data[7], c="red", marker=".", label="linear_acceleration.z")
ax2.legend()

# plt.savefig("{}-pressure.png".format(filename, i))
plt.show()
plt.close()
