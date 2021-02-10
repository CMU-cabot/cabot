#!/usr/bin/env python

# Copyright (c) 2020 Carnegie Mellon University
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
import rosbag
import os
import os.path
from matplotlib import pyplot as plt
import numpy
import math
import argparse
from tf.transformations import euler_from_quaternion, quaternion_from_euler

parser = argparse.ArgumentParser(description="Plot robot ctrl")
parser.add_argument('-f', '--file', type=str, help='bag file to plot')
parser.add_argument('-n', '--namespace', type=str, default='/cabot', help='namespace')

args = parser.parse_args()
ns = args.namespace
print(args)

bias = 0.254
bagfilename = args.file
filename=os.path.splitext(bagfilename)[0]
bag = rosbag.Bag(bagfilename)
data = tuple([[] for i in range(30)])

init_t = None
last_t = None
inityaw = None

for topic, msg, t in bag.read_messages(topics=[ns+"/pressure",
                                               ns+"/temperature",
                                               ns+"/imu_raw"]):
    if init_t is None:
        init_t = t.to_sec()
    last_t = t.to_sec()
    now = t.to_sec()-init_t
    
    if topic == ns+"/pressure":
        data[0].append(now)
        data[1].append(msg.fluid_pressure)
        #print(msg)
    if topic == ns+"/temperature":
        data[3].append(now)
        data[4].append(msg.temperature)
        #print(msg)
    if topic == ns+"/imu_raw":
        data[6].append(now)
        data[7].append(msg.data[8])

bag.close()

fig, ax1 = plt.subplots(figsize=(20, 10))

for i in range(0, len(data[1])):
    p = data[1][i]
    p0 = 101325
    t = data[4][i]
    a = (pow(p0/p, 1.0/5.257)-1.0) * (t+273.15) / 0.0065
    print(p, t, a)
    data[5].append(a)


ax1.scatter(data[0], data[5], c="blue", marker='.', label="pressure")
ax1.legend(bbox_to_anchor=(1.00, 1), loc='upper left')

ax2 = ax1.twinx()

ax2.scatter(data[6], data[7], c="red", marker='.', label="z")
ax2.legend()

#plt.savefig("{}-pressure.png".format(filename, i))
plt.show()
plt.close()
