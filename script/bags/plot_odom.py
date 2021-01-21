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

import sys
import rosbag
import numpy, numpy.linalg
from matplotlib import pyplot as plt
from tf_bag import BagTfTransformer

bag = rosbag.Bag(sys.argv[1])

btf = BagTfTransformer(bag)

ts=[[],[],[],[],[]]
xs=[[],[],[],[],[]]
ys=[[],[],[],[],[]]


for topic, msg, t in bag.read_messages(topics=["/cabot/odom", "/cabot/odom_raw", "/cabot/odom_hector", "/cabot/odometry/filtered"]):

    if topic == "/cabot/odom":
        ts[0].append(t.to_sec())
        xs[0].append(msg.pose.pose.position.x)
        ys[0].append(msg.pose.pose.position.y)
    if topic == "/cabot/odom_raw":
        ts[1].append(t.to_sec())
        xs[1].append(msg.pose.pose.position.x)
        ys[1].append(msg.pose.pose.position.y)
    if topic == "/cabot/odom_hector":
        ts[2].append(t.to_sec())
        xs[2].append(msg.pose.pose.position.x)
        ys[2].append(msg.pose.pose.position.y)
    if topic == "/cabot/odometry/filtered":
        ts[3].append(t.to_sec())
        xs[3].append(msg.pose.pose.position.x)
        ys[3].append(msg.pose.pose.position.y)

    try:
        transform = btf.lookupTransform("odom", "base_footprint", t)
        ts[4].append(t.to_sec())
        xs[4].append(transform[0][0])
        ys[4].append(transform[0][1])
    except:
        pass

bag.close()


plt.plot(xs[0], ys[0], color='blue', label="odom")
plt.plot(xs[1], ys[1], color='red', label="odom raw")
plt.plot(xs[2], ys[2], color='green', label="odom hector")
plt.plot(xs[3], ys[3], color='orange', label="odom filtered")
plt.plot(xs[4], ys[4], color='black', label="odom tf")

plt.legend()
plt.show()
