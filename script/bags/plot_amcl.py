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

ts = []
xs = []
ys = []
xs2 = []
ys2 = []
xs3 = []
ys3 = []

for topic, msg, t in bag.read_messages(topics=["/cabot/odom"]):

    transform = btf.lookupTransform("map", "base_footprint", t)
    ts.append(t.to_sec())
    xs.append(transform[0][0])
    ys.append(transform[0][1])
    transform = btf.lookupTransform("odom", "base_footprint", t)
    xs2.append(-transform[0][0])
    ys2.append(-transform[0][1])
    transform = btf.lookupTransform("map", "odom", t)
    xs3.append(transform[0][0])
    ys3.append(transform[0][1])

print xs3
print ys3
bag.close()

plt.plot(xs, ys, color='blue', label="amcl")
plt.plot(xs2, ys2, color='red', label="odom")
plt.plot(xs3, ys3, color='green', label="diff")
plt.legend()
plt.show()

plt.clf()
#plt.plot(ts, xs, color='blue', label="amcl")
#plt.plot(ts, ys2, color='red', label="odom")
#plt.plot(ts, ys, color='blue', label="amcl")
#plt.plot(ts, xs2, color='red', label="odom")
plt.plot(ts, xs3, color='blue', label="diff-x")
plt.plot(ts, ys3, color='green', label="diff-y")
plt.legend()
plt.show()
