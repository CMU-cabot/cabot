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

bag = rosbag.Bag(sys.argv[1])

ts=[]
xs=[]

ts2=[]
xs2=[]

for topic, msg, t in bag.read_messages(topics=["/tracked_humans"]):

    for c in msg.circles:
        a = [c.velocity.x, c.velocity.y]
        v = numpy.linalg.norm(a)
        if  v < 0.1:
            print "%.2f, %.2f, %.2f"% (v, a[0], a[1])
        ts.append(t.to_sec())
        xs.append(v)
    
bag.close()
plt.plot(ts, xs)
plt.show()
