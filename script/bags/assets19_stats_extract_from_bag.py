#!/usr/bin/env python

 ###############################################################################
 # Copyright (c) 2019  Carnegie Mellon University
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

import rosbag
import sys
import os.path
from tf_bag import BagTfTransformer
from matplotlib import pyplot as plt

plot_image = False


filename = sys.argv[1]
basename = os.path.splitext(filename)[0]


bag = rosbag.Bag(filename)
btf = None
if plot_image:
    btf = BagTfTransformer(bag)

start = None
startp = None
end = None
endp = None
last = None
lastx = None
lastp = None
totals = 0
totalt = 0

stoppingtime = 0
laststoppingtime = None

data = ([],[],[])

for topic, msg, t in bag.read_messages(topics=["/cabot/clutch", "/navcog/cancel", "/cabot/raw_cmd_vel", "/cabot/cmd_vel", "/cabot/motorStatus"]):
    pos = None
    if plot_image:
        try:
            transform = btf.lookupTransform("map", "base_footprint", t)
            pos = {'x': transform[0][0], 'y': transform[0][1]}
        except:
            continue

    if topic == "/cabot/clutch":
        if msg.data:
            start = t.to_sec()
            startp = pos
            end = None
            stoppingtime = 0
            laststoppingtime = None
            totals = 0
            totalt = 0
        else:
            if end is None:
                end = t.to_sec()
                endp = pos

    if topic == "/cabot/cmd_vel" and start is not None and end is None:
        if laststoppingtime is not None:
            stoppingtime += (t.to_sec()-laststoppingtime)
            if pos is not None:
                data[0].append(pos)
        if msg.linear.x == 0:
            laststoppingtime = t.to_sec()
        else:
            laststoppingtime = None

    if topic == "/cabot/motorStatus" and start is not None and end is None:
        speed = (msg.spdLeft+msg.spdRight)/2
        if last is not None and (lastx != 0 or speed != 0):
            totals += (lastx + speed)/2 * (t.to_sec() - last)
            totalt += (t.to_sec() - last)
            if pos is not None:
                data[1].append(pos)
        last = t.to_sec()
        lastx = speed
            
    #if topic == "/cabot/cmd_vel" and start is not None and end is None:
    #    if last is not None and (lastx != 0 or msg.linear.x != 0):
    #        totals += (lastx + msg.linear.x)/2 * (t.to_sec() - last)
    #        totalt += (t.to_sec() - last)
    #        if pos is not None:
    #            data[1].append(pos)
    #    last = t.to_sec()
    #    lastx = msg.linear.x

if startp is not None:
    data[2].append(startp)
if endp is not None:
    data[2].append(endp)

items = basename.split("_")
if len(items) < 6:
    items.insert(4,"")
items.append(start)
items.append(end)
            
if start is not None and end is not None:
    items.append(end-start-stoppingtime)
    items.append(stoppingtime)
else:
    items.append("")
    items.append("")
if totalt > 0:
    items.append(totals)
    items.append(totalt)
    items.append(totals/totalt)
else:
    items.append("")
    items.append("")
    items.append("")

print ",".join([str(x) for x in items])

if plot_image:
    plt.scatter([p["x"] for p in data[0]], [p["y"] for p in data[0]], linewidths=0, c='red', s=10)
    plt.scatter([p["x"] for p in data[1]], [p["y"] for p in data[1]], linewidths=0, c='blue', s=3)
    plt.scatter([p["x"] for p in data[2]], [p["y"] for p in data[2]], linewidths=0, c='black', s=30)
    plt.axes().set_aspect('equal', 'datalim')
    #plt.show()
    plt.savefig(basename+".png")
