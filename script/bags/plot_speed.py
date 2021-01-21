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
import os.path
from matplotlib import pyplot as plt
import numpy 

bagfilename = sys.argv[1]
filename=os.path.splitext(bagfilename)[0]

bag = rosbag.Bag(bagfilename)

ts=[]
xs=[]
rs=[]

ts2=[]
xs2=[]
rs2=[]

ts3=[]
xs3=[]
rs3=[]

ts4=[]
xs4=[]
rs4=[]

ts5=[]
td5=[]
ls5=[]
rs5=[]
ld5=[]
rd5=[]

for topic, msg, t in bag.read_messages(topics=["/cabot/raw_cmd_vel", "/cabot/motorTarget", "/cabot/odom", "/cabot/odom_raw", "/cabot/motorStatus"]):

    if topic == "/cabot/raw_cmd_vel":
        ts.append(t.to_sec())
        xs.append(msg.linear.x)
        rs.append(msg.angular.z)
    if topic == "/cabot/motorTarget":
        ts2.append(t.to_sec())
        xs2.append((msg.spdLeft+msg.spdRight)/2)
        rs2.append((msg.spdRight-msg.spdLeft)/0.165)
    if topic == "/cabot/odom":
        ts3.append(t.to_sec())
        xs3.append(msg.twist.twist.linear.x)
        rs3.append(msg.twist.twist.angular.z)
    if topic == "/cabot/odom_raw":
        ts4.append(t.to_sec())
        xs4.append(msg.twist.twist.linear.x)
        rs4.append(msg.twist.twist.angular.z)
    if topic == "/cabot/motorStatus":
        if ts5:
            td5.append(t.to_sec()-ts5[-1])
        ts5.append(t.to_sec())
        if ls5:
            ld5.append(msg.distLeft_c-ls5[-1])
        ls5.append(msg.distLeft_c)
        if rs5:
            rd5.append(msg.distRight_c-rs5[-1])
        rs5.append(msg.distRight_c)
        
bag.close()

plt.plot(ts2, xs2, color='blue', label="motorTarget.l")
plt.plot(ts2, rs2, color='green', label="motorTarget.r")
plt.grid(which="major", axis="y")
plt.ylim([-1,1])
plt.axes().yaxis.set_major_locator(plt.MultipleLocator(0.1))
plt.axes().yaxis.set_minor_locator(plt.MultipleLocator(0.05))
plt.legend(loc=3)
plt.savefig(filename+"-speed.png")

plt.clf()
plt.plot(ts, xs, color='blue', label="raw_cmd_vel.l")
plt.plot(ts, rs, color='green', label="raw_cmd_vel.r")
plt.plot(ts2, xs2, color='red', label="motorTarget.l")
plt.plot(ts2, rs2, color='orange', label="motorTarget.r")
plt.legend(loc=3)
plt.savefig(filename+"-raw_cmd_vel.png")

plt.clf()
plt.plot(ts3, xs3, color='blue', label="odom.l")
plt.plot(ts3, rs3, color='green', label="odom.r")
plt.legend(loc=3)
#plt.show()
plt.savefig(filename+"-odom.png")

plt.clf()
plt.plot(ts4, xs4, color='red', label="odom_raw.l")
plt.plot(ts4, rs4, color='orange', label="odom_raw.r")
plt.legend(loc=3)
#plt.show()
plt.savefig(filename+"-odom_raw.png")


plt.clf()
plt.plot(td5, color='blue', label="time diff")
plt.plot(numpy.array(ld5)*-1, color='red', label="distLeft")
plt.plot(numpy.array(rd5)*-1, color='orange', label="distRight")
plt.plot(numpy.array(ld5)/numpy.array(td5), color='orange', label="ratio")
plt.legend(loc=3)
#plt.show()
plt.savefig(filename+"-motor_status.png")



#plt.plot(ts, rs, color='blue')
#plt.plot(ts4, rs4, color='red')
#plt.plot(ts3, rs3, color='green')
#plt.show()
