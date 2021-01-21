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
import os
import os.path
from matplotlib import pyplot as plt
import numpy
import argparse
from tf.transformations import euler_from_quaternion, quaternion_from_euler

parser = argparse.ArgumentParser(description="Plot robot ctrl")
parser.add_argument('-f', '--file', type=str, help='bag file to plot')

args = parser.parse_args()
print args

bias = 0.254
bagfilename = args.file
filename=os.path.splitext(bagfilename)[0]
bag = rosbag.Bag(bagfilename)
data = tuple([[] for i in range(30)])

init_t = None
last_t = None
inityaw = None

for topic, msg, t in bag.read_messages(topics=["/cabot/raw_cmd_vel",
                                               "/cabot/cmd_vel",
                                               "/cabot/odometry/filtered",
                                               "/cabot/odom_raw",
                                               "/cabot/odom_hector",
                                               "/cabot/motorTarget",
                                               "/cabot/map_speed",
                                               "/cabot/motorStatus"]):
    if init_t is None:
        init_t = t.to_sec()
    last_t = t.to_sec()
    now = t.to_sec()-init_t
    
    if topic == "/cabot/raw_cmd_vel":
        data[0].append(now)
        data[1].append(msg.linear.x)
        data[2].append(msg.angular.z)
    if topic == "/cabot/cmd_vel":
        data[3].append(now)
        data[4].append(msg.linear.x)
        data[5].append(msg.angular.z)
    if topic == "/cabot/odometry/filtered":
        data[6].append(now)
        data[7].append(msg.twist.twist.linear.x)
        data[8].append(msg.twist.twist.angular.z)
        po = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([po.x, po.y, po.z, po.w])
        if inityaw is None:
            inityaw = yaw
        data[23].append((yaw-inityaw)/3.14)
        prevyaw = yaw
    if topic == "/cabot/odom_raw":
        data[9].append(now)
        data[10].append(msg.twist.twist.linear.x)
        data[11].append(msg.twist.twist.angular.z)
    if topic == "/cabot/odom_hector":
        data[20].append(now)
        data[21].append(msg.twist.twist.linear.x)
        data[22].append(msg.twist.twist.angular.z)
    if topic == "/cabot/motorTarget":
        data[12].append(now)
        #data[13].append((msg.spdLeft+msg.spdRight)/2)
        #data[14].append((msg.spdRight-msg.spdLeft)/bias)
        data[13].append(msg.spdLeft)
        data[14].append(msg.spdRight)
    if topic == "/cabot/motorStatus":
        data[15].append(now)
        #data[16].append((msg.spdLeft+msg.spdRight)/2)
        #data[17].append((msg.spdRight-msg.spdLeft)/bias)
        data[16].append(msg.spdLeft)
        data[17].append(msg.spdRight)
    if topic == "/cabot/map_speed":
        data[18].append(now)
        data[19].append(msg.data)
        

p = None
for d in zip(data[6], data[23]):
    if p is None:
        p = d
        continue

    if d[0] - p[0] > 0.1:
        data[24].append(d[0])
        data[25].append(3.14 * (d[1] - p[1]) / (d[0] - p[0]))
        p = d


bag.close()

duration=last_t-init_t
interval=5
print "duration", duration

if not os.path.exists(filename):
    os.makedirs(filename)

for i in xrange(0, int(duration)-4):
    plt.figure(figsize=(40,20))
    
    #plt.plot(data[0], data[1], "red", linestyle='--', label='raw_cmd_vel.l')
    #plt.plot(data[0], data[2], "blue", linestyle='--', label='raw_cmd_vel.r')
    
    plt.plot(data[3], data[4], "red", linestyle='-', label='cmd_vel.l')
    plt.plot(data[3], data[5], "blue", linestyle='-', label='cmd_vel.r')
    
    plt.plot(data[6], data[23], "black", linestyle='--', label='pose.orientation.z')
    #plt.plot(data[24], data[25], "gray", linestyle='--', label='pose.orientation.z diff')
    #plt.plot(data[6], data[7], "black", linestyle='--', label='odom.l')
    plt.plot(data[6], data[8], "gray", linestyle='--', label='odom.r')
    
    #plt.plot(data[9], data[10], "black", linestyle='-', label='odom_raw.l')
    #plt.plot(data[9], data[11], "gray", linestyle='-', label='odom_raw.r')
    
    #plt.plot(data[20], data[21], "black", linestyle=':', label='odom_hector.l')
    #plt.plot(data[20], data[22], "gray", linestyle=':', label='odom_hector.r')
    
    plt.plot(data[12], data[13], "purple", linestyle='-', label='target.l')
    plt.plot(data[12], data[14], "green", linestyle='-', label='target.r')
    
    plt.plot(data[15], data[16], "purple", linestyle=':', label='status.l')
    plt.plot(data[15], data[17], "green", linestyle=':', label='status.r')
    
    #plt.plot(data[18], data[19], "black", linestyle=':', label='map_speed')
    plt.xlim([i, i+5])
    plt.ylim([-1.2, 1.2])
    plt.legend(bbox_to_anchor=(1.00, 1), loc='upper left')
    plt.subplots_adjust(right=0.7)
    plt.savefig("{}/{}.png".format(filename, i))
    plt.close()
