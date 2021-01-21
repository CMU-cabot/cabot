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

import rospy
import geometry_msgs.msg
import std_msgs.msg
import cabot_ui.event

accum = 0
prevt = None

MIN_POWER = 5
MIN_ACCU = 2
RESET_POWER = 3
eventPub = None

def wrench_callback(msg):
    f = msg.wrench.force.x
    t = msg.header.stamp.to_sec()
    r = process(f, t)

    event = None
    if r == 1:
        event = cabot_ui.event.NavigationEvent(subtype="speedup")
    if r == -1:
        event = cabot_ui.event.NavigationEvent(subtype="speeddown")

    if event is not None:
        eventPub.publish(str(event))

def process(f, t):
    global accum, prevt
    diff = 0
    if prevt is not None:
        diff = t - prevt
    prevt = t
    #print diff, accum
    if f > MIN_POWER:
        accum += (f - MIN_POWER)*diff
        if accum > MIN_ACCU:
            accum = 0
            return 1
        return 0
    if f < -MIN_POWER:
        accum += (f + MIN_POWER)*diff
        if accum < -MIN_ACCU:
            accum = 0
            return -1
        return 0

    if abs(f) < RESET_POWER:
        accum = 0
    return 0


if __name__ == "__main__":
    rospy.init_node("cabot_force")
    rospy.Subscriber("/cabot/cabot_e_sensor/wrench_norm",
                     geometry_msgs.msg.WrenchStamped, wrench_callback)

    eventPub = rospy.Publisher("/cabot/event", std_msgs.msg.String, queue_size=10)
    rospy.spin()
