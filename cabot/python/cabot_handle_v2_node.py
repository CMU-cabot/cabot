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
#
# memo: want to separate hardware spec and UI
#

import rospy
import std_msgs.msg
import cabot.event

from cabot.handle_v2 import Handle

def notification_callback(msg):
    rospy.loginfo(msg)
    handle.execute_stimulus(msg.data)

def event_listener(msg):
    rospy.loginfo(msg)
    event = None
    if "button" in msg:
        event = cabot.event.ButtonEvent(**msg)

        # button down confirmation
        if not event.up:
            handle.execute_stimulus(Handle.BUTTON_CLICK)
    if "buttons" in msg:
        event = cabot.event.ClickEvent(**msg)

    if event is not None:
        rospy.loginfo(event)
        msg = std_msgs.msg.String()
        msg.data = str(event)
        event_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node("cabot_handle_v2_node")
    event_pub = rospy.Publisher('/cabot/event', std_msgs.msg.String, queue_size=10, latch=True)
    handle = Handle(event_listener=event_listener)

    no_vibration = False
    if rospy.has_param("~no_vibration"):
        no_vibration = rospy.get_param("~no_vibration", False)
        rospy.loginfo(rospy.get_param("~no_vibration"))
    rospy.loginfo("no vibration = %s" %(str(no_vibration)))

    if not no_vibration:
        rospy.Subscriber("/cabot/notification", std_msgs.msg.Int8, notification_callback)

rospy.spin()
