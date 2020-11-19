#!/usr/bin/env python

# Copyright 2020 Carnegie Mellon University
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
# Contributor: Daisuke Sato <daisukes@cmu.edu>
#

import rospy
import cabot.event
import cabot.button
import std_msgs.msg

if __name__ == "__main__":
    rospy.init_node("menu_action_test_node", log_level=rospy.DEBUG)
    pub = rospy.Publisher("/cabot/event", std_msgs.msg.String, queue_size=10)
    
    rospy.sleep(3)
    rospy.loginfo("send click")
    msg = std_msgs.msg.String()
    evt = cabot.event.ClickEvent(buttons=cabot.button.BUTTON_SELECT, count=1)
    msg.data = str(evt)
    pub.publish(msg)
    
    rospy.sleep(2)
    rospy.loginfo("send click")
    evt = cabot.event.ClickEvent(buttons=cabot.button.BUTTON_SELECT, count=1)
    msg.data = str(evt)
    pub.publish(msg)

    rospy.sleep(2)
    rospy.loginfo("send click")
    evt = cabot.event.ClickEvent(buttons=cabot.button.BUTTON_SELECT, count=1)
    msg.data = str(evt)
    pub.publish(msg)

    
rospy.spin()
