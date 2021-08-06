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
import rospy
import std_msgs.msg
import geometry_msgs.msg

import cabot.event
import cabot.button
from cabot.util import setInterval

interval = 0.25
NKeys = 12
lastUp = [None]*NKeys
upCount = [0]*NKeys
btnDwn = [False]*NKeys
eventPub = rospy.Publisher("/cabot/event", std_msgs.msg.String, queue_size=1)


def getchar():
    #Returns a single character from standard input
    import tty, termios, sys
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    if ord(ch) == 3: quit() # handle ctrl+C
    return ch

button = -1

@setInterval(0.01)
def process():
    global button
    now = rospy.get_rostime().to_sec()
    event = None
    temp = [False]*NKeys
    for i in range(0,len(temp)):
        temp[i] = (button == i)
        
        if temp[i] and not btnDwn[i]:
            event = cabot.event.ButtonEvent(button=i, up=False)
            btnDwn[i] = True

        if not temp[i] and btnDwn[i]:
            event = cabot.event.ButtonEvent(button=i, up=True)
            upCount[i] += 1
            lastUp[i] = now
            btnDwn[i] = False

        if lastUp[i] is not None and now - lastUp[i] > interval:
            event = cabot.event.ClickEvent(buttons=i, count=upCount[i])
            lastUp[i] = None
            upCount[i] = 0

        #rospy.loginfo(upCount)
        #rospy.loginfo(temp)
        #rospy.loginfo(btnDwn)
    
    button = -1
    if event is not None:
        rospy.loginfo(str(event)+"\r")
        msg = std_msgs.msg.String()
        msg.data = str(event)
        eventPub.publish(msg)


if __name__ == '__main__':
    rospy.init_node("cabot_keyboard_node")
    process()

    '''
    rospy.loginfo("type 'j', 'k', or 'l' for 'up', 'center', 'down' buttons")
    while not rospy.is_shutdown():
        key = ord(getchar())
        button = -1
        if key == 106: #j
            button = cabot.button.BUTTON_NEXT
        elif key == 107: #k
            button = cabot.button.BUTTON_SELECT
        elif key == 108: #l
            button = cabot.button.BUTTON_PREV
    '''
    rospy.loginfo("type 'cursor keys' for 'up', 'down', 'left', and 'right' buttons")
    while not rospy.is_shutdown():
        key = ord(getchar())
        button = -1
        if key == 65: # arrow up
            button = cabot.button.BUTTON_UP
        elif key == 66: # arrow down
            button = cabot.button.BUTTON_DOWN
        elif key == 67: # arrow right
            button = cabot.button.BUTTON_RIGHT
        elif key == 68: # arrow left
            button = cabot.button.BUTTON_LEFT
            
        if button > 0:
            rospy.loginfo("button %d", button)
        else:
            rospy.loginfo("key %d", key)



rospy.spin()
