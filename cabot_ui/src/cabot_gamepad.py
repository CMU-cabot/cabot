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
import sensor_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import cabot.util
import cabot.button

from cabot.event import JoyButtonEvent, JoyClickEvent, ButtonEvent, ClickEvent
import cabot.button 
from cabot.handle_v2 import Handle

NUMBER_OF_BUTTONS = 13
NUMBER_OF_AXES_BUTTONS = 4
TOTAL_BUTTONS = NUMBER_OF_BUTTONS + NUMBER_OF_AXES_BUTTONS

JOYAXIS_UP = 0
JOYAXIS_DOWN = 1
JOYAXIS_LEFT = 2
JOYAXIS_RIGHT = 3
JOYBUTTON_UP = NUMBER_OF_BUTTONS + 0
JOYBUTTON_DOWN = NUMBER_OF_BUTTONS + 1
JOYBUTTON_LEFT = NUMBER_OF_BUTTONS + 2
JOYBUTTON_RIGHT = NUMBER_OF_BUTTONS + 3

JOYBUTTON_CROSS = 0
JOYBUTTON_CIRCLE = 1
JOYBUTTON_L2 = 6
JOYBUTTON_R2 = 7

interval = 0.25
lastUp = [None] * TOTAL_BUTTONS
upCount = [0] * TOTAL_BUTTONS
btnDwn = [False] * TOTAL_BUTTONS
eventPub = rospy.Publisher("/cabot/event", std_msgs.msg.String, queue_size=1)


def joy_callback(data):
    lc_lr = data.axes[6]
    lc_ud = data.axes[7]
    
    global btnDwn, vibration_mode
    now = rospy.get_rostime().to_sec()
    event = None
    temp = [False] * TOTAL_BUTTONS
    for i in range(0, NUMBER_OF_BUTTONS):
        temp[i] = data.buttons[i] == 1

    # cursor axis to button
    for i in range(0, NUMBER_OF_AXES_BUTTONS):
        j = NUMBER_OF_BUTTONS + i
        if i == JOYAXIS_UP: #up
            temp[j] = lc_ud == 1
        if i == JOYAXIS_DOWN: #down
            temp[j] = lc_ud == -1
        if i == JOYAXIS_LEFT: #left
            temp[j] = lc_lr == 1
        if i == JOYAXIS_RIGHT: #right
            temp[j] = lc_lr == -1

    # mapping buttons
    for i in range(0, TOTAL_BUTTONS):
        if temp[i] and not btnDwn[i]:
            event = JoyButtonEvent(button=i, up=False)
            btnDwn[i] = True
        if not temp[i] and btnDwn[i]:
            event = JoyButtonEvent(button=i, up=True)
            upCount[i] += 1
            lastUp[i] = now
            btnDwn[i] = False
        if lastUp[i] is not None and now - lastUp[i] > interval:
            event = JoyClickEvent(buttons=i, count=upCount[i])            
            lastUp[i] = None
            upCount[i] = 0

    if event is not None:
        rospy.loginfo(event)
        msg = std_msgs.msg.String()
        msg.data = str(event)
        eventPub.publish(msg)

        etemp = None
        ## L2 button - vibration mode
        if temp[JOYBUTTON_L2]:
            etemp = map_vib_event(event)
        ## R2 button - menu mode
        if temp[JOYBUTTON_R2]:
            etemp = map_menu_event(event)

        if etemp is not None:
            msg = std_msgs.msg.String()
            msg.data = str(etemp)
            eventPub.publish(msg)


def map_vib_event(event):
    if event.type == JoyClickEvent.TYPE:
        if event.buttons == JOYBUTTON_UP and event.count == 1:
            return Handle.get_name(Handle.FRONT)
        if event.buttons == JOYBUTTON_LEFT:
            if event.count == 1:
                return Handle.get_name(Handle.LEFT_DEV)
            if event.count == 2:
                return Handle.get_name(Handle.LEFT_TURN)
        if event.buttons == JOYBUTTON_RIGHT:
            if event.count == 1:
                return Handle.get_name(Handle.RIGHT_DEV)
            if event.count == 2:
                return Handle.get_name(Handle.RIGHT_TURN)


def map_menu_event(event):
    if event.type == JoyButtonEvent.TYPE and event.down:
        if event.button == JOYBUTTON_UP:
            return ButtonEvent(button=cabot.button.BUTTON_PREV, up=False)
        if event.button == JOYBUTTON_DOWN:
            return ButtonEvent(button=cabot.button.BUTTON_NEXT, up=False)
        if event.button == JOYBUTTON_CROSS:
            return ButtonEvent(button=cabot.button.BUTTON_SELECT, up=False)

    if event.type == JoyClickEvent.TYPE:
        if event.count == 1:
            if event.buttons == JOYBUTTON_UP:
                return ClickEvent(buttons=cabot.button.BUTTON_PREV, count=1)
            if event.buttons == JOYBUTTON_DOWN:
                return ClickEvent(buttons=cabot.button.BUTTON_NEXT, count=1)
            if event.buttons == JOYBUTTON_CROSS:
                return ClickEvent(buttons=cabot.button.BUTTON_SELECT, count=1)
            if event.buttons == JOYBUTTON_CIRCLE:
                return ClickEvent(buttons=cabot.button.BUTTON_SELECT, count=2)

        if event.count == 2:
            if event.buttons == JOYBUTTON_CROSS:
                return ClickEvent(buttons=cabot.button.BUTTON_SELECT, count=2)

if __name__ == "__main__":
    rospy.init_node("cabot_gamepad")
    rospy.Subscriber("/joy", sensor_msgs.msg.Joy, joy_callback)    
    rospy.spin()
