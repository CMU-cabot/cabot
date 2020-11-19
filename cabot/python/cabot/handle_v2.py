#!/usr/bin/env python

# Copyright (c) 2020 Carnegie Mellon University
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
import button
from std_msgs.msg import String, UInt8, UInt8MultiArray, Bool
import random

class Handle:
    UNKNOWN = 0
    LEFT_TURN = 1
    RIGHT_TURN = 2
    LEFT_DEV = 3
    RIGHT_DEV = 4
    FRONT = 5
    stimuli_names = ["unknown", "left_turn", "right_turn", "left_dev", "right_dev", "front"]
        
    @staticmethod
    def get_name(stimulus):
        return Handle.stimuli_names[stimulus]
        
    def __init__(self, realworld_use=True, event_listener=None):
        self.event_listener = event_listener
        self.power = 255
        self.vibrator1_pub= rospy.Publisher('/cabot/vibrator1', UInt8, queue_size=10, latch=True)
        self.vibrator2_pub= rospy.Publisher('/cabot/vibrator2', UInt8, queue_size=10, latch=True)
        self.vibrator3_pub= rospy.Publisher('/cabot/vibrator3', UInt8, queue_size=10, latch=True)
        self.vibrator4_pub= rospy.Publisher('/cabot/vibrator4', UInt8, queue_size=10, latch=True)
        self.button1_sub = rospy.Subscriber('/cabot/pushed_1', Bool, self.button1_callback)
        self.button2_sub = rospy.Subscriber('/cabot/pushed_2', Bool, self.button2_callback)

        self.duration = 3
        self.duration_single_vibration = 8
        self.sleep = 150
        self.updown = True
        self.num_vibrations_turn = 4
        self.num_vibrations_deviation = 2
        self.num_vibrations_confirmation = 1

        self.callbacks = [None]*6
        self.callbacks[Handle.LEFT_TURN] = self.vibrate_left_turn
        self.callbacks[Handle.RIGHT_TURN] = self.vibrate_right_turn
        self.callbacks[Handle.LEFT_DEV] = self.vibrate_left_deviation
        self.callbacks[Handle.RIGHT_DEV] = self.vibrate_right_deviation
        self.callbacks[Handle.FRONT] = self.vibrate_front

        self.eventSub = rospy.Subscriber('/cabot/event', String, self.event_callback)

    interval = 0.25
    lastUp = [None, None]
    upCount = [0, 0]
    btnDwn = [False, False]
    def button1_callback(self, msg):
        self._button_check(msg, button.BUTTON_SPEED_UP, 0)

    def button2_callback(self, msg):
        self._button_check(msg, button.BUTTON_SPEED_DOWN, 1)

    def _button_check(self, msg, key, index):
        event = None
        now = rospy.get_rostime().to_sec()
        if msg.data and not self.btnDwn[index]:
            event = {"button":key, "up":False}
            self.btnDwn[index] = True
        if not msg.data and self.btnDwn[index]:
            event = {"button":key, "up":True}
            self.upCount[index] += 1
            self.lastUp[index] = now
            self.btnDwn[index] = False
        if self.lastUp[index] is not None and \
           now - self.lastUp[index] > self.interval:
            event = {"buttons":key, "count":self.upCount[index]}
            self.lastUp[index] = None
            self.upCount[index] = 0
            
        if event is not None:
            if self.event_listener is not None:
                self.event_listener(event)

    def event_callback(self, msg):
        name = msg.data
        if name in Handle.stimuli_names: 
            index = Handle.stimuli_names.index(name)
            self.execute_stimulus(index)

    def execute_stimulus(self, index):
        rospy.loginfo("execute_stimulus, %d" %(index))
        callback = self.callbacks[index]
        if callback is not None:
            callback()
            rospy.loginfo("executed")

    def vibrate(self, pub, time):
        msg = UInt8()
        msg.data = self.power
        pub.publish(msg)
        
        rospy.sleep(time)
        
        msg.data = 0
        pub.publish(msg)

    def vibrate_all(self, time):
        msg = UInt8()
        msg.data = self.power
        self.vibrator1_pub.publish(msg)
        self.vibrator2_pub.publish(msg)
        self.vibrator3_pub.publish(msg)
        self.vibrator4_pub.publish(msg)
        
        rospy.sleep(time)
        
        msg.data = 0
        self.vibrator1_pub.publish(msg)
        self.vibrator2_pub.publish(msg)
        self.vibrator3_pub.publish(msg)
        self.vibrator4_pub.publish(msg)

    def vibrate_left_turn(self):
        if self.updown:
            self.vibrate_pattern(self.vibrator3_pub , self.num_vibrations_turn, self.duration)
        else:
            self.vibrate_pattern(self.vibrator4_pub , self.num_vibrations_turn, self.duration)

    def vibrate_right_turn(self):
        if self.updown:
            self.vibrate_pattern(self.vibrator4_pub , self.num_vibrations_turn, self.duration)
        else:
            self.vibrate_pattern(self.vibrator3_pub , self.num_vibrations_turn, self.duration)

    def vibrate_left_deviation(self):
        if self.updown:
            self.vibrate_pattern(self.vibrator3_pub , self.num_vibrations_deviation, self.duration)
        else:
            self.vibrate_pattern(self.vibrator4_pub , self.num_vibrations_deviation, self.duration)

    def vibrate_right_deviation(self):
        if self.updown:
            self.vibrate_pattern(self.vibrator4_pub , self.num_vibrations_deviation, self.duration)
        else:
            self.vibrate_pattern(self.vibrator3_pub , self.num_vibrations_deviation, self.duration)

    def vibrate_front(self):
        self.vibrate_pattern(self.vibrator1_pub, self.num_vibrations_confirmation,
                             self.duration_single_vibration)

    def vibrate_back(self):
        self.vibrate_pattern(self.vibrator2_pub, self.num_vibrations_confirmation,
                             self.duration_single_vibration)

    def vibrate_pattern(self, vibrator_pub, number_vibrations, duration):
        i=0
        while True:
            self.vibrate(vibrator_pub, 0.05*duration)
            i=i+1
            if i >= number_vibrations:
                break
            rospy.sleep(self.sleep/1000.0)

