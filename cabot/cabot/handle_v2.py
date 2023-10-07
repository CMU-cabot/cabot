#!/usr/bin/env python

# Copyright (c) 2020, 2022  Carnegie Mellon University
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

from rclpy.duration import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from cabot import button
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import UInt8
import time


class Handle:
    UNKNOWN = 0
    LEFT_TURN = 1
    RIGHT_TURN = 2
    LEFT_DEV = 3
    RIGHT_DEV = 4
    FRONT = 5
    LEFT_ABOUT_TURN = 6
    RIGHT_ABOUT_TURN = 7
    BUTTON_CLICK = 8
    BUTTON_HOLDDOWN = 9
    STIMULI_COUNT = 10
    stimuli_names = ["unknown", "left_turn", "right_turn", "left_dev", "right_dev",
                     "front", "left_about_turn", "right_about_turn", "button_click", "button_holddown"]

    @staticmethod
    def get_name(stimulus):
        return Handle.stimuli_names[stimulus]

    def __init__(self, node, realworld_use=True, event_listener=None, button_keys=[]):
        self.node = node
        self.logger = node.get_logger()
        self.event_listener = event_listener
        self.button_keys = button_keys
        self.number_of_buttons = len(button_keys)
        self.lastUp = [None]*self.number_of_buttons
        self.lastDwn = [None]*self.number_of_buttons
        self.upCount = [0]*self.number_of_buttons
        self.btnDwn = [False]*self.number_of_buttons
        self.power = 255
        self.vibrator1_pub = node.create_publisher(UInt8, '/cabot/vibrator1', 100)
        self.vibrator2_pub = node.create_publisher(UInt8, '/cabot/vibrator2', 100)
        self.vibrator3_pub = node.create_publisher(UInt8, '/cabot/vibrator3', 100)
        self.vibrator4_pub = node.create_publisher(UInt8, '/cabot/vibrator4', 100)
        for i in range(0, self.number_of_buttons):
            _ = node.create_subscription(Bool, F"/cabot/pushed_{i+1}",
                                         lambda msg, i=i: self.button_callback(msg, i), qos_profile_sensor_data, callback_group=MutuallyExclusiveCallbackGroup())

        self.duration = 15
        self.duration_single_vibration = 40
        self.duration_about_turn = 40
        self.duration_button_click = 5
        self.duration_button_holddown = 10
        self.sleep = 150
        self.updown = True
        self.num_vibrations_turn = 4
        self.num_vibrations_deviation = 2
        self.num_vibrations_about_turn = 2
        self.num_vibrations_confirmation = 1
        self.num_vibrations_button_click = 1
        self.num_vibrations_button_holddown = 1

        self.callbacks = [None]*Handle.STIMULI_COUNT
        self.callbacks[Handle.LEFT_TURN] = self.vibrate_left_turn
        self.callbacks[Handle.RIGHT_TURN] = self.vibrate_right_turn
        self.callbacks[Handle.LEFT_DEV] = self.vibrate_left_deviation
        self.callbacks[Handle.RIGHT_DEV] = self.vibrate_right_deviation
        self.callbacks[Handle.FRONT] = self.vibrate_front
        self.callbacks[Handle.LEFT_ABOUT_TURN] = self.vibrate_about_left_turn
        self.callbacks[Handle.RIGHT_ABOUT_TURN] = self.vibrate_about_right_turn
        self.callbacks[Handle.BUTTON_CLICK] = self.vibrate_button_click
        self.callbacks[Handle.BUTTON_HOLDDOWN] = self.vibrate_button_holddown

        self.eventSub = node.create_subscription(String, '/cabot/event', self.event_callback, 10, callback_group=MutuallyExclusiveCallbackGroup())

        self.double_click_interval = Duration(seconds=0.25)
        self.ignore_interval = Duration(seconds=0.05)
        self.holddown_interval = Duration(seconds=3.0)

    def button_callback(self, msg, index):
        self._button_check(msg, button.__dict__[self.button_keys[index]], index)

    def _button_check(self, msg, key, index):
        event = None
        now = self.node.get_clock().now()
        # detect change from button up to button down to emit a button down event
        if msg.data and not self.btnDwn[index] and \
           not (self.lastUp[index] is not None and
                now - self.lastUp[index] < self.ignore_interval):
            event = {"button": key, "up": False}
            self.btnDwn[index] = True
            self.lastDwn[index] = now # for holddown detection
        # detect change from button down to button up to emit a button up event
        if not msg.data and self.btnDwn[index]:
            event = {"button": key, "up": True}
            self.upCount[index] += 1
            self.lastUp[index] = now
            self.btnDwn[index] = False
        # detect the end of a series of button down/up events to emit a click event
        if self.lastUp[index] is not None and \
           not self.btnDwn[index] and \
           now - self.lastUp[index] > self.double_click_interval:
            # emit click event only if a hold down event is not detected
            if self.lastDwn[index] is not None:
                event = {"buttons": key, "count": self.upCount[index]}
            self.lastUp[index] = None
            self.upCount[index] = 0
        # detect button hold down to emit a holddown event
        if msg.data and self.btnDwn[index] and \
           (self.lastDwn[index] is not None and \
           now - self.lastDwn[index] > self.holddown_interval):
            event = {"holddown":key}
            # clear lastDwn[index] after holddown event emission to prevent multiple event emissions
            self.lastDwn[index] = None

        if event is not None:
            if self.event_listener is not None:
                self.event_listener(event)

    def event_callback(self, msg):
        name = msg.data
        if name in Handle.stimuli_names:
            index = Handle.stimuli_names.index(name)
            self.execute_stimulus(index)

    def execute_stimulus(self, index):
        self.logger.info("execute_stimulus, %d" % (index))
        callback = self.callbacks[index]
        if callback is not None:
            callback()
            self.logger.info("executed")

    def vibrate(self, pub):
        msg = UInt8()
        msg.data = self.power
        pub.publish(msg)

    def stop(self, pub):
        msg = UInt8()
        msg.data = 0
        pub.publish(msg)

    def vibrate_all(self, time):
        msg = UInt8()
        msg.data = self.power
        self.vibrator1_pub.publish(msg)
        self.vibrator2_pub.publish(msg)
        self.vibrator3_pub.publish(msg)
        self.vibrator4_pub.publish(msg)

        time.sleep(time)

        msg.data = 0
        self.vibrator1_pub.publish(msg)
        self.vibrator2_pub.publish(msg)
        self.vibrator3_pub.publish(msg)
        self.vibrator4_pub.publish(msg)

    def vibrate_left_turn(self):
        if self.updown:
            self.vibrate_pattern(self.vibrator3_pub, self.num_vibrations_turn, self.duration)
        else:
            self.vibrate_pattern(self.vibrator4_pub, self.num_vibrations_turn, self.duration)

    def vibrate_right_turn(self):
        if self.updown:
            self.vibrate_pattern(self.vibrator4_pub, self.num_vibrations_turn, self.duration)
        else:
            self.vibrate_pattern(self.vibrator3_pub, self.num_vibrations_turn, self.duration)

    def vibrate_left_deviation(self):
        if self.updown:
            self.vibrate_pattern(self.vibrator3_pub, self.num_vibrations_deviation, self.duration)
        else:
            self.vibrate_pattern(self.vibrator4_pub, self.num_vibrations_deviation, self.duration)

    def vibrate_right_deviation(self):
        if self.updown:
            self.vibrate_pattern(self.vibrator4_pub, self.num_vibrations_deviation, self.duration)
        else:
            self.vibrate_pattern(self.vibrator3_pub, self.num_vibrations_deviation, self.duration)

    def vibrate_front(self):
        self.vibrate_pattern(self.vibrator1_pub, self.num_vibrations_confirmation,
                             self.duration_single_vibration)

    def vibrate_about_left_turn(self):
        if self.updown:
            self.vibrate_pattern(
                self.vibrator3_pub, self.num_vibrations_about_turn, self.duration_about_turn)
        else:
            self.vibrate_pattern(
                self.vibrator4_pub, self.num_vibrations_about_turn, self.duration_about_turn)

    def vibrate_about_right_turn(self):
        if self.updown:
            self.vibrate_pattern(
                self.vibrator4_pub, self.num_vibrations_about_turn, self.duration_about_turn)
        else:
            self.vibrate_pattern(
                self.vibrator3_pub, self.num_vibrations_about_turn, self.duration_about_turn)

    def vibrate_back(self):
        self.vibrate_pattern(self.vibrator2_pub, self.num_vibrations_confirmation,
                             self.duration_single_vibration)

    def vibrate_button_click(self):
        self.vibrate_pattern(
            self.vibrator1_pub, self.num_vibrations_button_click, self.duration_button_click)

    def vibrate_button_holddown(self):
        self.vibrate_pattern(
            self.vibrator1_pub, self.num_vibrations_button_holddown, self.duration_button_holddown)

    def vibrate_pattern(self, vibrator_pub, number_vibrations, duration):
        i = 0
        mode = "NEW"

        if mode == "SAFETY":
            while True:
                for v in range(0, duration):
                    self.vibrate(vibrator_pub)
                    time.sleep(0.01)
                self.stop(vibrator_pub)
                self.stop(vibrator_pub)
                self.stop(vibrator_pub)

                i += 1
                if i >= number_vibrations:
                    break
                time.sleep(self.sleep/1000.0)

            # make sure it stops
            for v in range(0, 10):
                self.stop(vibrator_pub)

        if mode == "SIMPLE":
            for i in range(0, number_vibrations):
                self.vibrate(vibrator_pub)
                time.sleep(0.01*duration)
                self.stop(vibrator_pub)
                if i < number_vibrations - 1:
                    time.sleep(self.sleep/1000.0)
            self.stop(vibrator_pub)

        if mode == "NEW":
            for i in range(0, number_vibrations):
                msg = UInt8()
                msg.data = duration
                vibrator_pub.publish(msg)
                time.sleep(0.01*duration)
                if i < number_vibrations - 1:
                    time.sleep(self.sleep/1000.0)
