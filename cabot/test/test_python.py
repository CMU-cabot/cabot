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

"""Test cabot module"""

import sys
import unittest
import traceback

import rospy
import roslib

import cabot.event

PKG = 'cabot'
roslib.load_manifest(PKG)
sys.path.append('../src')

## test

class TestPython(unittest.TestCase):
    """Test class"""

    def setUp(self):
        rospy.init_node("test_event_node")

    def test_button_parse1(self):
        b = cabot.event.BaseEvent.parse("button_up_1")
        self.assertEqual(b.type, "button")
        self.assertEqual(b.up, True)
        self.assertEqual(b.down, False)
        self.assertEqual(b.hold, False)
        self.assertEqual(b.button, 1)
        
    def test_button_parse2(self):
        b = cabot.event.BaseEvent.parse("button_down_2")
        self.assertEqual(b.type, "button")
        self.assertEqual(b.up, False)
        self.assertEqual(b.down, True)
        self.assertEqual(b.hold, False)
        self.assertEqual(b.button, 2)

    def test_button_parse3(self):
        b = cabot.event.BaseEvent.parse("button_hold_3")
        self.assertEqual(b.type, "button")
        self.assertEqual(b.up, False)
        self.assertEqual(b.down, False)
        self.assertEqual(b.hold, True)
        self.assertEqual(b.button, 3)

    def test_click_parse1(self):
        b = cabot.event.BaseEvent.parse("click_1_1")
        self.assertEqual(b.type, "click")
        self.assertEqual(b.buttons, 1)
        self.assertEqual(b.count, 1)
        
    def test_click_parse2(self):
        b = cabot.event.BaseEvent.parse("click_2_3")
        self.assertEqual(b.type, "click")
        self.assertEqual(b.buttons, 2)
        self.assertEqual(b.count, 3)

    def test_joy_parse1(self):
        b = cabot.event.BaseEvent.parse("joyclick_1_1")
        self.assertEqual(b.type, "joyclick")
        self.assertEqual(b.buttons, 1)
        self.assertEqual(b.count, 1)
        
    def test_joy_parse2(self):
        b = cabot.event.BaseEvent.parse("joyclick_2_3")
        self.assertEqual(b.type, "joyclick")
        self.assertEqual(b.buttons, 2)
        self.assertEqual(b.count, 3)

    def test_joy_parse3(self):
        b = cabot.event.BaseEvent.parse("joybutton_up_1")
        self.assertEqual(b.type, "joybutton")
        self.assertEqual(b.up, True)
        self.assertEqual(b.down, False)
        self.assertEqual(b.hold, False)
        self.assertEqual(b.button, 1)
        
    def test_joy_parse4(self):
        b = cabot.event.BaseEvent.parse("joybutton_down_3")
        self.assertEqual(b.type, "joybutton")
        self.assertEqual(b.up, False)
        self.assertEqual(b.down, True)
        self.assertEqual(b.hold, False)
        self.assertEqual(b.button, 3)

    def test_joy_parse5(self):
        b = cabot.event.BaseEvent.parse("joybutton_hold_2")
        self.assertEqual(b.type, "joybutton")
        self.assertEqual(b.up, False)
        self.assertEqual(b.down, False)
        self.assertEqual(b.hold, True)
        self.assertEqual(b.button, 2)


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(PKG, 'test_python', TestPython)
