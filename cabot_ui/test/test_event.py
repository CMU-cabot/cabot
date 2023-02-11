#!/usr/bin/env python3

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
"""Test cabot_ui.event module"""

import unittest

import cabot_ui.event
import cabot.event


class TestEvent(unittest.TestCase):
    """Test class"""

    def setUp(self):
        pass

    def test_basic(self):
        """test events"""
        e1 = cabot_ui.event.MenuEvent("next")
        e2 = cabot.event.ButtonEvent(1, False)
        e3 = cabot.event.ButtonEvent(1, False)
        print(e1)
        print(e2)
        print(e1 == e2)

        self.assertEqual(str(e1), "menu_next")
        self.assertFalse(e1 == e2)
        self.assertTrue(e2 != e3)

    def test_parse(self):
        e1 = cabot.event.BaseEvent.parse("menu_next")
        self.assertEqual(type(e1), cabot_ui.event.MenuEvent)
        self.assertEqual(e1.subtype, "next")

        e2 = cabot.event.BaseEvent.parse("navigation_pause")
        self.assertEqual(type(e2), cabot_ui.event.NavigationEvent)
        self.assertEqual(e2.subtype, "pause")

        e3 = cabot.event.BaseEvent.parse("navigation_destination;hogehoge")
        self.assertEqual(type(e3), cabot_ui.event.NavigationEvent)
        self.assertEqual(e3.subtype, "destination")
        self.assertEqual(e3.param, "hogehoge")
