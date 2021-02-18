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
"""Test cabot.menu module"""

import sys
import unittest
import traceback

import rospy
import roslib

from cabot_ui import menu

PKG = 'cabot_ui'
roslib.load_manifest(PKG)
sys.path.append('../src')

## test

class TestMenu(unittest.TestCase):
    """Test class"""

    def setUp(self):
        rospy.init_node("test_menu_node")

    def test_save_value(self):
        """test saving value"""
        rospy.set_param("test_menu/speech_speed_step1/value", 50)
        menu1 = menu.Menu.create_menu({"menu":"adjust_menu_step1"},
                                      name_space="test_menu")
        self.assertEqual(menu1.value, 50)
        menu1.prev()
        self.assertEqual(menu1.value, 51)
        menu2 = menu.Menu.create_menu({"menu":"adjust_menu_step1"},
                                      name_space="test_menu")
        self.assertEqual(menu2.value, 51)

    def test_save_value_step5(self):
        """test saving value with different step"""
        rospy.set_param("test_menu/speech_speed_step5/value", 50)
        menu1 = menu.Menu.create_menu({"menu":"adjust_menu_step5"},
                                      name_space="test_menu")
        self.assertEqual(menu1.value, 50)
        menu1.prev()
        self.assertEqual(menu1.value, 55)
        menu2 = menu.Menu.create_menu({"menu":"adjust_menu_step5"},
                                      name_space="test_menu")
        self.assertEqual(menu2.value, 55)

    def test_create_menu(self):
        """test creating menu"""
        try:
            main_menu = menu.Menu.create_menu({"menu":"main_menu"},
                                              name_space="test_menu")
            self.assertIsNotNone(main_menu)
        except KeyError:
            traceback.print_exc()
            self.fail("fail create menu")

    def test_fail_menu1(self):
        """test failure case 1 of menu creation"""
        with self.assertRaises(KeyError):
            menu.Menu.create_menu({"menu":"fail_menu1"},
                                  name_space="test_menu")

    def test_fail_menu2(self):
        """test failure case 2 of menu creation"""
        with self.assertRaises(ValueError):
            menu.Menu.create_menu({"menu":"fail_menu2"},
                                  name_space="test_menu")

    def test_fail_menu3(self):
        """test failure case 3 of menu creation"""
        with self.assertRaises(ValueError):
            menu.Menu.create_menu({"menu":"fail_menu3"},
                                  name_space="test_menu")

    def test_fail_menu4(self):
        """test failure case 4 of menu creation"""
        with self.assertRaises(ValueError):
            menu.Menu.create_menu({"menu":"fail_menu4"},
                                  name_space="test_menu")

    def test_explore_menu(self):
        """test menu exploration"""
        main_menu = menu.Menu.create_menu({"menu":"main_menu"},
                                          name_space="test_menu")

        self.assertEqual(main_menu.title, "MAIN_MENU")
        f = open("speech-test.txt", "w")
        self.explore(main_menu, f)

    def explore(self, item, f):
        """explore menu"""

        if item.type == menu.Menu.List:
            first = _next = item.next()
            while True:
            self.assertIsNotNone(item.description)
                f.write("%s\n" %(item.description))
                
                if _next.can_explore:
                    self.explore(_next, f)

                _next = item.next()
                if first == _next:
                    break

        elif item.type == menu.Menu.Adjust:
            last = item.value
            count = 0
            while True:
                now = item.next()
                rospy.loginfo(now)
                self.assertIsNotNone(item.description)
                f.write("%s\n" %(item.description))
                count += 1
                if now == last:
                    self.assertEqual(now, item.min)
                    break
                if count > 10000:
                    self.fail("too much repeat")
                    return
                last = now
            count = 0
            while True:
                now = item.prev()
                rospy.loginfo(now)
                self.assertIsNotNone(item.description)
                f.write("%s\n" %(item.description))
                count += 1
                if now == last:
                    self.assertEqual(now, item.max)
                    break
                if count > 10000:
                    self.fail("too much repeat")
                    return
                last = now

    def test_get_menu_by_name(self):
        main_menu = menu.Menu.create_menu({"menu":"main_menu"}, name_space="test_menu")
        self.assertIsNotNone(main_menu)

        speed_menu = main_menu.get_menu_by_identifier("max_velocity_menu")
        self.assertIsNotNone(speed_menu)


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(PKG, 'test_menu', TestMenu)
