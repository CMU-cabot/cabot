#!/usr/bin/env python

# Copyright (c) 2023  Carnegie Mellon University, IBM Corporation, and others
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
"""Test navigation module"""

import sys
import unittest
import traceback

import rospy
import roslib
import std_msgs.msg
import tf
import threading
import json
import math


from cabot_ui import geojson, datautil, geoutil
from cabot_ui.navigation_explore import NavigationExplore

from cabot_ui import menu
from cabot_ui import navgoal

PKG = 'cabot_ui'
roslib.load_manifest(PKG)
sys.path.append('../src')

## test

class TestNavigationNode(unittest.TestCase, navgoal.GoalInterface):
    """Test class"""
    
    def setUp(self):
        print("setUp")
        self.exp = NavigationExplore()

    def test_find_index1(self):
        self.exp.current_pose = geoutil.Pose(x=0, y=0, r=0)
        self.exp._last_explore_status["goal_candidates"] = [{"x":10, "y":0}]
        print(self.exp._find_current_index())
        self.assertEqual(self.exp._find_current_index(), 0)

    def test_find_index2(self):
        self.exp.current_pose = geoutil.Pose(x=0, y=0, r=0)
        self.exp._last_explore_status["goal_candidates"] = [{"x":10, "y":0}, {"x":10, "y":10}]
        self.assertEqual(self.exp._find_current_index(), 0)

    def test_find_index3(self):
        self.exp.current_pose = geoutil.Pose(x=0, y=0, r=0)
        self.exp._last_explore_status["goal_candidates"] = [{"x":5, "y":5}, {"x":-5, "y":-5}]
        self.assertEqual(self.exp._find_current_index(), 0.5)

    def test_find_index4(self):
        self.exp.current_pose = geoutil.Pose(x=0, y=0, r=-math.pi/4)
        self.exp._last_explore_status["goal_candidates"] = [{"x":10, "y":0}, {"x":0, "y": -10}]
        self.assertEqual(self.exp._find_current_index(), 0.5)

    def test_find_index5(self):
        self.exp.current_pose = geoutil.Pose(x=0, y=0, r=0)
        self.exp._last_explore_status["goal_candidates"] = [{"x":5, "y":-5}, {"x":-5, "y":0}, {"x":5, "y":5}]
        self.assertEqual(self.exp._find_current_index(), -0.5)



def test():
    rosunit.unitrun(PKG, 'test_navigation_explore_node', TestNavigationNode)
    rospy.signal_shutdown("finish")


if __name__ == "__main__":
    import rosunit

    rospy.init_node("test_navcognode_node")
    import rosunit

    t = threading.Thread(target=test)
    t.start()

    while not rospy.is_shutdown():
        
        rospy.sleep(0.1)

