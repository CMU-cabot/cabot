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

from cabot_ui import geojson, datautil, geoutil
from cabot_ui.navigation import Navigation

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
        import os 
        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        du = datautil.DataUtil()
        
        du.init_by_data(landmarks=du.get_landmarks(self.dir_path+"/data/landmarks1.json"),
                        node_map=du.get_node_map(self.dir_path+"/data/node_map1.json"),
                        features=du.get_features(self.dir_path+"/data/features1.json"))
        
        self.node = Navigation(datautil_instance=du,
                               anchor_file=self.dir_path+"/data/test_map.yaml")
        while not self.node.i_am_ready:
            print("wait")
            rospy.sleep(1)

    def _test_making_route(self):
        f3 = open(self.dir_path+"/data/route1.json")
        r1 = geojson.Object.marshal_list(json.load(f3))

        self.assertEqual(r1[6].is_leaf, True)
        self.assertLess(r1[6].length, 3)
        self.assertIsInstance(r1[6], geojson.RouteLink)
        last = r1[6].start_node.local_geometry

        r2 = navgoal.make_goals(self, r1, self.node._anchor)
        self.assertEqual(len(r2), 1)
        self.assertIsInstance(r2[0], navgoal.NavGoal)
        self.assertIsNotNone(r2[0].navcog_route)
        self.assertIsNotNone(r2[0].ros_path)
        self.assertIsNotNone(r2[0].pois)

        self.assertAlmostEqual(last.x, r2[0].x)
        self.assertAlmostEqual(last.y, r2[0].y)
        self.assertTrue(r2[0].is_last)

    def _test_making_route2(self):
        f3 = open(self.dir_path+"/data/route2.json")
        r1 = geojson.Object.marshal_list(json.load(f3))
        r2 = navgoal.make_goals(self, r1, self.node._anchor)
        self.assertEqual(len(r2), 4)
        self.assertIsInstance(r2[0], navgoal.NavGoal)
        self.assertIsNotNone(r2[0].navcog_route)
        self.assertIsNotNone(r2[0].ros_path)
        self.assertIsNotNone(r2[0].pois)

        self.assertIsInstance(r2[1], navgoal.ElevatorInGoal)
        self.assertIsInstance(r2[2], navgoal.ElevatorOutGoal)
        self.assertIsInstance(r2[3], navgoal.NavGoal)

    def test_pause_navigation(self):
        f3 = open(self.dir_path+"/data/route1.json")
        groute = geojson.Object.marshal_list(json.load(f3))
        self.node._sub_goals = navgoal.make_goals(self, groute, self.node._anchor)
        print(self.node._sub_goals)
        self.assertEqual(len(self.node._sub_goals), 1)
        
        self.node._navigate_next_sub_goal()
        rospy.sleep(1)
        self.assertEqual(len(self.node._sub_goals), 0)

        self.node.pause_navigation()
        self.assertEqual(len(self.node._sub_goals), 1)

        self.node._navigate_next_sub_goal()
        rospy.sleep(1)
        self.assertEqual(len(self.node._sub_goals), 0)

        self.node.cancel_navigation()


def test():
    rosunit.unitrun(PKG, 'test_navigation_node', TestNavigationNode)
    rospy.signal_shutdown("finish")
        
if __name__ == "__main__":
    rospy.init_node("test_navigation_node_node")
    import rosunit

    t = threading.Thread(target=test)
    t.start()

    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        br.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "/base_footprint",
                         "/map")
        
        rospy.sleep(0.1)
