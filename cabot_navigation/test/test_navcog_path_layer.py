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
"""Test turn detector module"""

import sys
import unittest
import roslib
import json
import rospy
import geometry_msgs.msg
import nav_msgs.msg
import rosbag

PKG = 'cabot_navigation'
roslib.load_manifest(PKG)
sys.path.append('../src')

from cabot_ui import geojson, datautil
from cabot_ui.navigation import Navigation

USE_BAG = False

## test

class TestNavCogPathLayer(unittest.TestCase):
    """Test class"""

    def setUp(self):
        print "setUp"
        import os
        self.dir_path = os.path.dirname(os.path.realpath(__file__))

        if USE_BAG:
            self.bag = rosbag.Bag(os.path.join(self.dir_path, 'data','path.bag'))
        else:
            # test data from server
            # to record bag file, see navcog_path_layer.test
            self.node = Navigation(anchor_file=self.dir_path + "/data/test_map.yaml")
            while not self.node.i_am_ready:
                print "wait"
                rospy.sleep(1)

    def test_from_bag(self):
        if not USE_BAG:
            return
        rospy.loginfo("test_from_bag")
        pub = rospy.Publisher("/path", nav_msgs.msg.Path, queue_size=1)
        for topic, msg, t in self.bag.read_messages(topics=['/path']):
            pub.publish(msg)
            rospy.sleep(1)
            pub.publish(msg)
            for i in range(1, 10):
                print "wait costmap update"
                rospy.sleep(1)

    def test_path_1(self):
        if not USE_BAG:
            self._test_path("EDITOR_node_1496171299873")  # kitchen

    def test_path_2(self):
        if not USE_BAG:
            self._test_path("EDITOR_node_1490023596125")  # gates elevator

    def test_path_3(self):
        if not USE_BAG:
            self._test_path("EDITOR_node_1497843690742")  # wean library

    def _test_path(self, to_id):
        path = self._get_path_to(to_id)

        while path:
            self.assertGreater(len(path.poses), 0)
            print "Pose length:", len(path.poses)
            for i in range(1, 10):
                print "wait costmap update"
                rospy.sleep(1)

            path = self._get_next_path()

    def _get_next_path(self):
        if len(self.node.sub_routes) == 0:
            return None

        self.node._navigate_next_sub_route()
        self._plan = None
        for i in range(1, 30):
            if self._plan is not None:
                return self._plan
            print "wait the path"
            rospy.sleep(1)
        self.assertTrue(False)  # Fail

    def _get_path_to(self, to_id):
        self.node.set_destination(to_id)
        rospy.Subscriber("/path",
                         nav_msgs.msg.Path,
                         self._plan_callback)
        self._plan = None
        for i in range(1, 30):
            if self._plan is not None:
                return self._plan
            print "wait the path"
            rospy.sleep(1)
        self.assertTrue(False)  # Fail

    def _plan_callback(self, msg):
        self._plan = msg


if __name__ == "__main__":
    import rosunit

    rospy.init_node("test_navcog_path_layer")
    USE_BAG = rospy.get_param("~use_bag", False)
    rospy.loginfo("USE_BAG "+str(USE_BAG))
    rosunit.unitrun(PKG, 'test_navcog_path_layer', TestNavCogPathLayer)
    while True:
        rospy.sleep(1)
