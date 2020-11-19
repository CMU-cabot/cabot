#!/usr/bin/env python

# Copyright 2020 Carnegie Mellon University
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

from cabot_ui import geojson, datautil
from cabot_ui.turn_detector import TurnDetector
from cabot_ui.navcog import NavCog

PKG = 'cabot_ui'
roslib.load_manifest(PKG)
sys.path.append('../src')

## test

class TestTurnDetector(unittest.TestCase):
    """Test class"""

    def setUp(self):
        print "setUp"
        import os 
        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        du = datautil.instance
        
        du.init_by_data(landmarks=du.get_landmarks(self.dir_path+"/data/landmarks.json"),
                        node_map=du.get_node_map(self.dir_path+"/data/node_map.json"),
                        features=du.get_features(self.dir_path+"/data/features.json"))
        
        self.node = NavCog(datautil_instance=du,
                           anchor_file=self.dir_path+"/data/test_map.yaml")
        while not self.node.is_ready:
            print "wait"
            rospy.sleep(1)
            
    def test_turn_detection_1(self):
        self._turn_detection_test("EDITOR_node_1490023596125") #gates elevator

    def test_turn_detection_2(self):
        self._turn_detection_test("EDITOR_node_1498750866007") #wean library

    def test_turn_detection_3(self):
        self._turn_detection_test("EDITOR_node_1496171299873") #RI kitchen

    def _turn_detection_test(self, to_id):
        path = self._get_path_to(to_id)
        pub = rospy.Publisher("/path",
                              nav_msgs.msg.Path,
                              queue_size=1, latch=True)
        pub.publish(path)
        self.assertGreater(len(path.poses), 0)
        print "Pose length:", len(path.poses)
        
        # do turn detection here
        result = TurnDetector.detects(path)
        self.assertGreater(len(result), 0) # have some results
        rospy.sleep(3)
        # add assertion

    
    def _get_path_to(self, to_id):
        node = geojson.Object.get_object_by_id(to_id)
        goal = node.local_geometry
        print goal

        msg = geometry_msgs.msg.PoseStamped()
        
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = goal.x
        msg.pose.position.y = goal.y
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 1

        # publish goal and wait until the path is available
        pub = rospy.Publisher("/move_base_simple/goal",
                              geometry_msgs.msg.PoseStamped,
                              queue_size=100)
        rospy.Subscriber("/move_base/NavfnROS/plan",
                         nav_msgs.msg.Path,
                         self._plan_callback)
        self._plan = None
        pub.publish(msg)
        for i in range(1,10):
            if self._plan is not None:
                return self._plan
            print "wait the path"
            rospy.sleep(1)
        self.assertTrue(False) #Fail

    def _plan_callback(self, msg):
        self._plan = msg

if __name__ == "__main__":
    import rosunit
    rospy.init_node("test_turn_detector_node")
    rosunit.unitrun(PKG, 'test_turn_detector', TestTurnDetector)
    while True:
        rospy.sleep(1)
