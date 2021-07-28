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
"""Test navcognode module"""

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
from cabot_ui.navcog import NavCog

from cabot_ui import menu

PKG = 'cabot_ui'
roslib.load_manifest(PKG)
sys.path.append('../src')

## test

class TestNavCogNode(unittest.TestCase):
    """Test class"""
    
    def setUp(self):
        print("setUp")
        import os 
        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        du = datautil.DataUtil()
        
        du.init_by_data(landmarks=du.get_landmarks(self.dir_path+"/data/landmarks.json"),
                        node_map=du.get_node_map(self.dir_path+"/data/node_map.json"),
                        features=du.get_features(self.dir_path+"/data/features.json"))
        
        self.node = NavCog(datautil_instance=du,
                           anchor_file=self.dir_path+"/data/test_map.yaml")
        while not self.node.is_ready:
            print("wait")
            rospy.sleep(1)

    def test_pause_navigation(self):
        self.node.pause_navigation()
        rospy.sleep(2)

    def test_convert_route(self):
        f3 = open(self.dir_path+"/data/route1.json")
        route = geojson.Object.marshal_list(json.load(f3))
        route2 = self.node._convert_route(route)

        self.assertEqual(route[6].is_leaf, True)
        self.assertLess(route[6].length, 3)
        self.assertIsInstance(route[6], geojson.RouteLink)
        last = route[6].start_node.local_geometry
        self.assertAlmostEqual(last.x, route2[-1].x)
        self.assertAlmostEqual(last.y, route2[-1].y)

    def test_directional_poi(self):
        poi1 = geojson.Object.get_object_by_id("EDITOR_poi_1490196409525")
        poi2 = geojson.Object.get_object_by_id("EDITOR_poi_1503069612611")

        targetx = int(poi1.local_pose.x*10)
        
        for x in range(0, targetx):
            pose = geoutil.Pose(x=x/10.0, y=0, r=0)
            poi1.is_approaching(pose)
            poi1.is_approached(pose)
            poi2.is_approaching(pose)
            poi2.is_approached(pose)

        print(poi1.local_pose)
        print(poi2.local_pose)

        self.assertTrue(poi1._was_approaching)
        self.assertTrue(poi1._was_approached)
        self.assertFalse(poi2._was_approaching)
        self.assertFalse(poi2._was_approached)

        
def test():
    rosunit.unitrun(PKG, 'test_navcognode', TestNavCogNode)
    rospy.signal_shutdown("finish")
        
if __name__ == "__main__":
    rospy.init_node("test_navcognode_node")
    import rosunit

    t = threading.Thread(target=test)
    t.start()

    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        br.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "/base_link",
                         "/map")
        
        rospy.sleep(0.1)
