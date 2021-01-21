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

from cabot_ui import geojson, datautil
from cabot_ui.turn_detector import TurnDetector

PKG = 'cabot_ui'
roslib.load_manifest(PKG)
sys.path.append('../src')

## test

class TestTurnDetector(unittest.TestCase):
    """Test class"""

    def test_turn_detection(self):
        import os
        self.dir_path = os.path.dirname(os.path.realpath(__file__))

        pub = rospy.Publisher("/move_base/NavfnROS/plan",
                              nav_msgs.msg.Path,
                              queue_size=1, latch=True)

        bag = rosbag.Bag(self.dir_path+"/data/path.bag")
        i=0
        for topic, msg, t in bag.read_messages():
            i+=1
            if i==i:
                pub.publish(msg)
                #rospy.sleep(3) # you can remove this
                result = TurnDetector.detects(msg)
                '''
                for turn in result:
                    print turn.angle
                    print turn.startPose
                '''
                self.assertGreater(len(result), 0)
                ## just for visualization
                rospy.sleep(1)



    def _plan_callback(self, msg):
        self._plan = msg

if __name__ == "__main__":
    import rosunit
    rospy.init_node("test_turn_detector")
    rosunit.unitrun(PKG, 'test_turn_detector', TestTurnDetector)
