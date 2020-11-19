#!/usr/bin/env python

 ###############################################################################
 # Copyright (c) 2020  Carnegie Mellon University
 #
 # Permission is hereby granted, free of charge, to any person obtaining a copy
 # of this software and associated documentation files (the "Software"), to deal
 # in the Software without restriction, including without limitation the rights
 # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 # copies of the Software, and to permit persons to whom the Software is
 # furnished to do so, subject to the following conditions:
 #
 # The above copyright notice and this permission notice shall be included in
 # all copies or substantial portions of the Software.
 #
 # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 # THE SOFTWARE.
 ###############################################################################

"""Test geoutil module"""

import sys
import unittest
import roslib
import math
import time

import rospy
import actionlib
import move_base_msgs.msg

from cabot_ui import geoutil
from tf.transformations import quaternion_from_euler

PKG = 'cabot_ui'
roslib.load_manifest(PKG)
sys.path.append('../src')

## test

class TestGeoutil(unittest.TestCase):
    """Test class"""

    def setUp(self):
        pass

    def test_spin(self):
        self._spin_client = actionlib.SimpleActionClient("spin", move_base_msgs.msg.MoveBaseAction)
        rospy.loginfo("waiting spin action")
        if self._spin_client.wait_for_server(timeout = rospy.Duration(5.0)):
            rospy.loginfo("spin is ready")
        else:
            rospy.logerr("spin is not ready")
            self._spin_client = None

        if self._spin_client:
            goal = move_base_msgs.msg.MoveBaseGoal()
            #use orientation.y for target spin angle
            goal.target_pose.pose.orientation.y = -1.57
            self._spin_client.send_goal(goal, self.callback)
            self._spin_client.wait_for_result(rospy.Duration.from_sec(10))

    def callback(self, *args):
        print("success", args)

if __name__ == "__main__":
    import rosunit
    rospy.init_node("test_spin_node")
    rosunit.unitrun(PKG, 'test_geoutil', TestGeoutil)


