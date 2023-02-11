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
"""Test navigation module"""

import json
import os
import threading
import time
import unittest

import rclpy
import rclpy.node
import rclpy.time
import tf2_ros
import tf_transformations
from geometry_msgs.msg import TransformStamped

from cabot_ui import geojson, datautil
from cabot_ui.navigation import Navigation, NavigationInterface
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil
from cabot_ui import navgoal, geoutil
from mf_localization_msgs.msg import MFLocalizeStatus


class TestNavigationNode(unittest.TestCase, navgoal.GoalInterface, NavigationInterface):

    def update_pose(self, **kwargs):
        pass

    def i_am_ready(self):
        pass

    def activity_log(self, category="", text="", memo=""):
        pass

    """Test class"""
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self):
        print("setUp")

        self.node = rclpy.node.Node("test_navigation_node_node")
        CaBotRclpyUtil.initialize(self.node)
        self.runner = NavigationRunner(self.node)

        br = tf2_ros.TransformBroadcaster(self.node)

        def send_transform():
            msg = TransformStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.child_frame_id = "/map"
            msg.header.frame_id = "/base_footprint"
            msg.transform.translation.x = 0.0
            msg.transform.translation.y = 0.0
            msg.transform.translation.z = 0.0
            q = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
            msg.transform.rotation.x = q[0]
            msg.transform.rotation.y = q[1]
            msg.transform.rotation.z = q[2]
            msg.transform.rotation.w = q[3]
            br.sendTransform(msg)

        rate = self.node.create_timer(0.1, send_transform)

        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        du = datautil.DataUtil()

        du.init_by_data(landmarks=du.get_landmarks(self.dir_path+"/data/landmarks1.json"),
                        node_map=du.get_node_map(self.dir_path+"/data/node_map1.json"),
                        features=du.get_features(self.dir_path+"/data/features1.json"))

        self.nav = Navigation(self.node, datautil_instance=du,
                              anchor_file=self.dir_path+"/data/test_map.yaml",
                              wait_for_action=False)
        self.nav.delegate = self
        # dummy local pose
        self.nav.current_local_pose = lambda: geoutil.Pose(x=0.0, y=0.0, z=0.0, r=0.0)
        self.nav.current_ros_pose = lambda: geoutil.Pose(x=0.0, y=0.0, z=0.0, r=0.0)
        self.nav.current_global_pose = lambda: geoutil.Latlng(lat=0.0, lng=0.0)
        self.nav.current_local_odom_pose = lambda: geoutil.Pose(x=0.0, y=0.0, z=0.0, r=0.0)
        # dummy localize status
        self.nav.localize_status = MFLocalizeStatus.TRACKING

        while not self.nav.i_am_ready:
            rclpy.spin_once(self.node)

    def test_making_route(self):
        f3 = open(self.dir_path+"/data/route1.json")
        r1 = geojson.Object.marshal_list(json.load(f3))

        self.assertEqual(r1[6].is_leaf, True)
        self.assertLess(r1[6].length, 3)
        self.assertIsInstance(r1[6], geojson.RouteLink)
        last = r1[6].start_node.local_geometry

        r2 = navgoal.make_goals(self.nav, r1, self.nav._anchor)
        self.assertIsNotNone(r2)
        self.assertEqual(len(r2), 1)
        self.assertIsInstance(r2[0], navgoal.NavGoal)
        self.assertIsNotNone(r2[0].navcog_route)
        self.assertIsNotNone(r2[0].ros_path)
        self.assertIsNotNone(r2[0].pois)

        self.assertAlmostEqual(last.x, r2[0].x)
        self.assertAlmostEqual(last.y, r2[0].y)
        self.assertTrue(r2[0].is_last)

    def test_through_elevator(self):
        f3 = open(self.dir_path+"/data/route2.json")
        r1 = geojson.Object.marshal_list(json.load(f3))
        r2 = navgoal.make_goals(self.nav, r1, self.nav._anchor)
        self.assertIsNotNone(r2)
        self.assertEqual(len(r2), 7)
        self.assertIsInstance(r2[0], navgoal.NavGoal)
        self.assertIsNotNone(r2[0].navcog_route)
        self.assertIsNotNone(r2[0].ros_path)
        self.assertIsNotNone(r2[0].pois)

        self.assertIsInstance(r2[1], navgoal.ElevatorWaitGoal)
        self.assertIsInstance(r2[2], navgoal.ElevatorInGoal)
        self.assertIsInstance(r2[3], navgoal.ElevatorTurnGoal)
        self.assertIsInstance(r2[4], navgoal.ElevatorFloorGoal)
        self.assertIsInstance(r2[5], navgoal.ElevatorOutGoal)
        self.assertIsInstance(r2[6], navgoal.NavGoal)

    def test_pause_navigation(self):
        f3 = open(self.dir_path+"/data/route1.json")
        groute = geojson.Object.marshal_list(json.load(f3))
        self.nav._sub_goals = navgoal.make_goals(self.nav, groute, self.nav._anchor)
        print(self.nav._sub_goals)
        self.assertEqual(len(self.nav._sub_goals), 1)

        self.nav._navigate_next_sub_goal()

        self.assertEqual(len(self.nav._sub_goals), 0)
        time.sleep(1)
        self.nav.pause_navigation()
        self.assertEqual(len(self.nav._sub_goals), 1)

        self.nav._navigate_next_sub_goal()
        time.sleep(1)
        self.assertEqual(len(self.nav._sub_goals), 0)

        self.nav.cancel_navigation()


class NavigationRunner:
    def __init__(self, node, context=None):
        self._node = node
        self._thread = threading.Thread(
            target=self._run,
        )
        self._run = True
        self._context = context

    def start(self):
        self._thread.start()

    def stop(self):
        self._run = False
        self._thread.join(timeout=5.0)
        if self._thread.is_alive():
            raise Exception('Timed out waiting')

    def _run(self):
        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor(context=self._context)
        executor.add_node(self._node)
        while self._run:
            print(F"running {self._run}")
            executor.spin_once(timeout_sec=1.0)
