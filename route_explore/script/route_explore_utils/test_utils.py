#!/usr/bin/python
# -*- coding: utf-8 -*-

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

import math
import random

from geometry_msgs.msg import Point, Pose
import tf

from route_explore_utils import geometry_utils


def get_random_robot_pose(costmap, lethal_cost_threshold):
    while True:
        robot_point = [0.0, 0.0]
        robot_point[0] = random.uniform(costmap.origin.position.x, costmap.origin.position.x+costmap.width*costmap.resolution)
        robot_point[1] = random.uniform(costmap.origin.position.y, costmap.origin.position.y+costmap.height*costmap.resolution)
        if geometry_utils.check_is_point_free(costmap, lethal_cost_threshold, Point(robot_point[0], robot_point[1], 0.0)):
            break
    robot_yaw = random.uniform(-math.pi, math.pi)
    robot_quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, robot_yaw)

    robot_pose = Pose()
    robot_pose.position = Point(robot_point[0], robot_point[1], 0.0)
    robot_pose.orientation.x = robot_quaternion[0]
    robot_pose.orientation.y = robot_quaternion[1]
    robot_pose.orientation.z = robot_quaternion[2]
    robot_pose.orientation.w = robot_quaternion[3]
    return robot_pose, robot_yaw