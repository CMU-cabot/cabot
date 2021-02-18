#!/usr/bin/env python

# Copyright (c) 2021  IBM Corporation
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
# -*- coding: utf-8 -*-

import json
import argparse

import numpy as np

import rospy
from geometry_msgs.msg import Point, Quaternion, Pose
from geometry_msgs.msg import PoseWithCovarianceStamped

from cartographer_ros_msgs.msg import *
from cartographer_ros_msgs.srv import *

# TrajectoryStates
#   ACTIVE = 0
#   FINISHED = 1
#   FROZEN = 2
#   DELETED = 3

class TrajectoryRestarter:
    def __init__(self, configuration_directory, configuration_basename):
        self._count = 0
        self._configuration_directory = configuration_directory
        self._configuration_basename = configuration_basename

    def finish_last_trajectory(self):
        # get trajectory states
        res0 = get_trajectory_states()
        print(res0)
        last_trajectory_id = res0.trajectory_states.trajectory_id[-1]
        last_trajectory_state = ord(res0.trajectory_states.trajectory_state[-1]) # uint8 -> int

        if last_trajectory_state in [TrajectoryStates.ACTIVE]:
            # finish trajectory only if the trajectory is active.
            trajectory_id_to_finish = last_trajectory_id
            res1 = finish_trajectory(trajectory_id_to_finish)
            print(res1)
            # wait for completing finish_trajectory
            rospy.sleep(1)

    def restart_trajectory_with_pose(self, pose_with_covariance):
        initial_pose = pose_with_covariance.pose

        self.finish_last_trajectory()

        # start trajectory
        configuration_directory = self._configuration_directory
        configuration_basename = self._configuration_basename
        use_initial_pose = True
        relative_to_trajectory_id = 0

        res2 = start_trajectory(configuration_directory,
                                configuration_basename,
                                use_initial_pose,
                                initial_pose,
                                relative_to_trajectory_id
                                )
        print(res2)

        status_code = res2.status.code

        return status_code

    def pose_fix_callback(self,message):
        if self._count == 0:
            status_code = self.restart_trajectory_with_pose(message.pose)
            if status_code == 0: # "Success."
                print("trajectory started successfully.")
                self._count += 1

    def initialpose_callback(self,message):
        status_code = self.restart_trajectory_with_pose(message.pose)
        if status_code == 0: # "Success."
            print("trajectory started successfully.")

if __name__ == "__main__":
    rospy.init_node("trajectory_restarter")
    rospy.wait_for_service('get_trajectory_states')
    rospy.wait_for_service('finish_trajectory')
    rospy.wait_for_service('start_trajectory')
    get_trajectory_states = rospy.ServiceProxy('get_trajectory_states', GetTrajectoryStates)
    finish_trajectory = rospy.ServiceProxy('finish_trajectory', FinishTrajectory)
    start_trajectory = rospy.ServiceProxy('start_trajectory', StartTrajectory)

    configuration_directory = rospy.get_param("~configuration_directory")
    configuration_basename = rospy.get_param("~configuration_basename")

    trajectory_restarter = TrajectoryRestarter(configuration_directory, configuration_basename)

    sub = rospy.Subscriber("pose_fix", PoseWithCovarianceStamped, trajectory_restarter.pose_fix_callback)
    sub_initialpose = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, trajectory_restarter.initialpose_callback)

    rospy.spin()
