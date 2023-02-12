#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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

import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

from cartographer_ros_msgs.msg import TrajectoryStates
from cartographer_ros_msgs.srv import GetTrajectoryStates
from cartographer_ros_msgs.srv import FinishTrajectory
from cartographer_ros_msgs.srv import StartTrajectory

# TrajectoryStates
#   ACTIVE = 0
#   FINISHED = 1
#   FROZEN = 2
#   DELETED = 3


class TrajectoryRestarter:
    def __init__(self, node, configuration_directory, configuration_basename):
        self.node = node
        self.logger = node.get_logger()
        self._count = 0
        self._configuration_directory = configuration_directory
        self._configuration_basename = configuration_basename

    def finish_last_trajectory(self):
        # get trajectory states
        res0 = get_trajectory_states()
        self.logger.info(F"{res0}")
        last_trajectory_id = res0.trajectory_states.trajectory_id[-1]
        last_trajectory_state = res0.trajectory_states.trajectory_state[-1]

        if last_trajectory_state in [TrajectoryStates.ACTIVE]:
            # finish trajectory only if the trajectory is active.
            trajectory_id_to_finish = last_trajectory_id
            res1 = finish_trajectory(trajectory_id_to_finish)
            self.logger.info(F"{res1}")
            # wait for completing finish_trajectory
            time.sleep(1)

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
        self.logger.info(F"{res2}")

        status_code = res2.status.code

        return status_code

    def pose_fix_callback(self, message):
        if self._count == 0:
            status_code = self.restart_trajectory_with_pose(message.pose)
            if status_code == 0:  # "Success."
                self.logger.info("trajectory started successfully.")
                self._count += 1

    def initialpose_callback(self, message):
        status_code = self.restart_trajectory_with_pose(message.pose)
        if status_code == 0:  # "Success."
            self.logger.info("trajectory started successfully.")


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("trajectory_restarter")

    get_trajectory_states = node.create_client(GetTrajectoryStates, 'get_trajectory_states')
    get_trajectory_states.wait_for_service()
    finish_trajectory = node.create_client(FinishTrajectory, 'finish_trajectory')
    finish_trajectory.wait_for_service()
    start_trajectory = node.create_client(StartTrajectory, 'start_trajectory')
    start_trajectory.wait_for_service()

    configuration_directory = node.declare_parameter("configuration_directory", '').value
    configuration_basename = node.declare_parameter("configuration_basename", '').value

    trajectory_restarter = TrajectoryRestarter(node, configuration_directory, configuration_basename)

    sub = node.create_subscription(PoseWithCovarianceStamped, "pose_fix", trajectory_restarter.pose_fix_callback, 10)
    sub_initialpose = node.create_subscription(PoseWithCovarianceStamped, "initialpose", trajectory_restarter.initialpose_callback, 10)

    try:
        rclpy.spin(node)
    except:  # noqa: E722
        pass
