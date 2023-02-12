#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2021, 2022  IBM Corporation, Carnegie Mellon University, and others
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

import sys
import signal
import os
import threading
import yaml

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from mf_localization_msgs.srv import FloorChange
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState


class FloorTransition:
    def __init__(self, node, floor_list, robot_name, model_name):
        self._node = node
        self._logger = node.get_logger()
        floors = {}
        for floor_height in floor_list:
            floor = floor_height["floor"]
            height = floor_height["height"]
            floors[floor] = height

        self._floors = floors
        self._logger.info(str(self._floors))

        self._prev_floor_check = self._node.get_clock().now()

        self._robot_name = robot_name
        self._model_name = model_name

        self.service = self._node.create_service(FloorChange, "/floor_change", self.handle_floor_change)
        self.model_sub = self._node.create_subscription(ModelStates, "gazebo/model_states", self.model_callback, 10)
        self.gazeb_service = self._node.create_client(SetEntityState, "gazebo/set_entity_state")

    def model_callback(self, message):
        if (self._node.get_clock().now() - self._prev_floor_check) < Duration(seconds=0.5):
            return
        self._prev_floor_check = self._node.get_clock().now()

        # get the position of the robot relative to the model
        mesh_name = self._model_name
        robot_name = self._robot_name

        names = message.name
        poses = message.pose

        pose_dict = {}
        for i, name in enumerate(names):
            pose_dict[name] = poses[i]

        if mesh_name not in pose_dict:
            return
        if robot_name not in pose_dict:
            return

        z_mesh = pose_dict[mesh_name].position.z
        z_robot = pose_dict[robot_name].position.z
        z_r2m = z_robot - z_mesh

        self.robot_pose = pose_dict[robot_name]

        # convert z to floor
        floor = None
        min_score = 10000
        for f in self._floors:
            score = abs(z_r2m - self._floors[f])
            if score < min_score:
                min_score = score
                floor = f

        self.current_floor = floor

        self._logger.info(F"current_floor = {self.current_floor}")

    def handle_floor_change(self, request, response):
        next_floor = int(self.current_floor + request.diff.data)
        while next_floor not in self._floors:
            next_floor = int(next_floor + request.diff.data)

        self._logger.info(F"handle_floor_change, next_floor={next_floor}")

        if next_floor in self._floors:
            try:
                def async_func():
                    set_model_state_req = SetEntityState.Request()
                    set_model_state_req.state.name = self._robot_name
                    set_model_state_req.state.pose = self.robot_pose
                    set_model_state_req.state.pose.position.z = self._floors[next_floor]+0.01
                    self._logger.info(F"calling gazebo_service set z position to {set_model_state_req.state.pose.position.z}")
                    result = self.gazeb_service.call(set_model_state_req)
                    self._logger.info(F"gazebo service result: {result}")
                timer = threading.Timer(0, async_func)
                timer.daemon = True
                timer.start()
                response.status.code = 0
                response.status.message = "succeeded to change floor to {} @ {}".format(next_floor, self._floors[next_floor])
            except rclpy.ServiceException:
                response.status.code = 2
                response.status.message = "could not changed floor to {} @  {}".format(next_floor, self._floors[next_floor])

            self._logger.info(response.status.message)
        else:
            response.status.code = 1
            response.status.message = "could not find floor {}".format(next_floor)
            self._logger.error(response.status.message)

        return response


def receiveSignal(signal_num, frame):
    print("Received:", signal_num)
    sys.exit(0)


signal.signal(signal.SIGINT, receiveSignal)

if __name__ == "__main__":
    rclpy.init()
    node = Node("floor_transition_node")

    node.declare_parameter('wireless_config_file', '')
    wireless_config_file = node.get_parameter('wireless_config_file')
    wireless_config_basedir = os.path.dirname(wireless_config_file.value)
    wireless_config = None
    with open(wireless_config_file.value) as stream:
        wireless_config = yaml.safe_load(stream)

    floor_list = wireless_config['floor_list'] if 'floor_list' in wireless_config else []
    model = wireless_config['model'] if 'model' in wireless_config else None
    model_name = model['name'] if model and 'name' in model else None
    robot = wireless_config['robot'] if 'robot' in wireless_config else None
    robot_name = robot['name'] if robot and 'name' in robot else None

    ft = FloorTransition(node=node, floor_list=floor_list, model_name=model_name, robot_name=robot_name)

    rclpy.spin(node)
