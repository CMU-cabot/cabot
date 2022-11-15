#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2022  IBM Corporation, Carnegie Mellon University, and others
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

import abc
import os
import yaml

from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates


class AbstractSimulatorNode(abc.ABC):

    def __init__(self, node: Node, topic_name: str):
        self._node = node
        self._logger = node.get_logger()

        node.declare_parameter('verbose', False)
        self._verbose = node.get_parameter('verbose')

        node.declare_parameter('wireless_config_file', '')
        wireless_config_file = node.get_parameter('wireless_config_file')
        wireless_config_basedir = os.path.dirname(wireless_config_file.value)
        wireless_config = None
        with open(wireless_config_file.value) as stream:
            wireless_config = yaml.safe_load(stream)

        floor_list = wireless_config['floor_list'] if 'floor_list' in wireless_config else None
        model = wireless_config['model'] if 'model' in wireless_config else None
        self._model_name = model['name'] if model and 'name' in model else None
        robot = wireless_config['robot'] if 'robot' in wireless_config else None
        self._robot_name = model['name'] if robot and 'name' in robot else None

        self._simulator = self.create_simulator(wireless_config, wireless_config_basedir, self._verbose)

        self._publisher = node.create_publisher(String, topic_name, 10)
        self._model_sub = node.create_subscription(ModelStates, "gazebo/model_states", self.model_callback, 10)

        self._cutoff = -100

        floors = []
        heights = []
        for floor_height in floor_list:
            floor = floor_height["floor"]
            height = floor_height["height"]
            floors.append(floor)
            heights.append(height)

        self._floors = floors
        self._heights = heights

        self._previous_pub_time = None
        self._frequency = 1.0  # Hz

    @abc.abstractclassmethod
    def create_simulator(self, wireless_config, wireless_config_basedir, verbose):
        pass

    @abc.abstractclassmethod
    def simulate_message(self, timestamp, x, y, z, floor):
        pass

    def get_robot_pose(self, message):
        # get the position of the robot relative to the model
        mesh_name = self._model_name
        robot_name = self._robot_name

        names = message.name
        poses = message.pose

        pose_dict = {}
        for i, name in enumerate(names):
            pose_dict[name] = poses[i]

        x_mesh = pose_dict[mesh_name].position.x
        y_mesh = pose_dict[mesh_name].position.y
        z_mesh = pose_dict[mesh_name].position.z

        x_robot = pose_dict[robot_name].position.x
        y_robot = pose_dict[robot_name].position.y
        z_robot = pose_dict[robot_name].position.z

        x_r2m = x_robot - x_mesh
        y_r2m = y_robot - y_mesh
        z_r2m = z_robot - z_mesh

        # convert z to floor
        floors = self._floors
        heights = self._heights
        floor = None

        if 1 < len(floors):
            for i in range(0, len(floors)):
                floor_l = floors[i]
                floor_u = floors[i+1]
                height_l = heights[i]
                height_u = heights[i+1]

                z_floor = z_r2m - floor_l

                if z_r2m < height_l:
                    floor = floor_l
                    z_floor = z_r2m - height_l
                    break
                elif height_u < z_r2m:
                    floor = floor_u
                    z_floor = z_r2m - height_u
                    if i+1 == len(floors)-1:
                        break
                else:
                    floor = floor_l + (floor_u - floor_l)/(height_u - height_l) * (z_r2m - height_l)
                    if round(floor) == floor_l:
                        z_floor = z_r2m - height_l
                    else:
                        z_floor = z_r2m - height_u

                    break
            floor = round(floor)  # float -> int

        elif len(floors) == 1:
            floor = floors[0]
            height = heights[0]
            z_floor = z_r2m - height
        else:  # len(floors) == 0
            raise RuntimeError("floors are not defined.")

        return x_r2m, y_r2m, z_floor, floor

    def model_callback(self, message):
        now = self._node.get_clock().now()

        # keep message publish frequency
        if self._previous_pub_time is None:
            self._previous_pub_time = self._node.get_clock().now()
        elif (now - self._previous_pub_time) < Duration(seconds=1.0/self._frequency):
            return
        self._previous_pub_time = now

        # get robot pose from the model_states message
        x_r2m, y_r2m, z_floor, floor = self.get_robot_pose(message)

        if self._verbose:
            print("x_r2m, y_r2m, z_floor, floor=", x_r2m, y_r2m, z_floor, floor)

        # simulate beacons
        if self._simulator is None:
            return
        msg = self.simulate_message(now, x_r2m, y_r2m, z_floor, floor)

        # publish message
        self._publisher.publish(msg)
