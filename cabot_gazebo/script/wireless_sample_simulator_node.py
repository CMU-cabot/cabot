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

import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from wireless_abstract_simulator_node import AbstractSimulatorNode
from wireless_utils import extract_samples
from wireless_sample_simulator import SampleSimulator


class SimpleSampleSimulatorNode(AbstractSimulatorNode):

    def __init__(self, node: Node, topic_name: str):
        super().__init__(node, topic_name)

    def create_simulator(self, world_config, world_config_basedir, verbose):
        wifi_ = world_config['wifi'] if 'wifi' in world_config else None
        if wifi_ is None:
            node_name = self._node.get_name()
            self._logger.error("Failed to run " + node_name + ": key 'wifi' is not defined in wireless_config_file.")
            return None

        wifi_file = wifi_['samples_file'] if 'samples_file' in wifi_ else None
        if wifi_ is None:
            node_name = self._node.get_name()
            self._logger.error("Failed to run " + node_name +
                               ": key 'wifi/samples_file' is not defined in wireless_config_file.")
            return None

        wifi_file = os.path.join(world_config_basedir, wifi_file)
        with open(wifi_file) as f:
            samples = json.load(f)

        samples = extract_samples(samples, key="WiFi")
        return SampleSimulator(samples)

    def simulate_message(self, timestamp, x, y, z, floor):
        beacons = self._simulator.simulate(x, y, z, floor)

        json_obj = {
            "type": "rssi",
            "phoneID": "gazebo_sample_simulator",
            "timestamp": timestamp.nanoseconds/1e9,
            "data": beacons
        }

        string_msg = String()
        string_msg.data = json.dumps(json_obj)
        return string_msg


def main():
    rclpy.init()
    node = Node('wireless_sample_simulator')
    _ = SimpleSampleSimulatorNode(node, "wifi")

    rclpy.spin(node)


if __name__ == "__main__":
    main()
