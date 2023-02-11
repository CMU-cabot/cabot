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

import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from wireless_abstract_simulator_node import AbstractSimulatorNode
from wireless_rss_simulator import read_csv_blebeacons, SimpleBLESimulator


class SimpleBLESimulatorNode(AbstractSimulatorNode):

    def __init__(self, node: Node, topic_name: str):
        super().__init__(node, topic_name)

    def create_simulator(self, world_config, world_config_basedir, verbose):
        # check parameter existence
        beacons_ = world_config['beacons'] if 'beacons' in world_config else None
        if beacons_ is None:
            node_name = self._node.get_name()
            self._logger.info("Failed to run " + node_name + ": key 'beacons' is not defined in wireless_config_file.")
            return None

        beacons_file = beacons_['file'] if 'file' in beacons_ else None
        beacon_params = beacons_['parameters'] if 'parameters' in beacons_ else None

        if beacons_file is None or beacon_params is None:
            node_name = self._node.get_name()
            self._logger.info("Failed to run " + node_name +
                              ": key 'beacons/file' or 'beacons/parameters' not defined in wireless_config_file.")
            return None

        blebeacons_file = os.path.join(world_config_basedir, beacons_file)
        blebeacons = read_csv_blebeacons(blebeacons_file)
        return SimpleBLESimulator(blebeacons, parameters=beacon_params, verbose=verbose)

    def simulate_message(self, timestamp, x, y, z, floor):
        # simulate beacons
        beacons = self._simulator.simulate(x, y, z, floor, self._cutoff)

        # convert beacons to String
        msg = String()

        json_object = {
            "type": "rssi",
            "phoneID": "gazebo_simulator",
            "timestamp": timestamp.nanoseconds/1e9
        }

        beacon_list = []
        for b in beacons:
            beacon_list.append({
                            "type": "iBeacon",
                            "id": str(b._uuid) + "-" + str(b._major) + "-" + str(b._minor),
                            "rssi": b._rss,
                            "timestamp": timestamp.nanoseconds/1e9
                            }
                        )

        json_object["data"] = beacon_list
        msg.data = json.dumps(json_object)
        return msg


def main():
    rclpy.init()
    node = rclpy.node.Node('wireless_rss_simulator')
    _ = SimpleBLESimulatorNode(node, "beacons")

    rclpy.spin(node)


if __name__ == "__main__":
    main()
