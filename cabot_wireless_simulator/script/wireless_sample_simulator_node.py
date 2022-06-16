#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2022  IBM Corporation
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
import argparse
import ast
import os

import numpy as np

import rospy
import tf2_ros
import tf_conversions
from std_msgs.msg import String
from geometry_msgs.msg import Point, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelStates, LinkStates

from wireless_utils import extract_samples
from wireless_rss_simulator import *
from wireless_rss_simulator_node import *
from wireless_sample_simulator import *

class SimpleSampleSimulatorNode(SimpleBLESimulatorNode):

    def simulate_message(self, timestamp, x, y, z, floor):
        beacons = self._simulator.simulate(x, y, z, floor)

        json_obj = {
            "type": "rssi",
            "phoneID": "gazebo_sample_simulator",
            "timestamp": timestamp,
            "data": beacons
        }

        string_msg = String()
        string_msg.data = json.dumps(json_obj)
        return string_msg

def main():
    rospy.init_node('wireless_sample_simulator')

    floor_list = rospy.get_param("~floor_list")

    model_name = rospy.get_param("~model/name")
    robot_name = rospy.get_param("~robot/name")

    world_config_file = rospy.get_param("~world_config_file")

    # check parameter existence
    wifi_ = rospy.get_param("~wifi", None)
    if wifi_ is None:
        node_name = rospy.get_name()
        rospy.logerr("Failed to run "+node_name+": key 'wifi' is not defined in world_config_file.")
        return

    wifi_file = rospy.get_param("~wifi/samples_file")
    verbose = rospy.get_param("~verbose", False)

    wifi_file = os.path.join( os.path.dirname(world_config_file), wifi_file)
    with open(wifi_file) as f:
        samples = json.load(f)

    samples = extract_samples(samples, key="WiFi")
    wifi_simulator = SampleSimulator(samples)

    simulator_node = SimpleSampleSimulatorNode(model_name, robot_name, floor_list, wifi_simulator, verbose=verbose)

    simulator_node._beacons_pub = rospy.Publisher("wifi", String, queue_size=10)

    model_sub = rospy.Subscriber("gazebo/model_states", ModelStates, simulator_node.model_callback)

    rospy.spin()

if __name__ == "__main__":
    main()
