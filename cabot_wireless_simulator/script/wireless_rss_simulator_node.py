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

from wireless_rss_simulator import *

def convert_beacons_to_string_msg(beacons, time, machine_name="gazebo_simulator"):
    string = String()

    json_object = {
        "type": "rssi",
        "phoneID": machine_name,
        "timestamp": time
    }

    beacon_list = []
    for b in beacons:
        beacon_list.append({
                        "type": "iBeacon",
                        "id": str(b._uuid) + "-" + str(b._major) + "-" + str(b._minor),
                        "rssi": b._rss,
                        "timestamp": time
                        }
                    )

    json_object["data"] = beacon_list
    string.data = json.dumps(json_object)
    return string

class SimpleBLESimulatorNode:

    def __init__(self, model_name, robot_name, floor_list, simulator, verbose=False):

        self._model_name = model_name
        self._robot_name = robot_name

        self._simulator = simulator
        self._cutoff = -100
        self._verbose = verbose

        floors = []
        heights = []
        for floor_height in floor_list:
            floor = floor_height["floor"]
            height = floor_height["height"]
            floors.append(floor)
            heights.append(height)

        self._floors = floors
        self._heights = heights

        self._beacons_pub = None

        self._previous_pub_time = None
        self._frequency = 1.0 # Hz

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
                    if i+1==len(floors)-1:
                        break
                else:
                    floor = floor_l + (floor_u - floor_l)/(height_u - height_l) * (z_r2m - height_l)
                    if round(floor) == floor_l:
                        z_floor = z_r2m - height_l
                    else:
                        z_floor = z_r2m - height_u

                    break
            floor = round(floor) # float -> int

        elif len(floors) == 1:
            floor = floors[0]
            height = heights[0]
            z_floor = z_r2m - height
        else: # len(floors) == 0
            raise RuntimeError("floors are not defined.")

        return x_r2m, y_r2m, z_floor, floor

    def simulate_message(self, timestamp, x, y, z, floor):
        # simulate beacons
        beacons = self._simulator.simulate(x, y, z, floor, self._cutoff)

        # convert beacons to String
        string_msg = convert_beacons_to_string_msg(beacons, time=timestamp)
        return string_msg

    def model_callback(self, message):

        now = rospy.get_time()

        # keep message publish frequency
        if self._previous_pub_time is None:
            self._previous_pub_time = rospy.get_time() # ms
        elif now < self._previous_pub_time + self._frequency:
            return
        self._previous_pub_time = now

        # get robot pose from the model_states message
        x_r2m, y_r2m, z_floor, floor = self.get_robot_pose(message)

        if self._verbose:
            print("x_r2m, y_r2m, z_floor, floor=",x_r2m, y_r2m, z_floor, floor)

        # simulate beacons
        if self._simulator is None:
            return
        msg = self.simulate_message(now, x_r2m, y_r2m, z_floor, floor)

        # publish message
        self._beacons_pub.publish(msg)

def main():
    rospy.init_node('wireless_rss_simulator')

    floor_list = rospy.get_param("~floor_list")

    model_name = rospy.get_param("~model/name")
    robot_name = rospy.get_param("~robot/name")

    world_config_file = rospy.get_param("~world_config_file")

    # check parameter existence
    beacons_ = rospy.get_param("~beacons", None)
    if beacons_ is None:
        node_name = rospy.get_name()
        rospy.logerr("Failed to run "+node_name+": key 'beacons' is not defined in world_config_file.")
        return

    beacons_file = rospy.get_param("~beacons/file")
    beacon_params = rospy.get_param("~beacons/parameters")
    verbose = rospy.get_param("~verbose", False)

    blebeacons_file = os.path.join( os.path.dirname(world_config_file), beacons_file )
    blebeacons = read_csv_blebeacons(blebeacons_file)
    ble_simulator = SimpleBLESimulator(blebeacons, parameters = beacon_params, verbose=verbose)

    simulator_node = SimpleBLESimulatorNode(model_name, robot_name, floor_list, ble_simulator, verbose=verbose)

    simulator_node._beacons_pub = rospy.Publisher("beacons", String, queue_size=10)

    model_sub = rospy.Subscriber("gazebo/model_states", ModelStates, simulator_node.model_callback)

    rospy.spin()

if __name__ == "__main__":
    main()
