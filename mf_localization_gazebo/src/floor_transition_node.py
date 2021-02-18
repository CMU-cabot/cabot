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

import rospy

from mf_localization_msgs.srv import FloorChange, FloorChangeResponse
from mf_localization_msgs.msg import StatusResponse
from gazebo_msgs.msg import ModelStates, LinkStates, ModelState
from gazebo_msgs.srv import SetModelState

class FloorTransition:
    def __init__(self, floor_list, robot_name, model_name):
        floors = {}
        for floor_height in floor_list:
            floor = floor_height["floor"]
            height = floor_height["height"]
            floors[floor] = height

        self._floors = floors
        rospy.loginfo(self._floors)

        self._prev_floor_check = rospy.get_time()

        self._robot_name = robot_name
        self._model_name = model_name
        
        self.service = rospy.Service("/floor_change", FloorChange, self.handle_floor_change)
        self.model_sub = rospy.Subscriber("gazebo/model_states", ModelStates, self.model_callback)
        self.gazeb_service = rospy.ServiceProxy("gazebo/set_model_state", SetModelState)
        

    def model_callback(self, message):
        if rospy.get_time() - self._prev_floor_check < 0.5:
            return
        self._prev_floor_check = rospy.get_time()
        
        # get the position of the robot relative to the model
        mesh_name = self._model_name
        robot_name = self._robot_name

        names = message.name
        poses = message.pose

        pose_dict = {}
        for i, name in enumerate(names):
            pose_dict[name] = poses[i]

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

        rospy.loginfo(self.current_floor)

    def handle_floor_change(self, req):
        next_floor = int(self.current_floor + req.diff.data)
        while not next_floor in self._floors:
            next_floor = int(next_floor + req.diff.data)
        
        resp = StatusResponse()
        if next_floor in self._floors:
            try:
                state = ModelState()
                state.model_name = self._robot_name
                state.pose = self.robot_pose
                state.pose.position.z = self._floors[next_floor]+0.01
                self.gazeb_service(state)
                resp.code = 0
                resp.message = "succeeded to change floor to {} @ {}".format(next_floor, self._floors[next_floor])
            except rospy.ServiceException, e:
                resp.code = 2
                resp.message = "could not changed floor to {} @  {}".format(next_floor, self._floors[next_floor])
                
            rospy.loginfo(resp.message)
        else:
            resp.code = 1
            resp.message = "could not find floor {}".format(next_floor)
            rospy.logerr(resp.message)

        return resp
        
        


if __name__ == "__main__":
    rospy.init_node("floor_transition_node")
    
    floor_list = rospy.get_param("~floor_list", [])
    model_name = rospy.get_param("~model/name")
    robot_name = rospy.get_param("~robot/name")
    
    node = FloorTransition(floor_list=floor_list, model_name=model_name, robot_name=robot_name)

    rospy.spin()
