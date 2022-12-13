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

import math

from geometry_msgs.msg import Pose
import numpy as np
from tf_transformations import euler_from_quaternion


def calc_navigate_pose_list(queue_path_pose_array, dist_interval_queue_navigate_path):
    """
    calculate robot navigation pose list from key pose list
    Args:
        queue_path_pose_array: array of geometry_msgs.msg.Pose representing key poses (pose at each turn) for navigation
        dist_interval_queue_navigate_path: interval to calculate navigation poses (e.g. 0.1m)
    Returns:
        queue_path_pose_array: array of geometry_msgs.msg.Pose for navigating robot along inputted key poess
    """
    navigate_pose_list = []
    for idx, pose in enumerate(queue_path_pose_array):
        pose_position = np.array([pose.position.x, pose.position.y])

        # add start pose of path segment as navigation path
        navigate_pose_list.append(pose)
        
        if idx<len(queue_path_pose_array)-1:
            pose_orientation_euler = euler_from_quaternion((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
            next_pose = queue_path_pose_array[idx+1]
            
            next_pose_position = np.array([next_pose.position.x, next_pose.position.y])
            dist_pose = np.linalg.norm(pose_position-next_pose_position)
            num_interpolate = int(math.floor(dist_pose/dist_interval_queue_navigate_path))

            for idx_interp in range(1, num_interpolate):
                # add interporated poses in path segment as navigation path
                interp_pose = Pose()
                interp_pose.position.x = pose.position.x + idx_interp*dist_interval_queue_navigate_path*math.cos(pose_orientation_euler[2])
                interp_pose.position.y = pose.position.y + idx_interp*dist_interval_queue_navigate_path*math.sin(pose_orientation_euler[2])
                interp_pose.position.z = pose.position.z
                interp_pose.orientation = pose.orientation
                navigate_pose_list.append(interp_pose)

            # add end pose of path segment as navigation path
            interp_pose = Pose()
            interp_pose.position = next_pose.position
            interp_pose.orientation = pose.orientation
            navigate_pose_list.append(interp_pose)
    return navigate_pose_list
