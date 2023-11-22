#!/usr/bin/env python3

# Copyright (c) 2023  Carnegie Mellon University, IBM Corporation, and others
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

import rospy

from route_explore_msgs.srv import AddLandmark, FindClosestLandmarkId, ResetCartographerLandmarkManager, SearchPath
from route_explore_utils.cartographer_landmark_manager import CartographerLandmarkManager


def main():
    rospy.init_node('cartographer_landmark_manager_node')

    costmap_topic = rospy.get_param("~costmap_topic", "/cabot_explore/local_map")
    lethal_cost_threshold = rospy.get_param("~lethal_cost_threshold", 65)
    landmark_loop_detect_distance = rospy.get_param("~landmark_loop_detect_distance", 10.0)

    cartographer_landmark_manager = CartographerLandmarkManager(costmap_topic, '/submap_list', lethal_cost_threshold, landmark_loop_detect_distance)

    add_landmark_service = rospy.Service("cartographer_landmark_manager/add_landmark", AddLandmark, cartographer_landmark_manager.add_landmark_callback)
    find_closest_landmark_id_service = rospy.Service("cartographer_landmark_manager/find_closest_landmark_id", FindClosestLandmarkId, cartographer_landmark_manager.find_closest_landmark_id_callback)
    reset_cartographer_landmark_manager_service = rospy.Service("cartographer_landmark_manager/reset_cartographer_landmark_manager", ResetCartographerLandmarkManager, cartographer_landmark_manager.reset_callback)
    search_path_service = rospy.Service("cartographer_landmark_manager/search_path", SearchPath, cartographer_landmark_manager.search_path_callback)

    rospy.spin()


if __name__ == "__main__":
    main()
