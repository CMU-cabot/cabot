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

import roslaunch
import rospy

from cabot_explore_msgs.srv import RestartExploreMapping


class CartographerExplore:

    def __init__(self, configuration_directory, configuration_basename, points2_topic, imu_topic):
        self._configuration_directory = configuration_directory
        self._configuration_basename = configuration_basename
        self._points2_topic = points2_topic
        self._imu_topic = imu_topic

        self._mapping_process = None

        self._launch = roslaunch.scriptapi.ROSLaunch()
        self._launch.start()

        self._restart_mapping()

    def _restart_mapping(self):
        if self._mapping_process is not None:
            self._mapping_process.stop()

        node = roslaunch.core.Node("cartographer_ros", "cartographer_node", name="cartographer_node",
                            remap_args = [("points2", self._points2_topic), ("imu", self._imu_topic)],
                            output = "log")
        node.args = "-configuration_directory " + self._configuration_directory + " -configuration_basename " + self._configuration_basename
        self._mapping_process = self._launch.launch(node)


    def restart_mapping_callback(self, data):
        success = True
        try:
            self._restart_mapping()
        except:
            success = False
        return success


def main():
    rospy.init_node('cartographer_explore_node')

    configuration_directory = rospy.get_param("~configuration_directory")
    configuration_basename = rospy.get_param("~configuration_basename")
    points2_topic = rospy.get_param("~points2_topic")
    imu_topic = rospy.get_param("~imu_topic")

    cartographer_explore_node = CartographerExplore(configuration_directory, configuration_basename, points2_topic, imu_topic)

    restart_explore_mapping_service = rospy.Service("restart_explore_mapping", RestartExploreMapping, cartographer_explore_node.restart_mapping_callback)

    rospy.spin()


if __name__ == "__main__":
    main()
