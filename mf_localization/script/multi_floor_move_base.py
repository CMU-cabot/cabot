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

import rclpy
from rclpy.parameter import Parameter
import roslaunch
from std_msgs.msg import String


class MultiFloorMoveBaseManager:
    def __init__(self):
        self.frame_id_list = []
        self.current_frame_changed = False
        self.current_frame = ""

    def current_frame_callback(self, message):
        frame_id = message.data

        if frame_id not in self.frame_id_list:
            self.logger.info("unknown frame [" + frame_id + "] was passed.")
            return

        if self.current_frame != frame_id:
            self.current_frame = frame_id
            self.current_frame_changed = True


def main():
    rclpy.init()
    node = rclpy.create_node('multi_floor_move_base')

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    local_map_frame = node.declare_parameter("local_map_frame", "map").value
    global_map_frame = node.declare_parameter("global_map_frame", "map").value

    mf_move_base = MultiFloorMoveBaseManager()

    # resolve topic remapping
    cmd_vel_topic = node.resolve_topic_name("cmd_vel")
    odom_topic = node.resolve_topic_name("odom")

    map_list = node.declare_parameter("map_list").value
    for i, map_dict in enumerate(map_list):
        frame_id = map_dict["frame_id"]
        mf_move_base.frame_id_list.append(frame_id)

    sub = node.create_subscription(String, "current_frame", mf_move_base.current_frame_callback)

    # ros spin
    r = node.create_rate(10)  # 10 Hz
    while rclpy.ok():
        if mf_move_base.current_frame_changed:

            if launch.started:
                launch.stop()

            # roslaunch should be used in the main thread
            launch = roslaunch.scriptapi.ROSLaunch()
            launch.start()

            map_name = mf_move_base.current_frame

            if local_map_frame == global_map_frame:
                # if local_map_frame is fixed to global_map_frame, it is necessary to set frame_id for global cost map
                node.set_parameters(Parameter("/move_base/global_costmap/global_frame", value=map_name))
                # node.set_parameters(Parameter("/move_base/global_costmap/static_layer/map_topic", value=map_name))  # it is not necessary to update map_topic because it is published by multi-floor map_server

            package = "move_base"
            executable = "move_base"
            # todo
            node = roslaunch.core.Node(package, executable,
                                name="move_base",
                                namespace = "/",
                                remap_args = [("cmd_vel", cmd_vel_topic), ("odom", odom_topic)],
                                output = "screen"
                                )
            script = launch.launch(node)

            mf_move_base.current_frame_changed = False
        r.sleep()


if __name__ == "__main__":
    main()
