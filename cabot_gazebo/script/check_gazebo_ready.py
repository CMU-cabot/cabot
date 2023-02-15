#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2023  Carnegie Mellon University
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
from rclpy.node import Node
# import threading
import time
from gazebo_msgs.msg import ModelStates

model_callback_called = False


def model_callback(msg):
    node.get_logger().info("model_callback")
    global model_callback_called
    model_callback_called = True


if __name__ == "__main__":
    rclpy.init()
    node = Node('check_gazebo_ready')
    _ = node.create_subscription(ModelStates, "/gazebo/model_states", model_callback, 10)
    # thread = threading.Thread(target=check_model_callback, daemon=False)
    # thread.start()
    try:
        while model_callback_called:
            rclpy.spin_once(node)
            time.sleep(0.1)
    except:  # noqa: E722
        pass
    node.get_logger().info("Gazebo seems to be ready")
