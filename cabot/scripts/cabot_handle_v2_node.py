#!/usr/bin/env python3

# Copyright (c) 2020, 2022  Carnegie Mellon University
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
#
# memo: want to separate hardware spec and UI
#

import rclpy
import rclpy.node
import std_msgs.msg
import cabot.event

from cabot.handle_v2 import Handle
from rclpy.qos import QoSProfile, DurabilityPolicy


def notification_callback(msg):
    node.get_logger().info(msg)
    handle.execute_stimulus(msg.data)


def event_listener(msg):
    node.get_logger().info(msg)
    event = None
    if "button" in msg:
        event = cabot.event.ButtonEvent(**msg)

        # button down confirmation
        if not event.up:
            handle.execute_stimulus(Handle.BUTTON_CLICK)
    if "buttons" in msg:
        event = cabot.event.ClickEvent(**msg)

    if event is not None:
        node.get_logger().info(event)
        msg = std_msgs.msg.String()
        msg.data = str(event)
        event_pub.publish(msg)


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.node.Node("cabot_handle_v2_node")
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    event_pub = node.create_publisher(std_msgs.msg.String, '/cabot/event', qos)
    button_keys = node.declare_parameter("buttons", ['']).value
    handle = Handle(node, event_listener=event_listener, button_keys=button_keys)

    node.get_logger().info("buttons: {}".format(button_keys))

    no_vibration = node.declare_parameter("no_vibration", False).value
    node.get_logger().info("no_vibration = {}".format(no_vibration))

    if not no_vibration:
        node.create_subscription(std_msgs.msg.Int8, "/cabot/notification", notification_callback, 10)

    rclpy.spin(node)
