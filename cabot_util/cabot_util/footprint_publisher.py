# Copyright (c) 2020  Carnegie Mellon University
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

import enum
import math

import rclpy
import rclpy.timer
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import SetParametersResult
import tf2_ros
from diagnostic_updater import Updater, DiagnosticTask
from diagnostic_msgs.msg import DiagnosticStatus


class Mode(enum.IntEnum):
    NORMAL = 0
    SMALLEST = 1
    DYNAMIC = 2
    SMALL = 3

def check_status(stat):
    g_node.get_logger().info("check_status")

    if not current_mode:
        stat.summary(DiagnosticStatus.ERROR, "Mode is not specified")
        return stat
    if current_mode < 0 or 3 < current_mode:
        stat.summary(DiagnosticStatus.ERROR, "Unknown mode = {}".format(current_mode))
        return stat
    if not footprint:
        stat.summary(DiagnosticStatus.WARN, "No footprint (mode={})".format(current_mode))
        return stat
    if len(transforms) == 0:
        stat.summary(DiagnosticStatus.WARN, "No transform is specified (mode={})".format(current_mode))
        return stat
    stat.summary(DiagnosticStatus.OK, "working (mode={})".format(current_mode))
    return stat

def main(args=None):
    global g_node, current_mode, publishers, broadcaster
    
    rclpy.init(args=args)

    g_node = rclpy.create_node('footprint_publisher')

    g_node.declare_parameter('footprint_mode', Mode.NORMAL.value)
    g_node.declare_parameter('footprint_topics', ['/global_costmap/footprint', '/local_costmap/footprint'])
    g_node.declare_parameter('footprint_normal', 0.45)
    g_node.declare_parameter('footprint_smallest', 0.35)
    g_node.declare_parameter('footprint_small', 0.40)
    g_node.declare_parameter('footprint_links', ["base_footprint"])
    g_node.declare_parameter('offset_links', ["base_control_shift"])
    g_node.declare_parameter('offset_normal', [0.0, 0.25, 0.038])
    g_node.declare_parameter('offset_smallest', [0.0, 0.15, 0.038])
    g_node.declare_parameter('offset_small', [0.0, 0.20, 0.038])

    footprint_topics = g_node.get_parameter("footprint_topics").value

    g_node.add_on_set_parameters_callback(param_callback)

    publishers = []
    for topic in footprint_topics:
        publisher = g_node.create_publisher(Polygon, topic,  10)
        publishers.append(publisher)

    broadcaster = tf2_ros.TransformBroadcaster(g_node)

    timer = g_node.create_timer(0.02, timer_callback)

    updater = Updater(g_node)
    updater.add("ROS2 Footprint Publisher", check_status)

    try:
        rclpy.spin(g_node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    g_node.destroy_node()
    rclpy.shutdown()

def get_footprint(mode):
    footprint = None
    if mode == Mode.NORMAL:
        footprint = g_node.get_parameter("footprint_normal").value
    if mode == Mode.SMALLEST:
        footprint = g_node.get_parameter("footprint_smallest").value
    if mode == Mode.SMALL:
        footprint = g_node.get_parameter("footprint_small").value

    if footprint is None:
        return None

    if isinstance(footprint, (int, float)):
        return circle_footprint(footprint)
    elif isinstance(footprint, list):
        return polygon_footprint(footprint)

    g_node.get_logger().error("points should be a number or a list")
    return None

def circle_footprint(radius):
    N = 16
    polygon = Polygon()
    for w in range(0, N):
        rad = 2 * math.pi * w / N
        p = Point32()
        p.x = math.cos(rad) * radius
        p.y = math.sin(rad) * radius
        polygon.points.append(p)
    return polygon

def polygon_footprint(points):
    polygon = Polygon()
    for i in range(0, len(points)/2):
        p = Point32()
        p.x = points[i*2]
        p.y = points[i*2+1]
        polygon.points.append(p)
    return polygon

def get_offset_transforms(mode):
    res = []

    for f, o in zip(g_node.get_parameter("footprint_links").value, g_node.get_parameter("offset_links").value):
        t = TransformStamped()
        offset = [0.0, 0.0, 0.0]
        if mode == Mode.NORMAL:
            offset = g_node.get_parameter("offset_normal").value
        if mode == Mode.SMALLEST:
            offset = g_node.get_parameter("offset_smallest").value
        if mode == Mode.SMALL:
            offset = g_node.get_parameter("offset_small").value
            
        t.header.stamp = g_node.get_clock().now().to_msg()
        t.header.frame_id = f
        t.child_frame_id = o
        t.transform.translation.x = offset[0]
        t.transform.translation.y = offset[1]
        t.transform.translation.z = offset[2]
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        res.append(t)

    return res

current_mode = None
footprint = None
transforms = []

def timer_callback():
    global current_mode, footprint, transforms
    new_mode = g_node.get_parameter("footprint_mode").value

    if current_mode != new_mode:
        current_mode = new_mode
        footprint = get_footprint(current_mode)
        if footprint is None:
            return
        
        #g_node.get_logger().info(str(footprint))
        # publish footprint
        for pub in publishers:
            pub.publish(footprint)

    transforms = get_offset_transforms(new_mode)
    broadcaster.sendTransform(transforms)
    #g_node.get_logger().info(str(transform))

def param_callback(params):
    for param in params:
        if param.value in map(lambda x:x.value, list(Mode)):
            return SetParametersResult(successful=True)
    return SetParametersResult(successful=False)
    

    
if __name__ == '__main__':
    main()


