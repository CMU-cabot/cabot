# Copyright (c) 2020 Carnegie Mellon University
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
from people_msgs.msg import People
from visualization_msgs.msg import MarkerArray, Marker
from rcl_interfaces.msg import SetParametersResult
import tf2_ros
import PyKDL

g_sub = None
g_pub = None

def main(args=None):
    global g_node, current_mode, publishers, broadcaster, g_pub, g_sub
    
    rclpy.init(args=args)

    g_node = rclpy.create_node('people_vis')

    g_node.declare_parameter('people_topic', "/people")
    g_node.declare_parameter('vis_topic', "/people_vis")

    people_topic = g_node.get_parameter("people_topic").value
    vis_topic = g_node.get_parameter("vis_topic").value

    
    g_sub = g_node.create_subscription(People, people_topic, people_callback, 10)
    g_pub = g_node.create_publisher(MarkerArray, vis_topic,  10)

    try:
        rclpy.spin(g_node)
    except KeyboardInterrupt:
        pass
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    g_node.destroy_node()
    rclpy.shutdown()

def people_callback(msg):
    array = MarkerArray()
    clear_marker = Marker()
    clear_marker.action = Marker.DELETEALL
    array.markers.append(clear_marker)
    for person in msg.people:
        array.markers.append(get_point(person, 0.5))
        array.markers.append(get_text(person))
        array.markers.append(get_arrow(person))

    g_pub.publish(array)

def init_marker(marker, person, type_, r=0.0, g=0.0, b=0.0, a=1.0):
    marker.header.stamp = g_node.get_clock().now().to_msg()
    marker.header.frame_id = "map"
    marker.id = int(person.name)
    marker.ns = "person_"+type_
    marker.action = Marker.MODIFY
    marker.pose.position.x = person.position.x
    marker.pose.position.y = person.position.y
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = a
    
def get_point(person, size):
    marker = Marker()
    init_marker(marker, person, "point", b=1.0)
    marker.type = Marker.SPHERE
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;
    return marker

def get_text(person):
    marker = Marker()
    init_marker(marker, person, "text", r=1.0, g=1.0, b=1.0)
    marker.type = Marker.TEXT_VIEW_FACING
    marker.pose.position.z = person.position.z + 0.5;
    marker.scale.z = 0.3;
    marker.text = person.name;
    return marker

def get_arrow(person):
    marker = Marker()
    init_marker(marker, person, "arrow")
    marker.type = Marker.ARROW
    v = math.sqrt(math.pow(person.velocity.x,2)+math.pow(person.velocity.y,2))
    y = math.atan2(person.velocity.y, person.velocity.x)
    q = PyKDL.Rotation.RPY(0, 0, y).GetQuaternion()
    print (v, y, q)
    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]
    marker.scale.x = v
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    return marker

                     
if __name__ == '__main__':
    main()


