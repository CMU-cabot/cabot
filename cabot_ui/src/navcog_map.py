#!/usr/bin/env python

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

import rospy
import tf2_ros
from cabot_ui import geoutil, geojson, datautil
from visualization_msgs.msg import MarkerArray, Marker, InteractiveMarkerControl, InteractiveMarker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from geometry_msgs.msg import Point
import std_msgs.msg

data_ready = False
navigate_menu = None
raw_current_floor = 0
meters_per_floor = 5
current_floor = 0
last_floor = None
menu_handler = MenuHandler()
server = None
map_frame = "map_global"

def visualize_features(features, node_map):
    vis_pub = rospy.Publisher("links", MarkerArray, queue_size=1, latch=True);

    array = MarkerArray()

    marker = Marker()
    marker.header.frame_id = map_frame
    marker.header.stamp = rospy.get_rostime()
    marker.ns = "links"
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.05
    marker.color.a = 0.5
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    for f in features:
        if not isinstance(f, geojson.Link) or f.floor != current_floor:
            continue
        s = Point()
        s.x = f.start_node.local_geometry.x
        s.y = f.start_node.local_geometry.y
        s.z = raw_current_floor*meters_per_floor + 0.1
        e = Point()
        e.x = f.end_node.local_geometry.x
        e.y = f.end_node.local_geometry.y
        e.z = raw_current_floor*meters_per_floor + 0.1
        marker.points.append(s)
        marker.points.append(e)

    array.markers.append(marker)

    for k, f in node_map.items():
        if isinstance(f, geojson.Landmark):
            continue
        if f.floor != current_floor:
            continue

        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True

        marker = InteractiveMarker()
        marker.header.frame_id = map_frame
        marker.name = k
        marker.pose.position.x = f.local_geometry.x
        marker.pose.position.y = f.local_geometry.y
        marker.pose.position.z = raw_current_floor*meters_per_floor + 0.1
        marker.scale = 1

        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = 0.2
        sphere.scale.y = 0.2
        sphere.scale.z = 0.2
        sphere.color.a = 1.0
        sphere.color.r = 0.0
        sphere.color.g = 0.0
        sphere.color.b = 1.0

        control.markers.append(sphere)
        marker.controls.append(control)
        server.insert(marker, process_feedback)
        menu_handler.apply(server, marker.name)

    vis_pub.publish( array )
    vis_pub.publish( array )

def process_feedback(feedback):
    print(feedback)

def menu_callback(feedback):
    print(feedback)
    msg = std_msgs.msg.String()
    msg.data = "navigation;destination;"+feedback.marker_name
    event_pub.publish(msg)

def cf_callback(msg):
    global current_floor, raw_current_floor
    raw_current_floor = msg.data
    current_floor = msg.data + 1 if msg.data >= 0 else msg.data
    check_update()

def check_update():
    global last_floor
    if not data_ready:
        return
    if "map" != map_frame and \
       not tf2_buffer.can_transform("map", map_frame, rospy.Time.now(), rospy.Duration(1)):
        return
    if current_floor != last_floor:
        visualize_features(datautil.getInstance().features, datautil.getInstance().node_map)
        server.applyChanges()
    last_floor = current_floor

def initMenu():
    global navigate_menu
    navigate_menu = menu_handler.insert("Navigate to Here", callback=menu_callback)

if __name__ == "__main__":
    rospy.init_node("navcog_map")

    tf2_buffer = tf2_ros.Buffer()
    tf2_listener = tf2_ros.TransformListener(tf2_buffer)
    event_pub = rospy.Publisher("/cabot/event", std_msgs.msg.String, queue_size=10)
    cf_sub = rospy.Subscriber("/current_floor", std_msgs.msg.Int64, cf_callback)

    if rospy.has_param("~initial_floor"):
        current_floor = rospy.get_param("~initial_floor")

    if rospy.has_param("~map_frame"):
        map_frame = rospy.get_param("~map_frame")

    anchor = None
    if rospy.has_param("~anchor_file"):
        anchor_file = rospy.get_param("~anchor_file")
    rospy.loginfo("Anchor file is %s", anchor_file)
    if anchor_file is not None:
        temp = geoutil.get_anchor(yaml=anchor_file)
        if temp is not None:
            anchor = temp
        else:
            rospy.logwarn("could not load anchor_file \"%s\"", anchor_file)

    rospy.loginfo(anchor)
    datautil.getInstance().set_anchor(anchor)
    datautil.getInstance().init_by_server()
    datautil.getInstance().set_anchor(anchor)
    data_ready = True
    rospy.loginfo("data ready")
    rospy.loginfo("initial floor = %d"%(current_floor))

    server = InteractiveMarkerServer("menu")
    initMenu()

    while not rospy.is_shutdown():
        check_update()
        rospy.sleep(1)

    rospy.spin()
