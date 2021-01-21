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
from cabot_ui import geoutil, geojson, datautil
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point


def visualize_features(features, node_map):
    vis_pub = rospy.Publisher("links", MarkerArray, queue_size=1, latch=True);
    
    array = MarkerArray()

    marker = Marker()
    marker.header.frame_id = "map"
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
        if not isinstance(f, geojson.Link) or f.floor != 4:
            continue
        s = Point()
        s.x = f.start_node.local_geometry.x
        s.y = f.start_node.local_geometry.y
        s.z = 0.1
        e = Point()
        e.x = f.end_node.local_geometry.x
        e.y = f.end_node.local_geometry.y
        e.z = 0.1
        marker.points.append(s)
        marker.points.append(e)

    array.markers.append(marker)

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.get_rostime()
    marker.ns = "nodes"
    marker.id = 1
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    
    for k, f in node_map.items():
        if not isinstance(f, geojson.Node) or f.floor != 4:
            continue
        s = Point()
        s.x = f.local_geometry.x
        s.y = f.local_geometry.y
        s.z = 0.1
        marker.points.append(s)

    array.markers.append(marker)
    
    vis_pub.publish( array )
    vis_pub.publish( array )

if __name__ == "__main__":
    rospy.init_node("navcog_map")

    datautil.getInstance().init_by_server()
    anchor = geoutil.Anchor(lat = 40.443259, lng = -79.945874, rotate=15.1)
    datautil.getInstance().set_anchor(anchor)
    rospy.loginfo("Data is ready")
    
    visualize_features(datautil.getInstance().features, datautil.getInstance().node_map)
    #rospy.loginfo(datautil.features)
    rospy.spin()
