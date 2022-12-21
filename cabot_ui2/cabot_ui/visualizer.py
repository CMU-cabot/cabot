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

from visualization_msgs.msg import Marker, MarkerArray
import nav2_msgs.msg
from geometry_msgs.msg import PoseStamped

from cabot_ui import geojson

__instance = None


def instance(node):
    global __instance
    if __instance is not None:
        return __instance
    if node is None:
        raise RuntimeError("need to pass a node to instanciate")
    __instance = Visualizer(node)
    return __instance


class Visualizer(object):
    def __init__(self, node):
        self._node = node
        self.reset()
        self.global_map_name = "map"

        self.poi_pub = node.create_publisher(MarkerArray, "/cabot/poi", 10)

    def visualize(self):
        self._visualize_pois()
        Visualizer._count = 0

    def reset(self):
        self.pois = []
        self.turns = []
        self.spoken = []
        self.goal = None

    _count = 0

    def _visualize_pois(self):

        def make_marker(poi, **kwargs):
            return make_marker2(ns=type(poi).__name__, pose=poi.local_pose.to_pose_msg(), **kwargs)

        def make_marker2(ns=None, pose=None,
                         a=0.8, r=0, g=0, b=0,
                         x=0.2, y=0.2, z=0.2,
                         s=1.0, pz=1.0,
                         frame_id=None, _type=Marker.CUBE, text=None):
            if ns is None or pose is None:
                raise RuntimeError(F"ns and pose should be specified, {ns}, {pose}")
            if isinstance(pose, PoseStamped):
                frame_id = pose.header.frame_id
                pose = pose.pose
            if frame_id is None:
                frame_id = self.global_map_name

            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = self._node.get_clock().now().to_msg()
            marker.action = Marker.ADD
            if text is not None:
                marker.text = text
            marker.type = _type
            marker.ns = ns
            marker.id = Visualizer._count
            Visualizer._count += 1
            marker.pose = pose
            marker.pose.position.z = pz
            marker.scale.x = x*s
            marker.scale.y = y*s
            marker.scale.z = z*s
            marker.color.a = a
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            return marker

        array = MarkerArray()
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        array.markers.append(clear_marker)

        if self.pois is not None:
            for poi in self.pois:
                if isinstance(poi, geojson.DoorPOI):
                    array.markers.append(make_marker(poi, g=1))
                elif isinstance(poi, geojson.InfoPOI):
                    array.markers.append(make_marker(poi, b=1, x=1.0, s=0.5, _type=Marker.ARROW))
                    array.markers.append(make_marker(poi, text=poi.approached_statement(),
                                                     _type=Marker.TEXT_VIEW_FACING))
                elif isinstance(poi, geojson.SpeedPOI):
                    array.markers.append(make_marker(poi, r=1, x=1.0, s=0.5, _type=Marker.ARROW))
                    array.markers.append(make_marker(poi, text="%.2f m/s" % (poi.limit),
                                                     _type=Marker.TEXT_VIEW_FACING))
                else:
                    array.markers.append(make_marker(poi))

        if self.turns is not None:
            for turn in self.turns:
                array.markers.append(make_marker2(ns="turn", pose=turn.pose,
                                                  r=0, g=1, b=0, _type=Marker.SPHERE, s=0.5))
                if turn.end is not None:
                    array.markers.append(make_marker2(ns="turn", pose=turn.end,
                                                      r=0, g=1, b=0, _type=Marker.SPHERE, s=0.5))
                array.markers.append(make_marker2(ns="turn", pose=turn.pose, text=turn.text,
                                                  _type=Marker.TEXT_VIEW_FACING))

        for entry in self.spoken:
            posemsg, text, ns = entry
            posemsg.position.z = 0  # for visualization
            pz = 1.0
            if ns == "vib":
                pz = 0.75
            array.markers.append(make_marker2(ns=ns, pose=posemsg, pz=pz,
                                              r=1, g=1, b=1, _type=Marker.CYLINDER))
            array.markers.append(make_marker2(ns=ns, pose=posemsg, text=text, pz=pz,
                                              _type=Marker.TEXT_VIEW_FACING))

        if self.goal is not None:
            if isinstance(self.goal, nav2_msgs.msg.NavigateToPose.Goal):
                array.markers.append(make_marker2(
                    ns="goal", pose=self.goal.pose.pose, pz=0, s=1.0,
                    frame_id=self.goal.pose.header.frame_id, b=1, _type=Marker.SPHERE))

        # print array
        self._node.get_logger().debug(F"publish {len(array.markers)} markers")
        self.poi_pub.publish(array)
