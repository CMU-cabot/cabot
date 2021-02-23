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

"""
Geography Utility
Author: Daisuke Sato<daisukes@cmu.edu>
"""

import math
import numpy
import numpy.linalg
import rosparam

import tf
import geometry_msgs.msg
from pyproj import Proj, Geod, transform
from pyproj import Transformer                                                                                                                


from tf.transformations import quaternion_multiply, euler_from_quaternion


def msg_from_q(q):
    """get Quaternion message from array"""
    return geometry_msgs.msg.Quaternion(*q)

def msg_from_p(p):
    """get Point message from array"""
    return geometry_msgs.msg.Point(*p)

def msg_from_pq(p, q):
    """get Pose message from pose and quaternion array"""
    pose = geometry_msgs.msg.Pose()
    pose.position = msg_from_p(p)
    pose.orientation = msg_from_q(q)
    return pose

def q_from_msg(msg):
    """get array from Quaternion message"""
    if not isinstance(msg, geometry_msgs.msg.Quaternion):
        raise(RuntimeError("msg should be a Quaternion message"))
    return [msg.x, msg.y, msg.z, msg.w]

def p_from_msg(msg):
    """get array from Point message"""
    if isinstance(msg, Point):
        msg = msg.to_point_msg()
    if not isinstance(msg, geometry_msgs.msg.Point):
        raise(RuntimeError("msg should be a Point message"))
    return [msg.x, msg.y, msg.z]

def q_from_points(msg1, msg2):
    """get quaternion array from two points"""
    point1 = p_from_msg(msg1)
    point2 = p_from_msg(msg2)

    org = numpy.array([1, 0, 0]) # robot default orientation
    diff = numpy.subtract(point2, point1) # get difference
    diff = diff / numpy.linalg.norm(diff) # normalize

    cross = numpy.cross(org, diff)
    dot = numpy.dot(org, diff)
    org_len = numpy.linalg.norm(org)
    diff_len = numpy.linalg.norm(diff)

    return numpy.append(cross, [org_len * diff_len + dot])

def q_inverse(q):
    """get inverse quaternion"""
    return [q[0], q[1], q[2], -q[3]]

def q_diff(q1, q2):
    rot = [0, 0, 1, 0] # 180 degree turn
    return quaternion_multiply(quaternion_multiply(q2, rot), q_inverse(q1))
   

def in_angle(pose1, pose2, margin_in_degree):
    """
    pose1: robot pose
    pose2: POI pose
    margin_in_degree: POI angle margin in degree
    return True if robot orientation is in POI angle
    """
    if isinstance(pose1 , Pose):
        pose1 = pose1.to_pose_msg()
    if isinstance(pose2 , Pose):
        pose2 = pose2.to_pose_msg()

    margin = margin_in_degree / 180.0 * math.pi
    p1_p2 = q_from_points(pose1.position, pose2.position)

    quat1 = q_from_msg(pose1.orientation)
    quat2 = q_from_msg(pose2.orientation)

    # check orientation diff from pose2 orientation to rotated pose1 orientation
    _, _, yaw1 = euler_from_quaternion(q_diff(quat2, quat1))

    # check orientation diff from pose2 orientation to rotated p1->p2 orientation
    _, _, yaw2 = euler_from_quaternion(q_diff(quat2, p1_p2))

    # check both in the margin
    return abs(yaw1) <= margin and abs(yaw2) <= margin


class Point(object):
    """represent a 2D point"""
    def __init__(self, **kwargs):
        s = super(Point, self)
        if self.__class__.mro()[-2] == s.__thisclass__:
            s.__init__()
        else:
            s.__init__(**kwargs)

        if 'xy' in kwargs:
            self.x = kwargs['xy'].x
            self.y = kwargs['xy'].y
        else:
            self.x = kwargs['x']
            self.y = kwargs['y']

    def __repr__(self):
        return "(%.2f, %.2f)" % (self.x, self.y)

    def distance_to(self, obj):
        """euclid distance between the instance point and the passed point"""
        if isinstance(obj, Point):
            return math.sqrt((self.x - obj.x) * (self.x - obj.x)
                             + (self.y - obj.y) * (self.y - obj.y))
        raise RuntimeError("need to pass a Point object (%s)"%(type(obj)))

    def interpolate(self, obj, ratio):
        """get a point between the instance point and the passed point
        defined by the 'ratio'. 'ratio'==1 means the passed point itself"""
        return Point(x=self.x * (1.0 - ratio) + obj.x * ratio,
                     y=self.y * (1.0 - ratio) + obj.y * ratio)

    def to_point_msg(self):
        """convert Point into ROS point msg"""
        point = geometry_msgs.msg.Point()
        point.x = self.x
        point.y = self.y
        point.z = 0
        return point


class Pose(Point):
    """represent a 2D pose. init with x, y, and r"""
    def __init__(self, **kwargs):
        super(Pose, self).__init__(**kwargs)
        self.r = kwargs['r']

    def __repr__(self):
        return "(%.2f, %.2f)[%.2f deg]" % (self.x, self.y, self.r/math.pi*180)

    @staticmethod
    def pose_from_points(p1, p2):
        p1_p2 = q_from_points(p1, p2)
        _, _, yaw = euler_from_quaternion(q_diff([0,0,0,1], p1_p2))        
        return Pose(x=p1.x, y=p1.y, r=yaw)

    @property
    def orientation(self):
        orientation = geometry_msgs.msg.Quaternion()
        q = tf.transformations.quaternion_from_euler(0, 0, self.r)
        orientation.x = q[0]
        orientation.y = q[1]
        orientation.z = q[2]
        orientation.w = q[3]
        return orientation
    
    @classmethod
    def from_pose_msg(cls, msg):
        """instanciate Pose from ROS pose msg"""
        quat = msg.pose.orientation
        quat_arr = [quat.x, quat.y, quat.z, quat.w]
        euler = tf.transformations.euler_from_quaternion(quat_arr)
        return Pose(x=msg.pose.position.x, y=msg.pose.position.y, r=euler[2])

    def to_pose_msg(self):
        """convert Pose into ROS pose msg"""
        pose = geometry_msgs.msg.Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        q = tf.transformations.quaternion_from_euler(0, 0, self.r)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose


class Latlng(object):
    """represent a global coordinate. init with lat, and lng"""
    def __init__(self, **kwargs):
        s = super(Latlng, self)
        if self.__class__.mro()[-2] == s.__thisclass__:
            s.__init__()
        else:
            s.__init__(**kwargs)

        self.lat = kwargs['lat']
        self.lng = kwargs['lng']

    def __repr__(self):
        return "[%.7f, %.7f]" % (self.lat, self.lng)

    def distance_to(self, obj):
        """distance between the instance latlng and the passed latlng"""
        if isinstance(obj, Latlng):
            geo = Geod(ellps='WGS84')
            (_, _, dist) = geo.inv(self.lng, self.lat, obj.lng, obj.lat)
            return dist
        raise RuntimeError("need to pass a Latlng object (%s)"%(type(obj)))

class Anchor(Latlng):
    """represent an anchor point. init with lat, lng, and rotate"""
    def __init__(self, **kwargs):
        super(Anchor, self).__init__(**kwargs)
        self.rotate = kwargs['rotate']

    def __repr__(self):
        return "[%.7f, %.7f](%.2f)" % (self.lat, self.lng, self.rotate)

    
EPSG4326 = Proj(init='epsg:4326')
EPSG3857 = Proj(init='epsg:3857')
transformer4326_3857 = Transformer.from_proj(EPSG4326, EPSG3857)
transformer3857_4326 = Transformer.from_proj(EPSG3857, EPSG4326)

def latlng2mercator(latlng):
    """convert a LatLng point into a Mercator point"""
    (x, y) = transformer4326_3857.transform(latlng.lng, latlng.lat)
    return Point(x=x, y=y)


def mercator2latlng(mercator):
    """convert a Mercatro point into a LatLng point"""
    (lng, lat) = transformer3857_4326.transform(mercator.x, mercator.y)
    return Latlng(lat=lat, lng=lng)


def get_point_resolution(anchor):
    """get a resolution at an anchor point"""
    geo = Geod(ellps='WGS84')
    ll1 = mercator2latlng(anchor)
    ll2 = mercator2latlng(Point(x=anchor.x+1, y=anchor.y))
    (_, _, dist) = geo.inv(ll1.lng, ll1.lat, ll2.lng, ll2.lat)
    return dist


def mercator2xy(src_mercator, anchor):
    """convert a Mercator point into a local point in the anchor coordinate"""
    res = get_point_resolution(anchor)
    dx = (src_mercator.x - anchor.x) * res
    dy = (src_mercator.y - anchor.y) * res
    rad = anchor.rotate / 180.0 * math.pi
    c = math.cos(rad)
    s = math.sin(rad)
    x = dx * c - dy * s
    y = dx * s + dy * c
    return Point(x=x, y=y)


def xy2mercator(src_xy, anchor):
    """convert a local point in the anchor coordinate into a Mercator point"""
    r = get_point_resolution(anchor)
    x = src_xy.x
    y = src_xy.y
    rad = - anchor.rotate / 180.0 * math.pi
    c = math.cos(rad)
    s = math.sin(rad)
    dx = (x * c - y * s) / r
    dy = (x * s + y * c) / r
    return Point(x=anchor.x + dx, y=anchor.y + dy)


def global2local(latlng, anchor):
    """convert a global point into a local point in the anchor coordinate"""
    temp = latlng2mercator(anchor)
    anchor.x = temp.x
    anchor.y = temp.y
    mer = latlng2mercator(latlng)
    xy = mercator2xy(mer, anchor)
    return xy


def local2global(xy, anchor):
    """convert a local point in the anchor coordinate into the global point"""
    temp = latlng2mercator(anchor)
    anchor.x = temp.x
    anchor.y = temp.y
    mer = xy2mercator(xy, anchor)
    latlng = mercator2latlng(mer)
    return latlng


def get_anchor(yaml=None):
    """get anchor"""
    if yaml is None:
        return None
    data = rosparam.load_file(yaml)
    print data
    if not data:
        return None
    temp = data[0][0]["anchor"]
    lat = temp["latitude"]
    lng = temp["longitude"]
    rot = temp["rotate"]
    return Anchor(lat=lat, lng=lng, rotate=rot)
