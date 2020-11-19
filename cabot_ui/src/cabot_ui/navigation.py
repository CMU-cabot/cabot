#!/usr/bin/env python

# Copyright 2020 Carnegie Mellon University
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

import math
import numpy, numpy.linalg

# ROS
import rospy
import rosparam
import tf
import actionlib
import move_base_msgs.msg
import std_msgs.msg
import nav_msgs.msg
import geometry_msgs.msg
from actionlib_msgs.msg import GoalStatus
from obstacle_detector.msg import Obstacles

# Other

import cabot_msgs.srv
from cabot import util
from cabot.handle_v2 import Handle
from cabot_ui import visualizer, geoutil, geojson, datautil
from cabot_ui.turn_detector import TurnDetector
from cabot_ui.human_notifier import HumanNotifier

class ControlBase(object):
    #_anchor = geoutil.Anchor(lat=40.443228, lng=-79.945705, rotate=15) # NSH NavCog anchor
    #_anchor = geoutil.Anchor(lat=40.443262, lng=-79.945888, rotate=15.1) # 4fr
    #_anchor = geoutil.Anchor(lat=40.443259, lng=-79.945874, rotate=15.1) # 4fr-gazebo
    _anchor = geoutil.Anchor(lat=40.443259, lng=-79.945874, rotate=-164.9) # 4fr-gazebo
    #_anchor = geoutil.Anchor(lat=40.443259, lng=-79.945874, rotate=16) # 4fr-gazebo

    def __init__(self, datautil_instance=None, anchor_file=None):
        self.visualizer = visualizer.instance

        self.delegate = None

        # for data
        if datautil_instance is not None:
            self._datautil = datautil_instance
        else:
            self._datautil = datautil.instance
            self._datautil.init_by_server()

        # for current location
        anchor_file = None
        self.current_pose = None
        self._anchor = ControlBase._anchor
        if rospy.has_param("~anchor_file"):
            anchor_file = rospy.get_param("~anchor_file")
        rospy.loginfo("Anchor file is %s", anchor_file)
        if anchor_file is not None:
            temp = geoutil.get_anchor(yaml=anchor_file)
            if temp is not None:
                self._anchor = temp
            else:
                rospy.logwarn("could not load anchor_file \"%s\"", anchor_file)

        self.listener = tf.TransformListener()

        # skip if datautil is already initialized
        if self._datautil.is_analyzed:
            return

        rospy.loginfo("set anchor and analyze")
        self._datautil.set_anchor(self._anchor)
        self._datautil.analyze_features()


    # current location
        
    def current_local_pose(self):
        """get current local location"""
        rospy.logdebug(util.callee_name())
        rate = rospy.Rate(10.0)
        trans = rotation = None
        for i in xrange(0, 10):
            rate.sleep()
            try:
                (trans, rotation) = self.listener.lookupTransform('/map', '/base_link', rospy.Time())
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.current_pose = geoutil.Pose(x=trans[0], y=trans[1], r=euler[2])
                return self.current_pose
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        raise RuntimeError("no transformation")

    def current_global_pose(self):
        local = self.current_local_pose()
        rospy.logdebug("current location (%s)", local)
        _global = geoutil.local2global(local, self._anchor)
        return _global

    def current_location_id(self):
        """get id string for the current loaction in ROS"""
        ## TODO floor is hard coded
        _global = self.current_global_pose()
        return "latlng:%.7f:%.7f:4"%(_global.lat, _global.lng)


class Navigation(ControlBase):
    """Navigation node for Cabot"""

    def __init__(self, datautil_instance=None, anchor_file=None):

        super(Navigation, self).__init__(datautil_instance=datautil_instance, anchor_file=anchor_file)

        self.pois = []
        self.speed_pois = []
        self.info_pois = []
        self.turns = []
        
        self.i_am_ready = False
        self.goal_pose = None
        self.is_final_goal = False
        self.sub_routes = []

        #self.client = None
        self._loop_handle = None
        self.clutch_state = False

        self._max_speed = rospy.get_param("~max_speed", 1.1)
        self._max_acc = rospy.get_param("~max_acc", 0.3)

        self._current_path = None

        self.human_notifier = None

        self._client = actionlib.SimpleActionClient("/move_base", move_base_msgs.msg.MoveBaseAction)
        self._client.wait_for_server(timeout = rospy.Duration(5.0))
        rospy.loginfo("move_base is ready")
        
        clutch_output = rospy.get_param("~clutch_topic", "/cabot/clutch")
        self.clutch_pub = rospy.Publisher(clutch_output,
                                          std_msgs.msg.Bool,
                                          queue_size=10)
        self.speed_limit_pub = rospy.Publisher("/cabot/map_speed",
                                               std_msgs.msg.Float32,
                                               queue_size=10)
        self.path_sub = rospy.Subscriber("/move_base/NavfnROS/plan", nav_msgs.msg.Path,
                                         self._path_callback)
        self.human_sub = rospy.Subscriber("/tracked_humans", Obstacles, self._humans_callback)

        self._start_loop()

    ### public interfaces

    def set_destination(self, destination):
        """
        memo: current logic is not beautiful.
        1. get a NavCog route from the server
        2. get the last point of the route
        3. set goal pose via actionlib
        ## issues
          - cannot use NavCog topology
          - NavCog topology needs to be fixed 
        """
        rospy.loginfo(util.callee_name())
        from_id = self.current_location_id()
        to_id = destination

        rospy.loginfo("%s => %s", from_id, to_id)
        groute = self._datautil.get_route(from_id, to_id)

        self.sub_routes = self._devide_route(groute)
        rospy.loginfo("%d sub routes", len(self.sub_routes))

        self._navigate_next_sub_route()

    def pause_navigation(self):
        rospy.loginfo(util.callee_name())
        self._client.cancel_goal()
        
        self.clutch_state = False
        self.clutch_pub.publish(False)

        self.turns = []

        #self._stop_loop()

    def resume_navigation(self):
        rospy.loginfo(util.callee_name())
        if self._current_path is None:
            return
        #todo resume

    def cancel_navigation(self):
        """callback for cancel topic"""
        rospy.loginfo(util.callee_name())
        self.pause_navigation()
        self._current_path = None
        self.goal_pose = None
        self._stop_loop()

    ## private methods for navigation
    def _navigate_next_sub_route(self):
        next_route = self.sub_routes.pop(0)
        self._navigate_sub_route(next_route, is_final_goal=False if self.sub_routes else True) 
    
    @util.setInterval(0.1, times=1)
    def _navigate_sub_route(self, groute, is_final_goal=False):
        self.is_final_goal = is_final_goal
        rospy.loginfo(util.callee_name())
        rospy.loginfo(groute)
        
        self.pois = self._extract_pois(groute)
        self.visualizer.pois = self.pois
        self.speed_pois = [x for x in self.pois if isinstance(x, geojson.SpeedPOI)]
        self.info_pois = [x for x in self.pois if not isinstance(x, geojson.SpeedPOI)]

        rospy.loginfo("pois %s", self.pois)
        rospy.loginfo("speed pois %s", self.speed_pois)
        rospy.loginfo("info pois %s", self.info_pois)

        rospy.loginfo("convert route")
        lroute = self._convert_route(groute)

        rospy.loginfo("inerpolate route")
        ipath = self._interpolate_path(lroute)

        rospy.loginfo("convert to ROS path")
        path = self._create_ros_path(ipath)
        self._current_path = path

        # devide a path into some
        #rospy.loginfo("publish path")
        #path_publisher = rospy.Publisher("/path", nav_msgs.msg.Path, queue_size=1, latch=True)
        #path_publisher.publish(path)

        self._start_navigation_to(path.poses[-1].pose)

    def _start_navigation_to(self, goal_pose):
        rospy.loginfo(util.callee_name())

        self.delegate.start_navigation()

        goal = move_base_msgs.msg.MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = goal_pose

        #rospy.loginfo("wait_for_server")
        #self._client.wait_for_server()
        rospy.loginfo("sending goal %s", str(goal))
        self._client.send_goal(goal)
        rospy.loginfo("sent goal")
        self.goal_pose = goal.target_pose
        
        self.clutch_state = True
        self.clutch_pub.publish(True)
        self._start_loop()

    def _extract_pois(self, route):
        """extract pois along the route"""
        temp = []
        for (_, item) in enumerate(route):
            if isinstance(item, geojson.RouteLink):
                print item._id
                for poi in item.pois:
                    print "  ", type(poi), poi._id
                temp.extend(item.pois)
        return temp

    def _devide_route(self, groute):
        sub_routes = []
        temp = []
        for r in groute:
            if not isinstance(r, geojson.RouteLink):
                temp.append(r)
                continue

            temp.append(r)
            for p in r.pois:
                if isinstance(p, geojson.DoorPOI):
                    sub_routes.append(temp)
                    temp = []
                    break

        sub_routes.append(temp)
        return sub_routes

    def _convert_route(self, route):
        """convert a list of LineString into a list of points"""
        temp = []
        for (_, item) in enumerate(route[:-1]):
            if isinstance(item, geojson.RouteLink):
                if item.is_leaf and item.length < 3.0:
                    continue
                hasdoor=False
                for p in item.pois:
                    if isinstance(p, geojson.DoorPOI):
                        hasdoor=True
                if hasdoor:
                    continue
            print "convert with anchor", self._anchor
            convert = lambda g, a=self._anchor: geoutil.global2local(g, a)

            if isinstance(item.geometry, geojson.Point):
                temp.append(convert(item.geometry))
            elif isinstance(item.geometry, geojson.LineString):
                temp.append(convert(item.target_node.geometry))
        return temp

    def _create_ros_path(self, points):
        """convert a list points into a ROS path"""
        path = nav_msgs.msg.Path()
        path.header.frame_id = "map"
        path.poses = []
        quat = None
        for i in xrange(0, len(points)):
            start = points[i]
            pose = geometry_msgs.msg.PoseStamped()
            pose.header.frame_id = "map"
            pose.pose = geometry_msgs.msg.Pose()
            pose.pose.position.x = start.x
            pose.pose.position.y = start.y
            if i+1 < len(points):
                end = points[i+1]
                direction = math.atan2(end.y - start.y, end.x - start.x)
                quat = tf.transformations.quaternion_from_euler(0, 0, direction)
                pose.pose.orientation.x = quat[0]
                pose.pose.orientation.y = quat[1]
                pose.pose.orientation.z = quat[2]
                pose.pose.orientation.w = quat[3]
                path.poses.append(pose)
        return path

    def _interpolate_path(self, points):
        """interpolate path"""
        MAXIMUM = 0.1

        while True:
            flag = True
            for i in xrange(0, len(points)-1):
                start = points[i]
                end = points[i+1]
                if start.distance_to(end) > MAXIMUM:
                    points.insert(i+1, start.interpolate(end, 0.5))
                    flag = False
                    break
            if flag:
                break
        return points

    def _path_callback(self, path):
        if self.human_notifier is not None:
            self.human_notifier.update_path(path)

        if self.turns is None:
            try:
                self.turns = TurnDetector.detects(path)
                self.visualizer.turns = self.turns
                """
                for i in xrange(len(self.turns)-1, 0, -1):
                t1 = self.turns[i]
                if abs(t1.angle) < math.pi/8:
                    self.turns.pop(i)
                """
                for i in xrange(len(self.turns)-2, 0, -1):
                    t1 = self.turns[i]
                    t2 = self.turns[i+1]
                    if (t1.angle < 0 and 0 < t2.angle) or \
                       (t2.angle < 0 and 0 < t1.angle):
                        if 0 < abs(t1.angle) and abs(t1.angle) < math.pi/3 and \
                           0 < abs(t2.angle) and abs(t2.angle) < math.pi/3:
                            self.turns.pop(i+1)
            except ValueError as error:
                pass

            self.human_notifier = HumanNotifier(path)
            self.visualizer.visualize()

            def path_length(path):
                last = None
                d = 0
                for p in path.poses:
                    curr = numpy.array([p.pose.position.x, p.pose.position.y])
                    if last is None:
                        last = curr
                    d += numpy.linalg.norm(last-curr)
                    last = curr
                return d

            rospy.loginfo("path-length %.2f", path_length(path))
            

    def _humans_callback(self, msg):
        if self.human_notifier is not None:
            self.human_notifier.update_humans(msg)

    def _start_loop(self):
        rospy.loginfo(util.callee_name())
        if self._loop_handle is None:
            self._loop_handle = self._check_loop()

    def _stop_loop(self):
        rospy.loginfo(util.callee_name())
        if self._loop_handle is None:
            return
        self._loop_handle.set()
        self._loop_handle = None

    GOAL_POSITION_TORELANCE = 1
        
    @util.setInterval(0.1)
    def _check_loop(self):
        if rospy.is_shutdown():
            self._stop_loop()
            return

        self._check_ready()

        try:
            current_pose = self.current_local_pose()
            rospy.logdebug("current pose %s", current_pose)
        except RuntimeError:
            rospy.loginfo("could not get position")
            return

        rospy.logdebug("cabot is active")
        self._check_speed_limit(current_pose)
        self._check_info_poi(current_pose)
        self._check_turn(current_pose)
        self._check_avoiding_person(current_pose)
        # if something need to ask the user, may ask something
        
        if self._goal_is_not_active():
            rospy.logdebug("goal is not active")
            return
        rospy.logdebug("goal is active")

        if self._goal_reached(current_pose):
            rospy.logdebug("goal reached")
            return


    def _check_ready(self):
        if not self.i_am_ready and self._datautil.is_analyzed:
            self.delegate.i_am_ready()
            self.i_am_ready = True

    def _goal_is_not_active(self):
        if not self.goal_pose:
            return True
            
        if self._client.get_state() != GoalStatus.ACTIVE and \
           self._client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logdebug("state %d", self._client.get_state())
            return True
        return False

    def _goal_reached(self, current_pose):
        goal_pose = geoutil.Pose.from_pose_msg(self.goal_pose)
        distance = current_pose.distance_to(goal_pose)

        rospy.logdebug("distance %.2f, status=%d %d, %d", 
                       distance, self._client.get_state(), GoalStatus.SUCCEEDED, self.is_final_goal)
        # if the user is close enough to the goal, say arrived
        if distance < self.GOAL_POSITION_TORELANCE or \
           self._client.get_state() == GoalStatus.SUCCEEDED:
            if self.is_final_goal:
                self._stop_loop()
                self.delegate.have_arrived()
                rospy.sleep(2)
                self.pause_navigation()
                return True

        return False

    def _check_speed_limit(self, current_pose):
        # check speed limit
        if not self.speed_pois:
            return

        limit = self._max_speed
        for poi in self.speed_pois:
            dist = poi.distance_to(current_pose)
            if dist < 5.0:
                #c2p = geoutil.Pose.pose_from_points(current_pose, poi.local_pose)
                
                if poi.in_angle(current_pose): # and poi.in_angle(c2p):
                    limit = min(limit, max(poi.limit, math.sqrt(2.0 * dist * self._max_acc)))
                else:
                    limit = min(limit, self._max_speed)
        msg = std_msgs.msg.Float32()
        msg.data = limit
        self.speed_limit_pub.publish(msg)

    def _check_info_poi(self, current_pose):
        if not self.info_pois:
            return

        if self.info_pois:
            poi = min(self.info_pois, key=lambda p, c=current_pose: p.distance_to(c))
            
            if poi is not None and poi.distance_to(current_pose) < 8:
                #rospy.loginfo("%s, %s, %s", poi._id, poi.local_geometry, current_pose)
                if poi.is_approaching(current_pose):
                    rospy.loginfo("approaching %s", poi._id)
                    self.delegate.approaching_to_poi(poi=poi, pose=current_pose)
                elif poi.is_approached(current_pose):
                    rospy.loginfo("approached %s", poi._id)
                    if poi.needs_user_action():
                        poi.wait_user_action()
                        self.pause_navigation()
                        self.delegate.request_user_action(poi=poi, pose=current_pose)
                    self.delegate.approached_to_poi(poi=poi, pose=current_pose)
                elif poi.is_passed(current_pose):
                    rospy.loginfo("passed %s", poi._id)
                    if poi.is_waiting_user_action():
                        poi.user_action_completed()
                        self.delegate.passed_poi(poi=poi, pose=current_pose)
                        if self.sub_routes:
                            self._navigate_next_sub_route()


    def _check_turn(self, current_pose):
        # provide turn tactile notification 
        if not self.turns:
            return

        if self.turns is not None:
            for turn in self.turns:
                dist = current_pose.distance_to(geoutil.Point(xy=turn.start))
                if dist < 1.0 and not turn.passed:
                    turn.passed = True
                    self.delegate.notify_turn(turn=turn, pose=current_pose)

    def _check_avoiding_person(self, current_pose):
        # the robot may be going to avoid human
        if not self.human_notifier:
            return

        self.human_notifier.update_current_pose(current_pose)
        if self.human_notifier.should_notify():
            self.delegate.notify_human(angle=self.human_notifier.get_angle(),
                                       pose=current_pose)
            self.human_notifier.notified()




