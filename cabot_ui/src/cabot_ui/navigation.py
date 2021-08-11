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

import sys
import math
import numpy, numpy.linalg
import inspect

# ROS
import rospy
import rosparam
import tf
import actionlib
import move_base_msgs.msg
import nav2_msgs.msg
import std_msgs.msg
import nav_msgs.msg
import geometry_msgs.msg
import people_msgs.msg
from actionlib_msgs.msg import GoalStatus
from obstacle_detector.msg import Obstacles

# Other

import cabot_msgs.srv
from cabot import util
from cabot.handle_v2 import Handle
from cabot_ui import visualizer, geoutil, geojson, datautil
from cabot_ui.turn_detector import TurnDetector, Turn
from cabot_ui import navgoal
from cabot_ui.social_navigation import SocialNavigation
from cabot_ui.geoutil import AvoidingTarget, AvoidingTargetType
import queue_msgs.msg

class NavigationInterface(object):
    def i_am_ready(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def start_navigation(self, pose):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def notify_turn(self, turn=None, pose=None):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def notify_human(self, angle=0, pose=None):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def goal_canceled(self, goal):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def have_arrived(self, goal):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def approaching_to_avoiding_target(self, target=None, pose=None):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def approaching_to_poi(self, poi=None, pose=None):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def approached_to_poi(self, poi=None, pose=None):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def passed_poi(self, poi=None, pose=None):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

#    def request_action(self, goal=None, pose=None):
#        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))
#
#    def completed_action(self, goal=None, pose=None):
#        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def enter_goal(self, goal):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def exit_goal(self, goal):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def could_not_get_current_location(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def announce_social(self, message):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def please_call_elevator(self, pos):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def elevator_opening(self, pose):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def floor_changed(self, floor):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def queue_start_arrived(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def queue_proceed(self, pose=None):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def please_pass_door(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def door_passed(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))


class ControlBase(object):
    #_anchor = geoutil.Anchor(lat=40.443228, lng=-79.945705, rotate=15) # NSH NavCog anchor
    #_anchor = geoutil.Anchor(lat=40.443262, lng=-79.945888, rotate=15.1) # 4fr
    #_anchor = geoutil.Anchor(lat=40.443259, lng=-79.945874, rotate=15.1) # 4fr-gazebo
    #_anchor = geoutil.Anchor(lat=40.443259, lng=-79.945874, rotate=-164.9) # 4fr-gazebo
    #_anchor = geoutil.Anchor(lat=40.443259, lng=-79.945874, rotate=16) # 4fr-gazebo

    def __init__(self, datautil_instance=None, anchor_file=None):
        self.visualizer = visualizer.instance

        self.delegate = NavigationInterface()
        self.listener = tf.TransformListener()
        self.current_pose = None
        self.current_odom_pose = None
        self.current_floor = int(rospy.get_param("initial_floor", 1))
        rospy.loginfo("current_floor is %d", self.current_floor)

        # for current location
        self._anchor = None
        if rospy.has_param("~anchor_file"):
            anchor_file = rospy.get_param("~anchor_file")
        rospy.loginfo("Anchor file is %s", anchor_file)
        if anchor_file is not None:
            temp = geoutil.get_anchor(yaml=anchor_file)
            if temp is not None:
                self._anchor = temp
            else:
                rospy.logwarn("could not load anchor_file \"%s\"", anchor_file)

        rospy.loginfo("set anchor and analyze")
        rospy.loginfo(self._anchor)
        # for data
        if datautil_instance is not None:
            self._datautil = datautil_instance
            self._datautil.set_anchor(self._anchor)
        else:
            self._datautil = datautil.getInstance()
            self._datautil.set_anchor(self._anchor)
            self._datautil.init_by_server()

    # current location
        
    def current_local_pose(self, frame=None):
        """get current local location"""
        if frame is None:
            frame = self._global_map_name
        rate = rospy.Rate(10.0)
        trans = rotation = None
        for i in range(0, 10):
            try:
                (trans, rotation) = self.listener.lookupTransform(frame, '/base_footprint', rospy.Time())
                euler = tf.transformations.euler_from_quaternion(rotation)
                current_pose = geoutil.Pose(x=trans[0], y=trans[1], r=euler[2])
                return current_pose
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()
        raise RuntimeError("no transformation")

    def current_local_odom_pose(self):
        """get current local odom location"""
        rate = rospy.Rate(10.0)
        trans = rotation = None
        for i in range(0, 10):
            try:
                (trans, rotation) = self.listener.lookupTransform('local/odom', '/local/base_footprint', rospy.Time())
                euler = tf.transformations.euler_from_quaternion(rotation)
                current_pose = geoutil.Pose(x=trans[0], y=trans[1], r=euler[2])
                return current_pose
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()
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
        return "latlng:%.7f:%.7f:%d"%(_global.lat, _global.lng, self.current_floor)


class Navigation(ControlBase, navgoal.GoalInterface):
    """Navigation node for Cabot"""

    ACTIONS = {"navigate_to_pose": nav2_msgs.msg.NavigateToPoseAction,
               "navigate_through_poses": nav2_msgs.msg.NavigateThroughPosesAction}
    NS = ["", "/local"]

    def __init__(self, datautil_instance=None, anchor_file=None):

        self.current_floor = None
        self.current_frame = None

        super(Navigation, self).__init__(datautil_instance=datautil_instance, anchor_file=anchor_file)

        self.info_pois = []
        self.queue_wait_pois = []
        self.speed_pois = []
        self.turns = []

        self.i_am_ready = False
        self._sub_routes = []
        self._current_goal = None

        #self.client = None
        self._loop_handle = None
        self.clutch_state = False
        

        self._max_speed = rospy.get_param("~max_speed", 1.1)
        self._max_acc = rospy.get_param("~max_acc", 0.3)

        self._global_map_name = rospy.get_param("~global_map_name", "map")
        self.visualizer.global_map_name = self._global_map_name

        self.social_navigation = SocialNavigation(self.listener)

        self._clients = {}

        for ns in Navigation.NS:
            for action in Navigation.ACTIONS:
                name = "/".join([ns, action])
                self._clients[name] = actionlib.SimpleActionClient(name, Navigation.ACTIONS[action])
                if not self._clients[name].wait_for_server(timeout = rospy.Duration(2.0)):
                    rospy.logerr("client for {} is not ready".format(name))

        self._spin_client = actionlib.SimpleActionClient("/spin", nav2_msgs.msg.SpinAction)
        if not self._spin_client.wait_for_server(timeout = rospy.Duration(2.0)):
            rospy.logerr("spin is not ready")

        clutch_output = rospy.get_param("~clutch_topic", "/cabot/clutch")
        self.clutch_pub = rospy.Publisher(clutch_output, std_msgs.msg.Bool, queue_size=10)
        map_speed_output = rospy.get_param("~map_speed_topic", "/cabot/map_speed")
        self.speed_limit_pub = rospy.Publisher(map_speed_output, std_msgs.msg.Float32, queue_size=10)
        current_floor_input = rospy.get_param("~current_floor_topic", "/current_floor")
        self.current_floor_sub = rospy.Subscriber(current_floor_input, std_msgs.msg.Int64, self._current_floor_callback)
        current_frame_input = rospy.get_param("~current_frame_topic", "/current_frame")
        self.current_frame_sub = rospy.Subscriber(current_frame_input, std_msgs.msg.String, self._current_frame_callback)

        plan_input = rospy.get_param("~plan_topic", "/move_base/NavfnROS/plan")
        self.plan_sub = rospy.Subscriber(plan_input, nav_msgs.msg.Path, self._plan_callback)
        path_output = rospy.get_param("~path_topic", "/path")
        self.path_pub = rospy.Publisher(path_output, nav_msgs.msg.Path, queue_size=1, latch=True)

        # deprecated
        # self.updated_goal_sub = rospy.Subscriber("/updated_goal", geometry_msgs.msg.PoseStamped, self._goal_updated_callback)
        people_input = rospy.get_param("~people_topic", "/people")
        self.people_sub = rospy.Subscriber(people_input, people_msgs.msg.People, self._people_callback)
        self.last_people_msg = None
        collision_input = rospy.get_param("~collision_topic", "/collision")
        self.collision_sub = rospy.Subscriber(collision_input, geometry_msgs.msg.PoseStamped, self._collision_callback)
        self.avoiding_targets = []

        self.current_queue_msg = None
        self.need_queue_start_arrived_info = False
        self.need_queue_proceed_info = False
        queue_input = rospy.get_param("~queue_topic", "/queue_people_py/queue")
        self.queue_sub = rospy.Subscriber(queue_input, queue_msgs.msg.Queue, self._queue_callback)
        queue_speed_output = rospy.get_param("~queue_speed_topic", "/cabot/queue_speed")
        self.queue_speed_limit_pub = rospy.Publisher(queue_speed_output, std_msgs.msg.Float32, queue_size=10)
        self._queue_tail_ignore_path_dist = rospy.get_param("~queue_tail_ignore_path_dist", 0.8)
        self._queue_wait_pass_tolerance = rospy.get_param("~queue_wait_pass_tolerance", 0.3)
        self._queue_wait_arrive_tolerance = rospy.get_param("~queue_wait_arrive_tolerance", 0.2)
        self._queue_tail_dist_error_tolerance = rospy.get_param("~queue_tail_dist_error_tolerance", 0.5)
        self._queue_wait_position_offset = rospy.get_param("~queue_wait_position_offset", 0.2)
        self.initial_queue_interval = rospy.get_param("~initial_queue_interval", 1.0)
        self.current_queue_interval = self.initial_queue_interval

        self.initial_social_distance = None
        self.current_social_distance = None
        get_social_distance_topic = rospy.get_param("~get_social_distance_topic", "/get_social_distance")
        self.get_social_distance_sub = rospy.Subscriber(get_social_distance_topic, geometry_msgs.msg.Point, self._get_social_distance_callback)
        set_social_distance_topic = rospy.get_param("~set_social_distance_topic", "/set_social_distance")
        self.set_social_distance_pub = rospy.Publisher(set_social_distance_topic, geometry_msgs.msg.Point, queue_size=1, latch=True)

        self._start_loop()

    def process_event(self, event):
        '''cabot navigation event'''
        ## do not provide social navigation messages while queue navigation
        if not isinstance(self._current_goal, navgoal.QueueNavGoal):
            if self.social_navigation is not None:
                self.social_navigation.event = event

        if event.param == "elevator_door_may_be_ready":
            self.delegate.elevator_opening(self.current_pose)

        if event.param == "navigation_start":
            self.delegate.start_navigation(self.current_pose)

    ## callback functions
    def _current_floor_callback(self, msg):
        self.current_floor = msg.data
        if msg.data >= 0:
            self.current_floor = msg.data + 1
        rospy.loginfo_throttle(1, "Current floor is %d", self.current_floor)

    def _current_frame_callback(self, msg):
        self.current_frame = msg.data
        rospy.loginfo_throttle(1, "Current frame is %s", self.current_frame)

    def _plan_callback(self, path):
        if self.social_navigation is not None:
            self.social_navigation.path = path

        if self.turns is not None or True:
            try:
                self.turns = TurnDetector.detects(path)
                self.visualizer.turns = self.turns

                rospy.loginfo("turns: %s", str(self.turns))
                """
                for i in range(len(self.turns)-1, 0, -1):
                t1 = self.turns[i]
                if abs(t1.angle) < math.pi/8:
                    self.turns.pop(i)
                """
                for i in range(len(self.turns)-2, 0, -1):
                    t1 = self.turns[i]
                    t2 = self.turns[i+1]
                    if (t1.angle < 0 and 0 < t2.angle) or \
                       (t2.angle < 0 and 0 < t1.angle):
                        if 0 < abs(t1.angle) and abs(t1.angle) < math.pi/3 and \
                           0 < abs(t2.angle) and abs(t2.angle) < math.pi/3:
                            self.turns.pop(i+1)
            except ValueError as error:
                pass

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

    def _queue_callback(self, msg):
        self.current_queue_msg = msg
        rospy.loginfo_throttle(1, "Current people in queue %s", str(self.current_queue_msg.people_names))

    def _get_social_distance_callback(self, msg):
        self.current_social_distance = msg
        if self.initial_social_distance is None:
            self.initial_social_distance = self.current_social_distance
        rospy.loginfo_throttle(3, "Current social distance parameter is %s", self.current_social_distance)

    def _get_queue_interval_callback(self, msg):
        self.current_queue_interval = msg.data
        if self.initial_queue_interval is None:
            self.initial_queue_interval = self.current_queue_interval
        rospy.loginfo_throttle(3, "Current queue interval parameter is %s", self.current_queue_interval)

    def _goal_updated_callback(self, msg):
        if self._current_goal:
            self._current_goal.update_goal(msg)

    def _people_callback(self, msg):
        self.last_people_msg = msg

    def _collision_callback(self, msg):
        if self.last_people_msg is not None:
            temp = []
            for p in self.last_people_msg.people:
                dx = p.position.x - msg.pose.position.x
                dy = p.position.y - msg.pose.position.y
                dist = math.sqrt(dx*dx+dy*dy)
                v = math.sqrt(pow(p.velocity.x,2)+pow(p.velocity.y,2))
                if dist < 1.5 and v < 0.2:
                    temp.append(self.listener.transformPose(self._global_map_name, msg))
            if temp:
                self.avoiding_targets.append(AvoidingTarget(temp, AvoidingTargetType.Person))
                return
        self.avoiding_targets.append(AvoidingTarget([self.listener.transformPose(self._global_map_name, msg)], AvoidingTargetType.Something))
        rospy.loginfo("collision_callback {}".format(self.avoiding_targets))

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
        rospy.loginfo("navigation.{} called".format(util.callee_name()))
        try:
            from_id = self.current_location_id()
        except RuntimeError as e:
            rospy.logerr("could not get current location")
            self.delegate.could_not_get_current_location()
            return

        # specify last orientation
        if destination.find("@") > -1:
            (to_id, yaw_str) = destination.split("@")
            yaw = float(yaw_str)
            self._sub_goals = navgoal.make_goals(self, self._datautil.get_route(from_id, to_id), self._anchor, yaw=yaw)
        else:
            to_id = destination
            self._sub_goals = navgoal.make_goals(self, self._datautil.get_route(from_id, to_id), self._anchor)

        # navigate from the first path
        self._navigate_next_sub_goal()

    def retry_navigation(self):
        rospy.loginfo("navigation.{} called".format(util.callee_name()))
        self.turns = []

        self._sub_goals.insert(0, self._current_goal)
        self._navigate_next_sub_goal()

    def pause_navigation(self):
        rospy.loginfo("navigation.{} called".format(util.callee_name()))

        for name in self._clients:
            if self._clients[name].get_state() == GoalStatus.ACTIVE:
                self._clients[name].cancel_goal()
        self.set_clutch(False)
        self.turns = []

        self._sub_goals.insert(0, self._current_goal)

    def resume_navigation(self):
        rospy.loginfo("navigation.{} called".format(util.callee_name()))
        self._navigate_next_sub_goal()

    def cancel_navigation(self):
        """callback for cancel topic"""
        rospy.loginfo("navigation.{} called".format(util.callee_name()))
        self.pause_navigation()
        self._current_goal = None
        self._stop_loop()

    ## private methods for navigation
    def _navigate_next_sub_goal(self):
        rospy.loginfo("navigation.{} called".format(util.callee_name()))
        if self._sub_goals:
            self._current_goal = self._sub_goals.pop(0)
            self._navigate_sub_goal(self._current_goal)
            return

        if self._current_goal:
            self._stop_loop()
            self.cancel_navigation()

    @util.setInterval(0.01, times=1)
    def _navigate_sub_goal(self, goal):
        rospy.loginfo("navigation.{} called".format(util.callee_name()))

        if isinstance(goal, navgoal.NavGoal):
            self.visualizer.pois = goal.pois
            self.visualizer.visualize()
            self.speed_pois = [x for x in goal.pois if isinstance(x, geojson.SpeedPOI)]
            self.info_pois = [x for x in goal.pois if not isinstance(x, geojson.SpeedPOI)]
            self.queue_wait_pois = [x for x in goal.pois if isinstance(x, geojson.QueueWaitPOI)]
        else:
            self.visualizer.pois = []
            self.speed_poi = []
            self.info_pois = []
            self.queue_wait_pois = []
        self.turns = []

        rospy.loginfo("goal %s", goal)
        try:
            goal.enter()
        except:
            import traceback
            rospy.logerr(traceback.format_exc())
        self._start_loop()

    def _start_loop(self):
        rospy.loginfo("navigation.{} called".format(util.callee_name()))
        if self._loop_handle is None:
            self._loop_handle = self._check_loop()

    def _stop_loop(self):
        rospy.loginfo("navigation.{} called".format(util.callee_name()))
        if self._loop_handle is None:
            return
        self._loop_handle.set()
        self._loop_handle = None


    ## Main loop of navigation
    GOAL_POSITION_TORELANCE = 1
        
    @util.setInterval(0.1)
    def _check_loop(self):
        if rospy.is_shutdown():
            self._stop_loop()
            return

        ## need a robot position
        try:
            self.current_pose = self.current_local_pose()
            rospy.logdebug_throttle(1, "current pose %s", self.current_pose)
            self.current_odom_pose = self.current_local_odom_pose()
        except RuntimeError:
            rospy.loginfo_throttle(3, "could not get position")
            return

        ## wait data is analyzed
        if not self._datautil.is_analyzed:
            return
        ## say I am ready once
        if not self.i_am_ready:
            rospy.logdebug("i am ready")
            self.delegate.i_am_ready()
            self.i_am_ready = True

        if self._current_goal is None:
            return

        ## cabot is active now
        rospy.logdebug_throttle(10, "cabot is active")

        try:
            self._check_avoiding_targets(self.current_pose)
            self._check_info_poi(self.current_pose)
            self._check_speed_limit(self.current_pose)
            self._check_turn(self.current_pose)
            self._check_queue_wait(self.current_pose)
            self._check_social(self.current_pose)
            self._check_goal(self.current_pose)
        except Exception as e:
            import traceback
            rospy.logerr_throttle(3, traceback.format_exc())

    def _check_avoiding_targets(self, current_pose):
        rospy.loginfo("navigation.{} called".format(util.callee_name()))
        removing = []
        for target in self.avoiding_targets:
            rospy.loginfo("{} is approaching {}".format(target, target.is_approaching(current_pose)))
            if target.is_approaching(current_pose):
                self.delegate.approaching_to_avoiding_target(target=target, pose=current_pose)
                removing.append(target)

        for r in removing:
            self.avoiding_targets.remove(r)

    def _check_info_poi(self, current_pose):
        if not self.info_pois:
            return

        poi = min(self.info_pois, key=lambda p, c=current_pose: p.distance_to(c))

        if poi is not None and poi.distance_to(current_pose) < 8:
            #rospy.loginfo("%s, %s, %s", poi._id, poi.local_geometry, current_pose)
            if poi.is_approaching(current_pose):
                rospy.loginfo("approaching %s", poi._id)
                self.delegate.approaching_to_poi(poi=poi, pose=current_pose)
            elif poi.is_approached(current_pose):
                rospy.loginfo("approached %s", poi._id)
                self.delegate.approached_to_poi(poi=poi, pose=current_pose)
            elif poi.is_passed(current_pose):
                rospy.loginfo("passed %s", poi._id)
                self.delegate.passed_poi(poi=poi, pose=current_pose)

    def _check_speed_limit(self, current_pose):
        # check speed limit
        if not self.speed_pois:
            return

        limit = self._max_speed
        for poi in self.speed_pois:
            dist = poi.distance_to(current_pose)
            if dist < 5.0:
                if poi.in_angle(current_pose): # and poi.in_angle(c2p):
                    limit = min(limit, max(poi.limit, math.sqrt(2.0 * dist * self._max_acc)))
                else:
                    limit = min(limit, self._max_speed)
                rospy.logdebug("speed poi dist=%.2fm, limit=%.2f", dist, limit)
        msg = std_msgs.msg.Float32()
        msg.data = limit
        self.speed_limit_pub.publish(msg)

    def _check_turn(self, current_pose):
        # provide turn tactile notification 
        if not self.turns:
            return

        rospy.loginfo_throttle(1, "check turn")
        if self.turns is not None:
            for turn in self.turns:
                try:
                    turn_pose = self.listener.transformPose(self._global_map_name, turn.pose)
                    dist = current_pose.distance_to(geoutil.Point(xy=turn_pose.pose.position))
                    if dist < 0.25 and not turn.passed:
                        turn.passed = True
                        rospy.loginfo("notify turn %s", str(turn))
                        self.delegate.notify_turn(turn=turn, pose=current_pose)
                except:
                    rospy.logerr_throttle(3, "could not convert pose for checking turn POI")

    def _check_queue_wait(self, current_pose):
        if not isinstance(self._current_goal, navgoal.QueueNavGoal) or not self.queue_wait_pois:
            return

        # Select queue wait POI which robot did not pass yet.
        # Even if POI is marked as passed, add POI which is closer than _queue_wait_pass_tolerance.
        poi = None
        forward_queue_wait_pois = [x for x in self.queue_wait_pois if geoutil.is_forward_point(current_pose, x) or x.distance_to(current_pose)<self._queue_wait_pass_tolerance]
        if len(forward_queue_wait_pois)>0:
            poi = min(forward_queue_wait_pois, key=lambda p, c=current_pose: p.distance_to(c))

        # control speed by Queue POI
        limit = self._max_speed
        if poi is not None:
            poi_pose = poi.to_pose_msg()
            poi_position = numpy.array([poi_pose.position.x, poi_pose.position.y])

            current_position = numpy.array([current_pose.x, current_pose.y])
            current_position_on_queue_path = geoutil.get_projected_point_to_line(current_position, poi_position, poi.link_orientation)

            if len(self.current_queue_msg.head_tail)>0:
                tail_pose = geometry_msgs.msg.PoseStamped()
                tail_pose.header = self.current_queue_msg.header
                tail_pose.pose.position = self.current_queue_msg.head_tail[-1]
                try:
                    tail_global_pose = self.listener.transformPose(self._global_map_name, tail_pose)
                    tail_global_position = numpy.array([tail_global_pose.pose.position.x, tail_global_pose.pose.position.y])
                    tail_global_position_on_queue_path = geoutil.get_projected_point_to_line(tail_global_position, poi_position, poi.link_orientation)

                    dist_tail = numpy.linalg.norm(tail_global_position_on_queue_path-current_position_on_queue_path)
                    dist_queue_wait = numpy.linalg.norm(poi_position-current_position_on_queue_path)
                    # If distance from next queue wait point to queue tail is larger than twice of queue wait interval,
                    # there should be another queue wait point. Skip next queue wait point.
                    if geoutil.is_forward_point(current_pose, poi):
                        dist_tail_queue_wait = dist_tail - dist_queue_wait
                    else:
                        dist_tail_queue_wait = dist_tail + dist_queue_wait
                    if dist_tail_queue_wait > max(2.0*self.current_queue_interval - self._queue_tail_dist_error_tolerance, self.current_queue_interval):
                        rospy.loginfo_throttle(1, "Skip next queue wait POI, POI is far from robot. dist_tail_queue_wait=%f, dist_tail=%f, dist_queue_wait=%f", dist_tail_queue_wait, dist_tail, dist_queue_wait)
                    else:
                        dist_tail_on_queue_tail = numpy.linalg.norm(tail_global_position_on_queue_path-tail_global_position)
                        # If distance from queue tail to queue path is larger than queue_tail_ignore_path_dist, robot and queue tail person should be on different queue paths.
                        # Skip next queue wait point.
                        if (dist_tail_on_queue_tail > self._queue_tail_ignore_path_dist):
                            rospy.loginfo_throttle(1, "Skip next queue wait POI, tail is far from path. dist_tail_on_queue_tail=%f", dist_tail_on_queue_tail)
                        else:
                            # adjust Queue POI position by offset _queue_wait_position_offset
                            poi_orientation = tf.transformations.euler_from_quaternion([poi.link_orientation.x, poi.link_orientation.y, poi.link_orientation.z, poi.link_orientation.w])
                            poi_offset_position = poi_position + numpy.array([self._queue_wait_position_offset*math.cos(poi_orientation[2]), self._queue_wait_position_offset*math.sin(poi_orientation[2])])

                            # limit speed by using adjusted Queue POI
                            dist_queue_wait_offset = numpy.linalg.norm(poi_offset_position-current_position_on_queue_path)
                            limit = min(limit, math.sqrt(2.0 * max(0.0, dist_queue_wait_offset - self._queue_wait_arrive_tolerance) * self._max_acc))
                            rospy.loginfo_throttle(1, "Set speed limit=%f. dist_tail_queue_wait=%f, dist_tail=%f, dist_queue_wait=%f", limit, dist_tail_queue_wait, dist_tail, dist_queue_wait)
                except:
                    rospy.logerr_throttle(3, "could not convert pose for checking queue POI")
            else:
                rospy.loginfo_throttle(1, "No queue people")
        msg = std_msgs.msg.Float32()
        msg.data = limit
        self.queue_speed_limit_pub.publish(msg)

        # announce beginning of the queue
        if self.need_queue_start_arrived_info and limit < self._max_speed:
            self.delegate.queue_start_arrived()
            self.need_queue_start_arrived_info = False
        # announce proceed in queue navigation
        if not self.need_queue_proceed_info and limit == 0.0:
            self.need_queue_proceed_info = True
        if self.need_queue_proceed_info and limit == self._max_speed:
            self.delegate.queue_proceed(pose=current_pose)
            self.need_queue_proceed_info = False

    def _check_social(self, current_pose):
        if self.social_navigation is None:
            return

        ## do not provide social navigation messages while queue navigation
        if isinstance(self._current_goal, navgoal.QueueNavGoal):
            return

        self.social_navigation.current_pose = current_pose
        message = self.social_navigation.get_message()
        if message is not None:
            self.delegate.announce_social(message)

    
    def _check_goal(self, current_pose):
        goal = self._current_goal
        if not goal:
            return

        goal.check(current_pose)

        if goal.is_canceled:
            # todo cancel
            self.delegate.goal_canceled(goal)
            self._stop_loop()
            return

        if not goal.is_completed:
            return

        goal.exit()

        if goal.need_to_announce_arrival:
            self.delegate.have_arrived(goal)

        self._navigate_next_sub_goal()

    ### GoalInterface

    def enter_goal(self, goal):
        self.delegate.enter_goal(goal)

    def exit_goal(self, goal):
        self.delegate.exit_goal(goal)

    def announce_social(self, messages):
        self.delegate.announce_social(messages)

    def navigate_to_pose(self, goal_pose, behavior_tree, done_cb, namespace=""):
        rospy.loginfo("{}/navigate_to_pose".format(namespace))
        client = self._clients["/".join([namespace, "navigate_to_pose"])]
        goal = nav2_msgs.msg.NavigateToPoseGoal()
        goal.behavior_tree = behavior_tree

        if namespace == "":
            goal.pose = self.listener.transformPose("map", goal_pose)
            goal.pose.header.stamp = rospy.Time.now()
            goal.pose.header.frame_id = "map"
            client.send_goal(goal, done_cb)
        elif namespace == "local":
            goal.pose = goal_pose
            goal.pose.header.stamp = rospy.Time.now()
            goal.pose.header.frame_id = "local/odom"
            client.send_goal(goal, done_cb)
        else:
            rospy.loginfo("unknown namespace %s", str(namespace))

        self.visualizer.reset()
        self.visualizer.goal = goal
        self.visualizer.visualize()
        rospy.loginfo("sent goal %s", str(goal))
        # need to move into the BT
        #self.delegate.start_navigation(self.current_pose)

    def navigate_through_poses(self, goal_poses, behavior_tree, done_cb, namespace=""):
        rospy.loginfo("{}/navigate_through_poses".format(namespace))
        client = self._clients["/".join([namespace, "navigate_through_poses"])]
        goal = nav2_msgs.msg.NavigateThroughPosesGoal()
        goal.behavior_tree = behavior_tree

        if namespace == "":
            goal.poses = []
            for pose in goal_poses:
                t_pose = self.listener.transformPose("map", pose)
                t_pose.pose.position.z = 0
                t_pose.header.stamp = rospy.Time.now()
                t_pose.header.frame_id = "map"
                goal.poses.append(t_pose)
            client.send_goal(goal, done_cb)
        elif namespace == "local":
            goal.poses = []
            for pose in goal_poses:
                t_pose = pose
                t_pose.header.stamp = rospy.Time.now()
                t_pose.header.frame_id = "local/odom"
                goal.poses.append(t_pose)
            client.send_goal(goal, done_cb)
        else:
            rospy.loginfo("unknown namespace %s", str(namespace))

        self.visualizer.reset()
        self.visualizer.goal = goal
        self.visualizer.visualize()
        rospy.loginfo("sent goal %s", str(goal))
        # need to move into the BT
        #self.delegate.start_navigation(self.current_pose)

    def turn_towards(self, orientation, callback, clockwise=0):
        rospy.loginfo("turn_towards")
        self._turn_towards(orientation, callback, clockwise=clockwise)

    @util.setInterval(0.01, times=1)
    def _turn_towards(self, orientation, callback, clockwise=0):
        goal = nav2_msgs.msg.SpinGoal()
        diff = geoutil.diff_angle(self.current_pose.orientation, orientation)

        rospy.loginfo("current pose %s, diff %.2f", str(self.current_pose), diff)

        if abs(diff) > 0.05 :
            if (clockwise < 0 and diff < - math.pi / 4) or \
               (clockwise > 0 and diff > + math.pi / 4):
                diff = diff - clockwise * math.pi * 2
            #use orientation.y for target spin angle
            rospy.loginfo("send turn %.2f", diff)
            # only use y for yaw
            turn_yaw = diff - (diff / abs(diff) * 0.05)
            goal.target_yaw = turn_yaw
            self._spin_client.send_goal(goal, lambda x,y: self._turn_towards(orientation, callback, clockwise=clockwise))
            rospy.loginfo("sent goal %s", str(goal))

            # add position and use quaternion to visualize
            #self.visualizer.goal = goal
            #self.visualizer.visualize()
            #rospy.loginfo("visualize goal %s", str(goal))
            self.delegate.notify_turn(turn=Turn(self.current_pose.to_pose_stamped_msg(self._global_map_name), turn_yaw), pose=self.current_pose)
            rospy.loginfo("notify turn %s", str(turn_yaw))
        else:
            rospy.loginfo("turn completed {}".format(diff))            
            callback(GoalStatus.SUCCEEDED, None)

    def goto_floor(self, floor, callback):
        self._goto_floor(floor, callback)

    @util.setInterval(0.01, times=1)
    def _goto_floor(self, floor, callback):
        rospy.loginfo("go to floor {}".format(floor))
        rate = rospy.Rate(2)
        while self.current_floor != floor:
            rate.sleep()

        rospy.loginfo("floor is changed")
        self.delegate.floor_changed(self.current_floor)
        while not rospy.is_shutdown():
            rospy.loginfo("trying to find tf from map to %s", self.current_frame)
            try:
                rate.sleep()
                trans = self.listener.lookupTransform("map", self.current_frame, rospy.Time())
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Could not find tf from map to %s", current_frame)
                continue

        rospy.loginfo("tf is changed")
        rate.sleep()

        callback(GoalStatus.SUCCEEDED, None)

    def set_clutch(self, flag):
        self.clutch_state = flag
        self.clutch_pub.publish(self.clutch_state)


    def publish_path(self, global_path, convert=True):
        local_path = global_path
        if convert:
            local_path = nav_msgs.msg.Path()
            local_path.header = global_path.header

            for pose in global_path.poses:
                local_path.poses.append(self.listener.transformPose("map", pose))
                local_path.poses[-1].pose.position.z = 0
        
        self.path_pub.publish(local_path)

    def please_call_elevator(self, pos):
        self.delegate.please_call_elevator(pos)

    def please_pass_door(self):
        self.delegate.please_pass_door()

    def door_passed(self):
        self.delegate.door_passed()

    def global_map_name(self):
        return self._global_map_name
