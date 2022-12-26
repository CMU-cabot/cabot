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

import math
import inspect
import numpy
import numpy.linalg
import threading
import time

# ROS
import rclpy
from rclpy.time import Duration, Time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import tf2_ros
import tf_transformations
import nav2_msgs.action
import std_msgs.msg
import nav_msgs.msg
import geometry_msgs.msg
from actionlib_msgs.msg import GoalStatus
import tf2_geometry_msgs.tf2_geometry_msgs
from ament_index_python.packages import get_package_share_directory

# Other
from cabot import util
from cabot_ui import visualizer, geoutil, geojson, datautil
from cabot_ui.turn_detector import TurnDetector, Turn
from cabot_ui import navgoal
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil
from cabot_ui.social_navigation import SocialNavigation
import queue_msgs.msg
from mf_localization_msgs.msg import MFLocalizeStatus


class NavigationInterface(object):
    def activity_log(self, category="", text="", memo=""):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def i_am_ready(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def start_navigation(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def update_pose(self, **kwargs):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def notify_turn(self, turn=None):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def notify_human(self, angle=0):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def goal_canceled(self, goal):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def have_arrived(self, goal):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def approaching_to_poi(self, poi=None):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def approached_to_poi(self, poi=None):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def passed_poi(self, poi=None):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def enter_goal(self, goal):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def exit_goal(self, goal):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def could_not_get_current_location(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def announce_social(self, message):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def please_call_elevator(self, pos):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def elevator_opening(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def floor_changed(self, floor):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def queue_start_arrived(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def queue_proceed(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def please_pass_door(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def door_passed(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def please_follow_behind(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")

    def please_return_position(self):
        CaBotRclpyUtil.error(F"{inspect.currentframe().f_code.co_name} is not implemented")


class ControlBase(object):
    # _anchor = geoutil.Anchor(lat=40.443228, lng=-79.945705, rotate=15) # NSH NavCog anchor
    # _anchor = geoutil.Anchor(lat=40.443262, lng=-79.945888, rotate=15.1) # 4fr
    # _anchor = geoutil.Anchor(lat=40.443259, lng=-79.945874, rotate=15.1) # 4fr-gazebo
    # _anchor = geoutil.Anchor(lat=40.443259, lng=-79.945874, rotate=-164.9) # 4fr-gazebo
    # _anchor = geoutil.Anchor(lat=40.443259, lng=-79.945874, rotate=16) # 4fr-gazebo

    def __init__(self, node: Node, datautil_instance=None, anchor_file=''):
        self._node = node
        self._logger = node.get_logger()
        self.visualizer = visualizer.instance(node)

        self.delegate = NavigationInterface()
        self.buffer = tf2_ros.Buffer(Duration(seconds=10), node)
        self.listener = tf2_ros.TransformListener(self.buffer, node)
        self.current_pose = None
        self.current_odom_pose = None
        self.current_floor = node.declare_parameter("initial_floor", 1).value
        self.floor_is_changed_at = node.get_clock().now()
        self._logger.info(F"current_floor is {self.current_floor}")

        # for current location
        self._anchor = None
        anchor_file = node.declare_parameter("anchor_file", anchor_file).value
        self._logger.info(F"Anchor file is {anchor_file}")
        if anchor_file is not None:
            temp = geoutil.get_anchor(anchor_file)
            if temp is not None:
                self._anchor = temp
            else:
                self._logger.warn(F"could not load anchor_file \"{anchor_file}\"")

        self._logger.info("set anchor and analyze")
        self._logger.info(F"anchor={str(self._anchor)}")
        # for data
        if datautil_instance is not None:
            self._datautil = datautil_instance
            self._datautil.set_anchor(self._anchor)
        else:
            self._datautil = datautil.getInstance(node)
            self._datautil.set_anchor(self._anchor)
            self._datautil.init_by_server()

    # current location
    def current_ros_pose(self, frame=None):
        """get current local location"""
        if frame is None:
            frame = self._global_map_name
        rate = self._node.create_rate(10.0)
        
        for i in range(0, 10):
            try:
                transformStamped = self.buffer.lookup_transform(
                    frame, 'base_footprint', CaBotRclpyUtil.time_zero())
                ros_pose = geometry_msgs.msg.Pose()
                ros_pose.position.x = transformStamped.transform.translation.x
                ros_pose.position.y = transformStamped.transform.translation.y
                ros_pose.position.z = transformStamped.transform.translation.z
                ros_pose.orientation.x = transformStamped.transform.rotation.x
                ros_pose.orientation.y = transformStamped.transform.rotation.y
                ros_pose.orientation.z = transformStamped.transform.rotation.z
                ros_pose.orientation.w = transformStamped.transform.rotation.w
                return ros_pose
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                continue
            rate.sleep()
        raise RuntimeError("no transformation")

    def current_local_pose(self, frame=None):
        """get current local location"""
        if frame is None:
            frame = self._global_map_name
        rate = self._node.create_rate(10.0)

        for i in range(0, 10):
            try:
                transformStamped = self.buffer.lookup_transform(
                    frame, 'base_footprint', CaBotRclpyUtil.time_zero())
                translation = transformStamped.transform.translation
                rotation = transformStamped.transform.rotation
                euler = tf_transformations.euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
                current_pose = geoutil.Pose(x=translation.x, y=translation.y, r=euler[2])
                return current_pose
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                continue
            rate.sleep()
        raise RuntimeError("no transformation")

    def current_local_odom_pose(self):
        """get current local odom location"""
        rate = self._node.create_rate(10.0)

        for i in range(0, 10):
            try:
                transformStamped = self.buffer.lookup_transform(
                    'local/odom', 'local/base_footprint', CaBotRclpyUtil.time_zero())
                translation = transformStamped.transform.translation
                rotation = transformStamped.transform.rotation
                euler = tf_transformations.euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
                current_pose = geoutil.Pose(x=translation.x, y=translation.y, r=euler[2])
                return current_pose
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                continue
            rate.sleep()
        raise RuntimeError("no transformation")

    def current_global_pose(self):
        local = self.current_local_pose()
        self._logger.debug(F"current location ({local})")

        _global = geoutil.local2global(local, self._anchor)
        return _global

    def current_location_id(self):
        """get id string for the current loaction in ROS"""

        _global = self.current_global_pose()
        return F"latlng:{_global.lat:.7f}:{_global.lng:.7f}:{self.current_floor}"


class Navigation(ControlBase, navgoal.GoalInterface):
    """Navigation node for Cabot"""

    ACTIONS = {"navigate_to_pose": nav2_msgs.action.NavigateToPose,
               "navigate_through_poses": nav2_msgs.action.NavigateThroughPoses}
    NS = ["", "/local"]

    def __init__(self, node: Node, datautil_instance=None, anchor_file=None, wait_for_action=True):

        self.current_floor = None
        self.current_frame = None

        super(Navigation, self).__init__(
            node, datautil_instance=datautil_instance, anchor_file=anchor_file)

        self.info_pois = []
        self.queue_wait_pois = []
        self.speed_pois = []
        self.turns = []

        self.i_am_ready = False
        self._sub_goals = []
        self._current_goal = None

        # self.client = None
        self._loop_handle = None
        self.pause_control_state = True
        self.pause_control_loop_handler = None
        self.lock = threading.Lock()

        self._max_speed = node.declare_parameter("max_speed", 1.1).value
        self._max_acc = node.declare_parameter("max_acc", 0.3).value

        self._global_map_name = node.declare_parameter("global_map_name", "map").value
        self.visualizer.global_map_name = self._global_map_name

        self.social_navigation = SocialNavigation(node, self.buffer)

        self._clients: dict[str, ActionClient] = {}

        for ns in Navigation.NS:
            for action in Navigation.ACTIONS:
                name = "/".join([ns, action])
                self._clients[name] = ActionClient(self._node, Navigation.ACTIONS[action], name)
                if wait_for_action and not self._clients[name].wait_for_server(timeout_sec=1.0):
                    self._logger.error(F"client for {name} is not ready")

        self._spin_client = ActionClient(self._node, nav2_msgs.action.Spin, "/spin")
        if wait_for_action and not self._spin_client.wait_for_server(timeout_sec=1.0):
            self._logger.error("spin is not ready")

        pause_control_output = node.declare_parameter("pause_control_topic", "/cabot/pause_control").value
        self.pause_control_pub = node.create_publisher(std_msgs.msg.Bool, pause_control_output, 10)
        map_speed_output = node.declare_parameter("map_speed_topic", "/cabot/map_speed").value
        self.speed_limit_pub = node.create_publisher(std_msgs.msg.Float32, map_speed_output, 10)
        current_floor_input = node.declare_parameter("current_floor_topic", "/current_floor").value
        self.current_floor_sub = node.create_subscription(std_msgs.msg.Int64, current_floor_input, self._current_floor_callback, 10)
        current_frame_input = node.declare_parameter("current_frame_topic", "/current_frame").value
        self.current_frame_sub = node.create_subscription(std_msgs.msg.String, current_frame_input, self._current_frame_callback, 10)
        self._localize_status_sub = node.create_subscription(MFLocalizeStatus, "/localize_status", self._localize_status_callback, 10)
        self.localize_status = MFLocalizeStatus.UNKNOWN

        plan_input = node.declare_parameter("plan_topic", "/move_base/NavfnROS/plan").value
        self.plan_sub = node.create_subscription(nav_msgs.msg.Path, plan_input, self._plan_callback, 10)
        path_output = node.declare_parameter("path_topic", "/path").value
        transient_local_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = node.create_publisher(nav_msgs.msg.Path, path_output, transient_local_qos)

        self.updated_goal_sub = node.create_subscription(geometry_msgs.msg.PoseStamped, "/updated_goal", self._goal_updated_callback, 10)

        self.current_queue_msg = None
        self.need_queue_start_arrived_info = False
        self.need_queue_proceed_info = False
        queue_input = node.declare_parameter("queue_topic", "/queue_people_py/queue").value
        self.queue_sub = node.create_subscription(queue_msgs.msg.Queue, queue_input, self._queue_callback, 10)
        queue_speed_output = node.declare_parameter("queue_speed_topic", "/cabot/queue_speed").value
        self.queue_speed_limit_pub = node.create_publisher(std_msgs.msg.Float32, queue_speed_output, 10)
        self._queue_tail_ignore_path_dist = node.declare_parameter("queue_tail_ignore_path_dist", 0.8).value
        self._queue_wait_pass_tolerance = node.declare_parameter("queue_wait_pass_tolerance", 0.3).value
        self._queue_wait_arrive_tolerance = node.declare_parameter("queue_wait_arrive_tolerance", 0.2)
        self._queue_tail_dist_error_tolerance = node.declare_parameter("queue_tail_dist_error_tolerance", 0.5).value
        self._queue_wait_position_offset = node.declare_parameter("queue_wait_position_offset", 0.2).value
        self.initial_queue_interval = node.declare_parameter("initial_queue_interval", 1.0).value
        self.current_queue_interval = self.initial_queue_interval

        self.initial_social_distance = None
        self.current_social_distance = None
        get_social_distance_topic = node.declare_parameter("get_social_distance_topic", "/get_social_distance").value
        self.get_social_distance_sub = node.create_subscription(geometry_msgs.msg.Point, get_social_distance_topic, self._get_social_distance_callback, 10)
        set_social_distance_topic = node.declare_parameter("set_social_distance_topic", "/set_social_distance").value
        self.set_social_distance_pub = node.create_publisher(geometry_msgs.msg.Point, set_social_distance_topic, transient_local_qos)

        self._start_loop()

    def _localize_status_callback(self, msg):
        self.localize_status = msg.status

    def process_event(self, event):
        '''cabot navigation event'''
        # do not provide social navigation messages while queue navigation
        if isinstance(self._current_goal, navgoal.QueueNavGoal):
            return

        self._logger.info(F"{event}")
        if event.param == "elevator_door_may_be_ready":
            self.delegate.elevator_opening()
        elif event.param == "navigation_start":
            self.delegate.start_navigation()
        elif self.social_navigation is not None:
            self.social_navigation.event = event

    # callback functions
    def _current_floor_callback(self, msg):
        prev = self.current_floor
        self.current_floor = msg.data
        if msg.data >= 0:
            self.current_floor = msg.data + 1
        if self.current_floor != prev:
            self.floor_is_changed_at = self._node.get_clock().now()
        self._logger.info(F"Current floor is {self.current_floor}")

    def _current_frame_callback(self, msg):
        if self.current_frame != msg.data:
            self.wait_for_restart_navigation()
        self.current_frame = msg.data
        self._logger.info(F"Current frame is {self.current_frame}")

    @util.setInterval(0.1, times=1)
    def wait_for_restart_navigation(self):
        now = self._node.get_clock().now()
        duration_in_sec = CaBotRclpyUtil.to_sec(now - self.floor_is_changed_at)
        self._logger.innfo(F"wait_for_restart_navigation {duration_in_sec:.2f}")
        if self._current_goal is None:
            return
        if duration_in_sec > 1.0:
            self._stop_loop()
            if self._current_goal:
                self._current_goal.prevent_callback = True
            self.pause_navigation()
            time.sleep(0.5)
            self.resume_navigation()

    def _plan_callback(self, path):
        try:
            self.turns = TurnDetector.detects(path, current_pose=self.current_pose)
            self.visualizer.turns = self.turns
            if self.social_navigation is not None:
                self.social_navigation.path = path

            self._logger.info(F"turns: {self.turns}")
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
        except ValueError:
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

        self._logger.info(F"path-length {path_length(path):.2f}")

    def _queue_callback(self, msg):
        self.current_queue_msg = msg
        names = [person.name for person in self.current_queue_msg.people]
        self._logger.info(F"Current people in queue {names}", throttle_duration_sec=1)

    def _get_social_distance_callback(self, msg):
        self.current_social_distance = msg
        if self.initial_social_distance is None:
            self.initial_social_distance = self.current_social_distance
        self._logger.info(F"Current social distance parameter is {self.current_social_distance}",
                          throttle_duration_sec=3)

    def _get_queue_interval_callback(self, msg):
        self.current_queue_interval = msg.data
        if self.initial_queue_interval is None:
            self.initial_queue_interval = self.current_queue_interval
        self._logger.info(F"Current queue interval parameter is {self.current_queue_interval}",
                          throttle_duration_sec=3)

    def _goal_updated_callback(self, msg):
        if self._current_goal:
            self._current_goal.update_goal(msg)

    # public interfaces

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
        self._logger.info(F"navigation.{util.callee_name()} called")
        try:
            from_id = self.current_location_id()
        except RuntimeError:
            self._logger.error("could not get current location")
            self.delegate.could_not_get_current_location()
            return

        self.delegate.activity_log("cabot/navigation", "from", from_id)
        self.delegate.activity_log("cabot/navigation", "to", destination)

        # specify last orientation
        if destination.find("@") > -1:
            (to_id, yaw_str) = destination.split("@")
            yaw = float(yaw_str)
            groute = self._datautil.get_route(from_id, to_id)
            self._sub_goals = navgoal.make_goals(self, groute, self._anchor, yaw=yaw)
        else:
            to_id = destination
            groute = self._datautil.get_route(from_id, to_id)
            self._sub_goals = navgoal.make_goals(self, groute, self._anchor)

        # navigate from the first path
        self._navigate_next_sub_goal()

    def retry_navigation(self):
        self._logger.info(F"navigation.{util.callee_name()} called")
        self.delegate.activity_log("cabot/navigation", "retry")
        self.turns = []

        if self._current_goal:
            self._sub_goals.insert(0, self._current_goal)
            self._navigate_next_sub_goal()

    def pause_navigation(self):
        self._logger.info(F"navigation.{util.callee_name()} called")
        self.delegate.activity_log("cabot/navigation", "pause")

        for name in self._clients:
            for _, handle in enumerate(self._clients[name]._goal_handles):
                if handle.status == GoalStatus.ACTIVE:
                    handle.cancel_goal()

        self.turns = []

        if self._current_goal:
            self._sub_goals.insert(0, self._current_goal)

    def resume_navigation(self):
        self._logger.info(F"navigation.{util.callee_name()} called")
        self.delegate.activity_log("cabot/navigation", "resume")
        self._navigate_next_sub_goal()

    def cancel_navigation(self):
        """callback for cancel topic"""
        self._logger.info(F"navigation.{util.callee_name()} called")
        self.delegate.activity_log("cabot/navigation", "cancel")
        self.pause_navigation()
        self._current_goal = None
        self._stop_loop()

    # private methods for navigation
    def _navigate_next_sub_goal(self):
        self._logger.info(F"navigation.{util.callee_name()} called")
        self.delegate.activity_log("cabot/navigation", "next_sub_goal")
        if self._sub_goals:
            self._current_goal = self._sub_goals.pop(0)
            self._navigate_sub_goal(self._current_goal)
            return

        if self._current_goal:
            self._stop_loop()
            self.cancel_navigation()

    @util.setInterval(0.01, times=1)
    def _navigate_sub_goal(self, goal):
        self._logger.info(F"navigation.{util.callee_name()} called")
        self.delegate.activity_log("cabot/navigation", "sub_goal")

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

        self._logger.info(F"goal: {goal}")
        try:
            goal.enter()
        except:  # noqa: E722
            import traceback
            self._logger.error(traceback.format_exc())
        self._start_loop()

    def _start_loop(self):
        self._logger.info(F"navigation.{util.callee_name()} called")
        if self.lock.acquire():
            if self._loop_handle is None:
                self._loop_handle = self._check_loop()
            self.lock.release()

    def _stop_loop(self):
        self._logger.info(F"navigation.{util.callee_name()} called")
        if self.lock.acquire():
            if self._loop_handle is not None:
                self._loop_handle.set()
                self._loop_handle = None
            self.lock.release()

    # Main loop of navigation
    GOAL_POSITION_TORELANCE = 1

    @util.setInterval(0.1)
    def _check_loop(self):
        if not rclpy.ok():
            self._stop_loop()
            return

        # need a robot position
        try:
            self.current_pose = self.current_local_pose()
            self.delegate.update_pose(ros_pose=self.current_ros_pose(),
                                      global_position=self.current_global_pose(),
                                      current_floor=self.current_floor,
                                      global_frame=self._global_map_name
                                      )
            self._logger.debug(F"current pose {self.current_pose}", throttle_duration_sec=1)
            self.current_odom_pose = self.current_local_odom_pose()
        except RuntimeError:
            self._logger.info("could not get position", throttle_duration_sec=3)
            return

        # wait data is analyzed
        if not self._datautil.is_analyzed:
            return

        if not self.i_am_ready and \
           self.localize_status != MFLocalizeStatus.TRACKING:
            return

        # say I am ready once
        if not self.i_am_ready:
            self._logger.debug("i am ready")
            self.delegate.i_am_ready()
            self.i_am_ready = True

        if self._current_goal is None:
            return

        # cabot is active now
        self._logger.debug("cabot is active", throttle_duration_sec=1)

        try:
            self._check_info_poi(self.current_pose)
            self._check_speed_limit(self.current_pose)
            self._check_turn(self.current_pose)
            self._check_queue_wait(self.current_pose)
            self._check_social(self.current_pose)
            self._check_goal(self.current_pose)
        except Exception:
            import traceback
            self._logger.error(traceback.format_exc(), throttle_duration_sec=3)

    def _check_info_poi(self, current_pose):
        if not self.info_pois:
            return

        poi = min(self.info_pois, key=lambda p, c=current_pose: p.distance_to(c))

        if poi is not None and poi.distance_to(current_pose) < 8:
            # self._logger.info(F"{poi._id}, {poi.local_geometry}, {current_pose}")
            if poi.is_approaching(current_pose):
                self._logger.info(F"approaching {poi._id}")
                self.delegate.approaching_to_poi(poi=poi)
            elif poi.is_approached(current_pose):
                self._logger.info(F"approached {poi._id}")
                self.delegate.approached_to_poi(poi=poi)
            elif poi.is_passed(current_pose):
                self._logger.info(F"passed {poi._id}")
                self.delegate.passed_poi(poi=poi)

    def _check_speed_limit(self, current_pose):
        # check speed limit
        if not self.speed_pois:
            return

        limit = self._max_speed
        for poi in self.speed_pois:
            dist = poi.distance_to(current_pose)
            if dist < 5.0:
                if poi.in_angle(current_pose):  # and poi.in_angle(c2p):
                    limit = min(limit, max(poi.limit, math.sqrt(2.0 * dist * self._max_acc)))
                else:
                    limit = min(limit, self._max_speed)
                self._logger.debug("speed poi dist=%.2fm, limit=%.2f", dist, limit)
        msg = std_msgs.msg.Float32()
        msg.data = limit
        self.speed_limit_pub.publish(msg)

    def _check_turn(self, current_pose):
        # provide turn tactile notification
        if not self.turns:
            return

        self._logger.info("check turn", throttle_duration_sec=1)
        if self.turns is not None:
            for turn in self.turns:
                try:
                    turn_pose = self.buffer.transform(turn.pose, self._global_map_name)
                    dist = current_pose.distance_to(geoutil.Point(xy=turn_pose.pose.position))
                    if dist < 0.25 and not turn.passed:
                        turn.passed = True
                        self._logger.info(F"notify turn {turn}")

                        self.delegate.notify_turn(turn=turn)

                        if turn.turn_type == Turn.Type.Avoiding:
                            # give avoiding announce
                            self._logger.info("social_navigation avoiding turn")
                            self.social_navigation.turn = turn
                except:  # noqa: E722
                    import traceback
                    self._logger.error(traceback.format_exc())
                    self._logger.error("could not convert pose for checking turn POI",
                                       throttle_duration_sec=3)

    def _check_queue_wait(self, current_pose):
        if not isinstance(self._current_goal, navgoal.QueueNavGoal) or not self.queue_wait_pois:
            return

        # Select queue wait POI which robot did not pass yet.
        # Even if POI is marked as passed, add POI which is closer than _queue_wait_pass_tolerance.
        poi = None
        forward_queue_wait_pois = [x for x in self.queue_wait_pois if geoutil.is_forward_point(current_pose, x) or x.distance_to(current_pose) < self._queue_wait_pass_tolerance]
        if len(forward_queue_wait_pois) > 0:
            poi = min(forward_queue_wait_pois, key=lambda p, c=current_pose: p.distance_to(c))

        # control speed by Queue POI
        limit = self._max_speed
        if poi is not None:
            poi_pose = poi.to_pose_msg()
            poi_position = numpy.array([poi_pose.position.x, poi_pose.position.y])

            current_position = numpy.array([current_pose.x, current_pose.y])
            current_position_on_queue_path = geoutil.get_projected_point_to_line(current_position, poi_position, poi.link_orientation)

            if len(self.current_queue_msg.people) > 0:
                tail_pose = geometry_msgs.msg.PoseStamped()
                tail_pose.header = self.current_queue_msg.header
                tail_pose.pose.position = self.current_queue_msg.people[-1].position
                try:
                    tail_global_pose = self.buffer.transform(tail_pose, self._global_map_name)
                    tail_global_position = numpy.array([tail_global_pose.pose.position.x, tail_global_pose.pose.position.y])
                    tail_global_position_on_queue_path = geoutil.get_projected_point_to_line(tail_global_position, poi_position, poi.link_orientation)
                    dist_robot_to_tail = numpy.linalg.norm(tail_global_position_on_queue_path - current_position_on_queue_path)
                    if dist_robot_to_tail <= self.current_queue_interval:
                        # if distance to queue tail is smaller than queue wait interval, stop immediately
                        limit = 0.0
                        self._logger.info(F"Stop at current position, tail is closer than next queue wait POI, Set speed limit={limit}, dist_robot_to_tail={dist_robot_to_tail}", throttle_duration_sec=1)
                    else:
                        # adjust Queue POI position by moving closer to link target node
                        poi_orientation = tf_transformations.euler_from_quaternion([poi.link_orientation.x, poi.link_orientation.y, poi.link_orientation.z, poi.link_orientation.w])
                        poi_fixed_position = poi_position + numpy.array([self._queue_wait_position_offset*math.cos(poi_orientation[2]), self._queue_wait_position_offset*math.sin(poi_orientation[2])])
                        dist_tail_to_queue_wait = numpy.linalg.norm(tail_global_position_on_queue_path - poi_fixed_position)
                        if dist_tail_to_queue_wait > max(2.0*self.current_queue_interval - self._queue_tail_dist_error_tolerance, self.current_queue_interval):
                            # If distance from next queue wait point to queue tail is larger than twice of queue wait interval,
                            # there should be another queue wait point. Skip next queue wait point.
                            self._logger.info(F"Skip next queue wait POI, POI is far from robot. dist_tail_to_queue_wait={dist_tail_to_queue_wait}, dist_robot_to_tail={dist_robot_to_tail}", throttle_duration_sec=1)
                        else:
                            dist_tail_to_queue_path = numpy.linalg.norm(tail_global_position_on_queue_path - tail_global_position)
                            if (dist_tail_to_queue_path > self._queue_tail_ignore_path_dist):
                                # If distance from queue tail to queue path is larger than queue_tail_ignore_path_dist, robot and queue tail person should be on different queue paths.
                                # Skip next queue wait point.
                                self._logger.info(F"Skip next queue wait POI, tail is far from path. dist_tail_to_queue_path={dist_tail_to_queue_path}", throttle_duration_sec=1)
                            else:
                                # limit speed by using adjusted Queue POI
                                dist_robot_to_queue_wait = numpy.linalg.norm(poi_fixed_position - current_position_on_queue_path)
                                limit = min(limit, math.sqrt(2.0 * max(0.0, dist_robot_to_queue_wait - self._queue_wait_arrive_tolerance) * self._max_acc))
                                self._logger.info(F"Set speed limit={limit}, dist_robot_to_queue_wait={dist_robot_to_queue_wait}", throttle_duration_sec=1)
                except:  # noqa: E722
                    self._logger.error("could not convert pose for checking queue POI", throttle_duration_sec=3)
            else:
                self._logger.info("No queue people", throttle_duration_sec=1)
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
            self.delegate.queue_proceed()
            self.need_queue_proceed_info = False

    def _check_social(self, current_pose):
        if self.social_navigation is None:
            return

        # do not provide social navigation messages while queue navigation
        if isinstance(self._current_goal, navgoal.QueueNavGoal):
            return

        self.social_navigation.current_pose = current_pose
        message = self.social_navigation.get_message()
        if message is not None:
            self.delegate.announce_social(message)

    def _check_goal(self, current_pose):
        self._logger.info(F"navigation.{util.callee_name()} called", throttle_duration_sec=1)
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
            self.delegate.activity_log("cabot/navigation", "navigation", "arrived")
            self.delegate.have_arrived(goal)

        self._navigate_next_sub_goal()

    # GoalInterface

    def enter_goal(self, goal):
        self.delegate.enter_goal(goal)

    def exit_goal(self, goal):
        self.delegate.exit_goal(goal)

    def announce_social(self, messages):
        self.delegate.announce_social(messages)

    def navigate_to_pose(self, goal_pose, behavior_tree, done_cb, namespace=""):
        self._logger.info(F"{namespace}/navigate_to_pose")
        self.delegate.activity_log("cabot/navigation", "navigate_to_pose")
        client = self._clients["/".join([namespace, "navigate_to_pose"])]
        goal = nav2_msgs.action.NavigateToPose.Goal()

        if behavior_tree.startswith("package://"):
            start = len("package://")
            end = behavior_tree.find("/", len("pacakge://"))
            package = behavior_tree[start:end]
            behavior_tree = get_package_share_directory(package) + behavior_tree[end:]
            self._logger.info(F"package={package}, behavior_tree={behavior_tree}")

        goal.behavior_tree = behavior_tree

        if namespace == "":
            goal.pose = self.buffer.transform(goal_pose, "map")
            goal.pose.header.stamp = self._node.get_clock().now().to_msg()
            goal.pose.header.frame_id = "map"
            client.send_goal_async(goal, done_cb)
        elif namespace == "/local":
            goal.pose = goal_pose
            goal.pose.header.stamp = self._node.get_clock().now().to_msg()
            goal.pose.header.frame_id = "local/odom"
            client.send_goal_async(goal, done_cb)
        else:
            self._logger.info(F"unknown namespace {namespace}")

        self.visualizer.reset()
        self.visualizer.goal = goal
        self.visualizer.visualize()
        self._logger.info(F"sent goal: {goal}")
        # need to move into the BT
        # self.delegate.start_navigation(self.current_pose)

    def navigate_through_poses(self, goal_poses, behavior_tree, done_cb, namespace=""):
        self._logger.info(F"{namespace}/navigate_through_poses")
        self.delegate.activity_log("cabot/navigation", "navigate_through_pose")
        client = self._clients["/".join([namespace, "navigate_through_poses"])]
        goal = nav2_msgs.action.NavigateThroughPoses.Goal()

        if behavior_tree.startswith("package://"):
            start = len("package://")
            end = behavior_tree.find("/", len("pacakge://"))
            package = behavior_tree[start:end]
            behavior_tree = get_package_share_directory(package) + behavior_tree[end:]
            self._logger.info(F"package={package}, behavior_tree={behavior_tree}")

        goal.behavior_tree = behavior_tree

        if namespace == "":
            goal.poses = []
            for pose in goal_poses:
                t_pose = self.buffer.transform(pose, "map")
                t_pose.pose.position.z = 0
                t_pose.header.stamp = self._node.get_clock().now().to_msg()
                t_pose.header.frame_id = "map"
                goal.poses.append(t_pose)
            client.send_goal(goal, done_cb)
        elif namespace == "local":
            goal.poses = []
            for pose in goal_poses:
                t_pose = pose
                t_pose.header.stamp = self._node.get_clock().now().to_msg()
                t_pose.header.frame_id = "local/odom"
                goal.poses.append(t_pose)
            client.send_goal(goal, done_cb)
        else:
            self._logger.info(F"unknown namespace {namespace}")

        self.visualizer.reset()
        self.visualizer.goal = goal
        self.visualizer.visualize()
        self._logger.info(F"sent goal {goal}")
        # need to move into the BT
        # self.delegate.start_navigation(self.current_pose)

    def turn_towards(self, orientation, callback, clockwise=0):
        self._logger.info("turn_towards")
        self.delegate.activity_log("cabot/navigation", "turn_towards",
                                   str(geoutil.get_yaw(geoutil.q_from_msg(orientation))))
        self._turn_towards(orientation, callback, clockwise=clockwise)

    @util.setInterval(0.01, times=1)
    def _turn_towards(self, orientation, callback, clockwise=0):
        goal = nav2_msgs.action.SpinGoal()
        diff = geoutil.diff_angle(self.current_pose.orientation, orientation)

        self._logger.info(F"current pose {self.current_pose}, diff {diff:.2f}")

        if abs(diff) > 0.05:
            if (clockwise < 0 and diff < - math.pi / 4) or \
               (clockwise > 0 and diff > + math.pi / 4):
                diff = diff - clockwise * math.pi * 2
            # use orientation.y for target spin angle
            self._logger.info(F"send turn {diff:.2f}")
            # only use y for yaw
            turn_yaw = diff - (diff / abs(diff) * 0.05)
            goal.target_yaw = turn_yaw
            self._spin_client.send_goal(goal, lambda x, y: self._turn_towards(orientation, callback, clockwise=clockwise))
            self._logger.info(F"sent goal {goal}")

            # add position and use quaternion to visualize
            # self.visualizer.goal = goal
            # self.visualizer.visualize()
            # self._logger.info(F"visualize goal {goal}")
            self.delegate.notify_turn(turn=Turn(self.current_pose.to_pose_stamped_msg(self._global_map_name), turn_yaw))
            self._logger.info(F"notify turn {turn_yaw}")
        else:
            self._logger.info(F"turn completed {diff}")
            callback(GoalStatus.SUCCEEDED, None)

    def goto_floor(self, floor, callback):
        self._goto_floor(floor, callback)

    @util.setInterval(0.01, times=1)
    def _goto_floor(self, floor, callback):
        self._logger.info(F"go to floor {floor}")
        self.delegate.activity_log("cabot/navigation", "go_to_floor", str(floor))
        rate = self._node.create_rate(2)
        while self.current_floor != floor:
            rate.sleep()

        self._logger.info("floor is changed")
        self.delegate.floor_changed(self.current_floor)
        while rclpy.ok():
            self._logger.info(F"trying to find tf from map to {self.current_frame}")
            try:
                rate.sleep()
                self.buffer.lookup_transform("map", self.current_frame, CaBotRclpyUtil.time_zero())
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                self._logger.warn(F"Could not find tf from map to {self.current_frame}")
                continue

        self._logger.info("tf is changed")
        rate.sleep()

        callback(GoalStatus.SUCCEEDED, None)

    def set_pause_control(self, flag):
        self.pause_control_state = flag
        self.pause_control_pub.publish(self.pause_control_state)
        if self.pause_control_loop_handler is None:
            self.pause_control_loop_handler = self.pause_control_loop()

    @util.setInterval(1.0)
    def pause_control_loop(self):
        self.pause_control_pub.publish(self.pause_control_state)

    def publish_path(self, global_path, convert=True):
        local_path = global_path
        if convert:
            local_path = nav_msgs.msg.Path()
            local_path.header = global_path.header

            for pose in global_path.poses:
                local_path.poses.append(self.buffer.transform(pose, "map"))
                local_path.poses[-1].pose.position.z = 0.0

        self.path_pub.publish(local_path)

    def please_call_elevator(self, pos):
        self.delegate.please_call_elevator(pos)

    def please_pass_door(self):
        self.delegate.please_pass_door()

    def door_passed(self):
        self.delegate.door_passed()

    def global_map_name(self):
        return self._global_map_name

    def please_follow_behind(self):
        self.delegate.please_follow_behind()

    def please_return_position(self):
        self.delegate.please_return_position()
