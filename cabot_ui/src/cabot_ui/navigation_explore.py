# Copyright (c) 2023  Carnegie Mellon University, IBM Corporation, and others
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

import copy
import inspect
import itertools
import json
import math
import sys
import threading
import time

import actionlib
from actionlib_msgs.msg import GoalStatus
import cv2
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from nav_msgs.msg import MapMetaData
import nav2_msgs.msg
import numpy as np
import rospy
from scipy.spatial import ConvexHull
import std_msgs.msg
import tf
import tf2_ros

from cabot import util
from cabot_ui import visualizer_explore, geoutil
from cabot_ui.turn_detector import Turn
from route_explore_utils import costmap_utils, geometry_utils


# used for check if detected turn is already notified using contours
class NotifiedTurnArea:
    def __init__(self, robot_pose, contour):
        self.robot_pose = robot_pose
        self.contour = contour


# used for optional check if detected turn is already notified using directions
class NotifiedTurnGoal:
    def __init__(self, robot_pose, goal_pose):
        self.robot_pose = robot_pose
        self.goal_pose = goal_pose


class NavigationExploreInterface(object):
    def i_am_ready(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def start_explore(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def start_explore_forward(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def finish_explore_forward(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def go_origin(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def arrive_origin(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def system_cancel_explore(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def user_cancel_explore(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def find_forward_right_left_route(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def find_forward_right_route(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def find_forward_left_route(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def find_right_left_route(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def find_right_route(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def find_left_route(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def find_dead_end(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def move_forward_route(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def turn_right_route(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def turn_left_route(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def cannot_move_forward_route(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def cannot_turn_right_route(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def cannot_turn_left_route(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def could_not_get_current_location(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def cannot_turn_route(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))

    def find_intersection_routes_in_clock_direction(self,paths):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))
    
    def error(self):
        rospy.logerr("{} is not implemented".format(inspect.currentframe().f_code.co_name))


class ControlExploreBase(object):

    def __init__(self):
        self.visualizer_explore = visualizer_explore.instance

        self.delegate = NavigationExploreInterface()
        self.listener = tf.TransformListener()
        self.current_pose = None

    # current location

    def current_2D_pose(self, frame=None):
        """get current local location"""
        if frame is None:
            frame = self._global_map_name
        rate = rospy.Rate(10.0)
        trans = rotation = None
        for i in range(0, 10):
            try:
                (trans, rotation) = self.listener.lookupTransform(frame, '/base_footprint', rospy.Time())
                ros_pose = Vector3()
                ros_pose.x = trans[0]
                ros_pose.y = trans[1]
                ros_pose.z = geoutil.get_yaw(rotation)
                return ros_pose
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()
        raise RuntimeError("no transformation")

        
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


class NavigationExplore(ControlExploreBase):
    """NavigationExplore node for Cabot"""

    def __init__(self):
        super(NavigationExplore, self).__init__()

        # start of constant parameters
        self._lookup_transform_duration = 1.0

        self._ignore_small_area_square_meters = 1.0
        self._notified_turn_visible_area_inflate_radius = 3.0

        self._min_abs_yaw_backward_goal = math.pi*3.0/4.0 # if absolute yaw direction from robot to goal candidate is larger than this value, that goal is considered as backward goal

        self._max_abs_yaw_read_goal = math.pi*3.0/4.0 # if absolute yaw direction from robot to goal candidate is larger than this value, the goal will not be read to the user
        self._distance_keep_missing_notified_turn_areas = 3.0 # if notified turn area become invisible, keep the turn area while robot does not move this distance value
        # end of constant parameters

        self.i_am_ready = False

        self._lock_last_explore_status = threading.Lock()
        self._last_explore_status = self.init_explore_status()
        # variables for check if detected turn is already notified using contours
        self._visible_notified_turn_areas = []
        self._missing_notified_turn_areas = []
        # variable for optinal check if detected turn is already notified using directions
        self._notified_turn_goals = []
        # variables for calculating confidence if contours are routes for test
        self._visible_notified_turn_areas_attributes = []
        self._visible_notified_turn_areas_route_confidence = []

        self._loop_handle = None

        self._max_speed = rospy.get_param("~max_speed", 1.1)
        self._max_acc = rospy.get_param("~max_acc", 0.3)

        self._global_map_name = rospy.get_param("~global_map_name", "map")
        self.visualizer_explore.global_map_name = self._global_map_name

        self._use_skip_notify_similar_turn_directions = rospy.get_param("~use_skip_notify_similar_turn_directions", True) # use optional check if detected turn is already notified using directions
        self._distance_keep_similar_turn_directions = rospy.get_param("~distance_keep_similar_turn_directions", 3.0) # to suppress notification for similar turn directions, keep notified turn goal while robot does not move this distance value
        self._max_abs_yaw_diff_similar_turn_directions = rospy.get_param("~max_abs_yaw_diff_similar_turn_directions", math.pi/4.0) # maximum angle of suppressing notification for similar turn directions

        self._use_skip_notify_backward_directions = rospy.get_param("~use_skip_notify_backward_directions", True) # use optional check to skip detected goal in backward directions
        self._max_abs_yaw_diff_skip_backward_directions = rospy.get_param("~max_abs_yaw_diff_skip_backward_directions", math.pi/4.0) # maximum angle to skip detected goal between backward direction and goal direction

        self._use_skip_notify_short_route = rospy.get_param("~use_skip_notify_short_route", True) # use optional check to skip detected goal in short distance (Not used when forward goal does not exist because route looks short when robot stops in front of wall)
        self._max_skip_short_route_distance = rospy.get_param("~max_skip_short_route_distance", 5.0) # maximum distance to skip notify goal in short distance

        self._pausing_at_detected_turn = False
        self._finishing_explore_forward = False

        self.should_read_intersection_path_in_clock_directions = rospy.get_param("~should_read_intersection_path_in_clock_direction", True)

        self._tf2_buffer = tf2_ros.Buffer()
        self._tf2_listener = tf2_ros.TransformListener(self._tf2_buffer)

        self._spin_client = actionlib.SimpleActionClient("/spin", nav2_msgs.msg.SpinAction)
        if not self._spin_client.wait_for_server(timeout = rospy.Duration(2.0)):
            rospy.logerr("spin is not ready")

        self._cabot_explore_event_pub = rospy.Publisher("/cabot_explore/explore_event", std_msgs.msg.String, queue_size=1)
        self._cabot_explore_status_sub = rospy.Subscriber("/cabot_explore/explore_status", std_msgs.msg.String, self.explore_status_callback, queue_size=1)
        self._pose_pub = rospy.Publisher("/cabot/pose", Vector3, queue_size=1)
        self._turn_pub = rospy.Publisher("/cabot/turn", std_msgs.msg.Float64, queue_size=1)
        self._arrived_intersection_pose_pub = rospy.Publisher("/cabot/arrived_intersection_pose", Vector3, queue_size=1)
        self._cabot_explore_ui_status_pub = rospy.Publisher("/cabot_explore/explore_ui_status", std_msgs.msg.String, queue_size=1)

        self._start_loop()

    def init_explore_status(self):
        explore_status = {
                        "status": "wait",
                        "goal_candidate_poses": [],
                        "has_forward_goal": False,
                        "has_backward_goal": False,
                        "has_right_goal": False,
                        "has_left_goal": False
                        }
        return explore_status

    # publish explore UI status for debug purpose
    def _publish_explore_ui_status(self, explore_status):
        json_data = {}
        json_data["status"] = explore_status["status"]
        json_data["goal_candidate_poses"] = [{"position":{"x":pose.position.x, "y":pose.position.y}, "orientation":{"x":pose.orientation.x, "y":pose.orientation.y, "z":pose.orientation.z, "w":pose.orientation.w}} \
                                                for pose in explore_status["goal_candidate_poses"]]
        json_data["has_forward_goal"] = explore_status["has_forward_goal"]
        json_data["has_backward_goal"] = explore_status["has_backward_goal"]
        json_data["has_right_goal"] = explore_status["has_right_goal"]
        json_data["has_left_goal"] = explore_status["has_left_goal"]
        self._cabot_explore_ui_status_pub.publish(json.dumps(json_data))

    def process_event(self, event):
        '''cabot navigation event'''

    def calculate_path_angles(self, goal_candidate_poses, has_forward_goal, backward_goal_idx):
        path_angles = []
        pose_msg = self.current_pose.to_pose_msg()
        pose_quat = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]

        for idx, goal_candidate_pose in enumerate(goal_candidate_poses):
            if has_forward_goal and idx==0:
                path_angle_str = "EXPLORE_FORWARD"
            elif backward_goal_idx is not None and idx==backward_goal_idx:
                # skip to read backward goal
                continue
            else:
                pose_to_goal_quat = geometry_utils.calc_point_direction_quaternion(pose_msg.position, goal_candidate_pose.position)
                pose_to_goal_quat_diff = geometry_utils.calc_relative_orientation_quaternion(pose_to_goal_quat, pose_quat)
                pose_to_goal_yaw_diff = tf.transformations.euler_from_quaternion(pose_to_goal_quat_diff)[2]
                if pose_to_goal_yaw_diff<0:
                    pose_to_goal_yaw_diff += math.pi*2.0
                pose_to_goal_yaw_diff_degree = pose_to_goal_yaw_diff*180.0/math.pi
                round_time = round(pose_to_goal_yaw_diff_degree*12.0/360.0)
                if round_time==0 or round_time==12:
                    # if rounded int value becomes 0 or 12 for non-forward direction, select closer value from 1 or 11
                    if math.sin(pose_to_goal_yaw_diff)>=0:
                        round_time = 1
                    else:
                        round_time = 11
                path_angle_str = str(round_time) + "_O_CLOCK"
            path_angles.append(path_angle_str)

        return path_angles

    def _visualize(self, coverage_polyhedron, pose_to_turn, goal_candidate_poses, visible_notified_turn_areas, missing_notified_turn_areas, visible_notified_turn_areas_attributes, visible_notified_turn_areas_route_confidence):
        self.visualizer_explore.reset()
        self.visualizer_explore.coverage_polyhedron = coverage_polyhedron
        self.visualizer_explore.pose_to_turn = pose_to_turn
        self.visualizer_explore.goal_candidate_poses = goal_candidate_poses
        self.visualizer_explore.visible_notified_turn_areas = visible_notified_turn_areas
        self.visualizer_explore.missing_notified_turn_areas = missing_notified_turn_areas
        self.visualizer_explore.visible_notified_turn_areas_attributes = visible_notified_turn_areas_attributes
        self.visualizer_explore.visible_notified_turn_areas_route_confidence = visible_notified_turn_areas_route_confidence
        self.visualizer_explore.visualize()

    def explore_status_callback(self, msg):
        if self.current_pose is None:
            self._visualize(None, None, None, None, None, None, None)
            return

        current_pose_msg = self.current_pose.to_pose_msg()

        json_data = json.loads(msg.data)
        # rospy.loginfo("Subscribe explore_status : " + json.dumps(json_data))

        # update last explore status
        explore_status = self.init_explore_status()
        explore_status["status"] = json_data["status"]

        if "goal_candidate_poses" in json_data:
            explore_status["goal_candidate_poses"] = [Pose(Point(goal_candidate["position"]["x"], goal_candidate["position"]["y"], 0.0), \
                                                        Quaternion(goal_candidate["orientation"]["x"], goal_candidate["orientation"]["y"], \
                                                                goal_candidate["orientation"]["z"], goal_candidate["orientation"]["w"])) \
                                                                    for goal_candidate in json_data["goal_candidate_poses"]]
        else:
            explore_status["goal_candidate_poses"] = []

        if "has_forward_goal" in json_data:
            explore_status["has_forward_goal"] = json_data["has_forward_goal"]
        else:
            explore_status["has_forward_goal"] = False

        robot_goal_candidate_poses_yaw_diff = []
        current_pose_msg_quat = (current_pose_msg.orientation.x, current_pose_msg.orientation.y, current_pose_msg.orientation.z, current_pose_msg.orientation.w)
        for goal_candidate_pose in explore_status["goal_candidate_poses"]:
            goal_quat = geometry_utils.calc_point_direction_quaternion(current_pose_msg.position, goal_candidate_pose.position)
            robot_goal_quat_diff = geometry_utils.calc_relative_orientation_quaternion(current_pose_msg_quat, goal_quat)
            robot_goal_yaw_diff = tf.transformations.euler_from_quaternion(robot_goal_quat_diff)[2]
            robot_goal_candidate_poses_yaw_diff.append(robot_goal_yaw_diff)

        # find backward goal index from goal candidates
        backward_goal_idx = None
        backward_goal_pose = None
        max_backward_abs_yaw_goal = sys.float_info.min
        for goal_idx, goal_candidate_pose in enumerate(explore_status["goal_candidate_poses"]):
            abs_robot_goal_yaw_diff = abs(robot_goal_candidate_poses_yaw_diff[goal_idx])
            if abs_robot_goal_yaw_diff>self._min_abs_yaw_backward_goal and abs_robot_goal_yaw_diff>max_backward_abs_yaw_goal:
                backward_goal_idx = goal_idx
                backward_goal_pose = explore_status["goal_candidate_poses"][backward_goal_idx]
                max_backward_abs_yaw_goal = abs_robot_goal_yaw_diff

        # use optional check to skip notify goals which is not backward goal but in backward directions
        if self._use_skip_notify_backward_directions:
            goal_candidate_poses_selector = [True] * len(explore_status["goal_candidate_poses"])
            for goal_idx, goal_candidate_pose in enumerate(explore_status["goal_candidate_poses"]):
                if goal_idx==backward_goal_idx:
                    continue

                abs_robot_backward_goal_yaw_diff = math.pi - abs(robot_goal_candidate_poses_yaw_diff[goal_idx])
                if abs_robot_backward_goal_yaw_diff<self._max_abs_yaw_diff_skip_backward_directions:
                    rospy.loginfo("Skip notify backward goal in direction " + str(robot_goal_candidate_poses_yaw_diff[goal_idx]))
                    goal_candidate_poses_selector[goal_idx] = False
            explore_status["goal_candidate_poses"] = list(itertools.compress(explore_status["goal_candidate_poses"], goal_candidate_poses_selector))

            if (backward_goal_idx is not None) and (backward_goal_pose is not None):
                backward_goal_idx = explore_status["goal_candidate_poses"].index(backward_goal_pose)
        # use optional check to skip notify goals which is in short distance and neither in forward or backward directions
        # if forward goal exists, do not ignore short route because route looks short when robot stops in front of wall
        if self._use_skip_notify_short_route and explore_status["has_forward_goal"]:
            goal_candidate_poses_selector = [True] * len(explore_status["goal_candidate_poses"])
            for goal_idx, goal_candidate_pose in enumerate(explore_status["goal_candidate_poses"]):
                if (explore_status["has_forward_goal"] and goal_idx==0) or goal_idx==backward_goal_idx:
                    continue

                robot_goal_distance = geometry_utils.calc_point_distance(current_pose_msg.position, goal_candidate_pose.position)
                if robot_goal_distance<self._max_skip_short_route_distance:
                    rospy.loginfo("Skip notify goal in short distance " + str(robot_goal_distance))
                    goal_candidate_poses_selector[goal_idx] = False
            explore_status["goal_candidate_poses"] = list(itertools.compress(explore_status["goal_candidate_poses"], goal_candidate_poses_selector))

            if (backward_goal_idx is not None) and (backward_goal_pose is not None):
                backward_goal_idx = explore_status["goal_candidate_poses"].index(backward_goal_pose)

        # get pose to turn
        if ("pose_to_turn" in json_data) and ("position" in json_data["pose_to_turn"]) and ("orientation" in json_data["pose_to_turn"]):
            pose_to_turn = Pose()
            pose_to_turn.position.x = json_data["pose_to_turn"]["position"]["x"]
            pose_to_turn.position.y = json_data["pose_to_turn"]["position"]["y"]
            pose_to_turn.position.z = 0.0
            pose_to_turn.orientation.x = json_data["pose_to_turn"]["orientation"]["x"]
            pose_to_turn.orientation.y = json_data["pose_to_turn"]["orientation"]["y"]
            pose_to_turn.orientation.z = json_data["pose_to_turn"]["orientation"]["z"]
            pose_to_turn.orientation.w = json_data["pose_to_turn"]["orientation"]["w"]
        else:
            pose_to_turn = None
        # get costmap info
        if ("costmap_info" in json_data) and ("resolution" in json_data["costmap_info"]) and ("width" in json_data["costmap_info"]) and ("height" in json_data["costmap_info"]) and ("origin" in json_data["costmap_info"]):
            costmap_grid_info = MapMetaData()
            costmap_grid_info.resolution = json_data["costmap_info"]["resolution"]
            costmap_grid_info.width = json_data["costmap_info"]["width"]
            costmap_grid_info.height = json_data["costmap_info"]["height"]
            costmap_grid_info.origin = Pose()
            costmap_grid_info.origin.position.x = json_data["costmap_info"]["origin"]["position"]["x"]
            costmap_grid_info.origin.position.y = json_data["costmap_info"]["origin"]["position"]["y"]
            costmap_grid_info.origin.position.z = 0.0
            costmap_grid_info.origin.orientation.x = json_data["costmap_info"]["origin"]["orientation"]["x"]
            costmap_grid_info.origin.orientation.y = json_data["costmap_info"]["origin"]["orientation"]["y"]
            costmap_grid_info.origin.orientation.z = json_data["costmap_info"]["origin"]["orientation"]["z"]
            costmap_grid_info.origin.orientation.w = json_data["costmap_info"]["origin"]["orientation"]["w"]
            costmap_info = costmap_utils.CostmapInfo(costmap_grid_info)
        else:
            costmap_info = None
        # get reachable area contours
        if ("reachable_area_contours" in json_data) and (json_data["reachable_area_contours"] is not None) and (len(json_data["reachable_area_contours"])>=0):
            reachable_area_contours = [np.array(contour) for contour in json_data["reachable_area_contours"]]
        else:
            reachable_area_contours = []
        # get reachable area hierarchy
        if ("reachable_area_hierarchy" in json_data) and (json_data["reachable_area_hierarchy"] is not None) and (len(json_data["reachable_area_hierarchy"])>=0):
            reachable_area_hierarchy = np.array(json_data["reachable_area_hierarchy"])
        else:
            reachable_area_hierarchy = np.array([])
        # get visible area contour
        if ("visible_area_contour" in json_data) and (json_data["visible_area_contour"] is not None) and (len(json_data["visible_area_contour"])>=0):
            visible_area_contour = np.array(json_data["visible_area_contour"])
        else:
            visible_area_contour = np.array([])
        # get coverage polyhedron
        if ("polyhedron_points" in json_data) and (json_data["polyhedron_points"] is not None) and (len(json_data["polyhedron_points"])>=0):
            coverage_polyhedron = ConvexHull(np.array(json_data["polyhedron_points"]))
        else:
            coverage_polyhedron = None

        with self._lock_last_explore_status:
            need_notify_new_turn, self._visible_notified_turn_areas, self._missing_notified_turn_areas, self._notified_turn_goals, has_new_goal_direction = self._update_notified_turn_areas(costmap_info, current_pose_msg, reachable_area_contours,
                                                                                                                                        reachable_area_hierarchy, visible_area_contour, coverage_polyhedron, explore_status["goal_candidate_poses"],
                                                                                                                                        explore_status["has_forward_goal"], backward_goal_idx, self._visible_notified_turn_areas,
                                                                                                                                        self._missing_notified_turn_areas, self._notified_turn_goals, notify_new_turn=(explore_status["status"]=="wait"))

            # calculate turn area attributes and route confidence for test
            self._visible_notified_turn_areas_attributes = geometry_utils.calc_contours_attributes(costmap_info, [turn_area.contour for turn_area in self._visible_notified_turn_areas])
            self._visible_notified_turn_areas_route_confidence = geometry_utils.calc_heuristic_route_contour_confidence(self._visible_notified_turn_areas_attributes)
            # rospy.loginfo("have_new_turn : " + str(have_new_turn))

            if backward_goal_idx is not None:
                explore_status["has_backward_goal"] = True
            else:
                explore_status["has_backward_goal"] = False

            # update last explore status for has_right_goal, has_left_goal from goal_candidate_poses
            right_goal_idx = None
            min_right_abs_yaw_goal = sys.float_info.max
            left_goal_idx = None
            min_left_abs_yaw_goal = sys.float_info.max
            for goal_idx, goal_candidate_pose in enumerate(explore_status["goal_candidate_poses"]):
                if (explore_status["has_forward_goal"] and goal_idx==0) or goal_idx==backward_goal_idx:
                    continue
                goal_quat = geometry_utils.calc_point_direction_quaternion(current_pose_msg.position, goal_candidate_pose.position)
                robot_goal_quat_diff = geometry_utils.calc_relative_orientation_quaternion(current_pose_msg_quat, goal_quat)
                robot_goal_yaw_diff = tf.transformations.euler_from_quaternion(robot_goal_quat_diff)[2]
                abs_robot_goal_yaw_diff = abs(robot_goal_yaw_diff)

                if robot_goal_yaw_diff<0 and abs_robot_goal_yaw_diff<min_right_abs_yaw_goal:
                    right_goal_idx = goal_idx
                    min_right_abs_yaw_goal = abs_robot_goal_yaw_diff

                if robot_goal_yaw_diff>0 and abs_robot_goal_yaw_diff<min_left_abs_yaw_goal:
                    left_goal_idx = goal_idx
                    min_left_abs_yaw_goal = abs_robot_goal_yaw_diff

            if right_goal_idx is not None:
                explore_status["has_right_goal"] = True
            else:
                explore_status["has_right_goal"] = False

            if left_goal_idx is not None:
                explore_status["has_left_goal"] = True
            else:
                explore_status["has_left_goal"] = False

            # handle updated last explore status
            if json_data["status"]=="explore":
                if need_notify_new_turn and (explore_status["has_right_goal"] or explore_status["has_left_goal"]):
                    # call cancel explore if new turn is observed
                    self._pausing_at_detected_turn = True
                    self.system_cancel_explore()
                elif not explore_status["has_forward_goal"] and not explore_status["has_right_goal"] and not explore_status["has_left_goal"] and explore_status["has_backward_goal"]:
                    # call cancel explore if dead end is found
                    self._pausing_at_detected_turn = True
                    self.system_cancel_explore()
            elif json_data["status"]=="explore_forward":
                if not explore_status["has_forward_goal"]:
                    # call cancel explore if forward goal is not found
                    self._pausing_at_detected_turn = True
                    self._finishing_explore_forward = True
                    self.system_cancel_explore()
            elif json_data["status"]=="wait" and self._last_explore_status["status"]!="turning":
                if (not has_new_goal_direction) and explore_status["has_forward_goal"] and self._pausing_at_detected_turn:
                    rospy.logwarn("Skip notify. Robot stopped when new turns are found, but all turns are in already notified directions.")
                    explore_status["status"] = "explore"
                    self.start_explore()
                elif self.should_read_intersection_path_in_clock_directions:
                    self.publish_arrived_intersection_pose()
                    
                    paths = self.calculate_path_angles(explore_status["goal_candidate_poses"], explore_status["has_forward_goal"], backward_goal_idx)
                    rospy.loginfo("paths found "+ str(paths))
                    if (len(paths) > 0) and (explore_status["has_right_goal"] or explore_status["has_left_goal"]):
                        if self._finishing_explore_forward:
                            self.delegate.finish_explore_forward()

                        self.delegate.find_intersection_routes_in_clock_direction(paths)
                    elif not explore_status["has_forward_goal"] and not explore_status["has_right_goal"] and not explore_status["has_left_goal"] and explore_status["has_backward_goal"]:
                        if self._finishing_explore_forward:
                            self.delegate.finish_explore_forward()

                        self.delegate.find_dead_end()
                    elif explore_status["has_forward_goal"] and not explore_status["has_right_goal"] and not explore_status["has_left_goal"] and self._pausing_at_detected_turn:
                        rospy.logwarn("This line is not expected to call. Robot stopped when new turn is found, but turn is not visible now.")
                        explore_status["status"] = "explore"
                        self.start_explore()
                else:
                    self.publish_arrived_intersection_pose()

                    if explore_status["has_forward_goal"] and explore_status["has_right_goal"] and explore_status["has_left_goal"]:
                        if self._finishing_explore_forward:
                            self.delegate.finish_explore_forward()

                        self.delegate.find_forward_right_left_route()
                    elif explore_status["has_forward_goal"] and explore_status["has_right_goal"]:
                        if self._finishing_explore_forward:
                            self.delegate.finish_explore_forward()

                        self.delegate.find_forward_right_route()
                    elif explore_status["has_forward_goal"] and explore_status["has_left_goal"]:
                        if self._finishing_explore_forward:
                            self.delegate.finish_explore_forward()

                        self.delegate.find_forward_left_route()
                    elif explore_status["has_right_goal"] and explore_status["has_left_goal"]:
                        if self._finishing_explore_forward:
                            self.delegate.finish_explore_forward()

                        self.delegate.find_right_left_route()
                    elif explore_status["has_right_goal"]:
                        if self._finishing_explore_forward:
                            self.delegate.finish_explore_forward()

                        self.delegate.find_right_route()
                    elif explore_status["has_left_goal"]:
                        if self._finishing_explore_forward:
                            self.delegate.finish_explore_forward()

                        self.delegate.find_left_route()
                    elif not explore_status["has_forward_goal"] and not explore_status["has_right_goal"] and not explore_status["has_left_goal"] and explore_status["has_backward_goal"]:
                        if self._finishing_explore_forward:
                            self.delegate.finish_explore_forward()

                        self.delegate.find_dead_end()
                    elif explore_status["has_forward_goal"] and not explore_status["has_right_goal"] and not explore_status["has_left_goal"] and self._pausing_at_detected_turn:
                        rospy.logwarn("This line is not expected to call. Robot stopped when new turn is found, but turn is not visible now.")
                        explore_status["status"] = "explore"
                        self.start_explore()

                self._pausing_at_detected_turn = False
                self._finishing_explore_forward = False
            elif json_data["status"]=="arrive_origin":
                self.delegate.arrive_origin()

            self._last_explore_status = copy.copy(explore_status)

            self._publish_explore_ui_status(explore_status)
            self._visualize(coverage_polyhedron, pose_to_turn, explore_status["goal_candidate_poses"], self._visible_notified_turn_areas, self._missing_notified_turn_areas, self._visible_notified_turn_areas_attributes, \
                            self._visible_notified_turn_areas_route_confidence)


    def _update_notified_turn_areas(self, costmap_info, robot_pose, reachable_area_contours, reachable_area_hierarchy, visible_area_contour, coverage_polyhedron, goal_candidate_poses, has_forward_goal,
                                    backward_goal_idx, visible_notified_turn_areas, missing_notified_turn_areas, notified_turn_goals, notify_new_turn=False):
        need_notify_new_turn = False
        has_new_goal_direction = False

        # check if costmap info and coverage polyhedron exist
        if (costmap_info is None) or (coverage_polyhedron is None):
            return need_notify_new_turn, [], [], [], has_new_goal_direction

        start_time = time.perf_counter()

        # step 1 : calculate IDs of visible outside coverage polyhedron areas

        # calculate binary image of approximated reachable and visible area which is used for finding goal candidates
        # Note that all visible area should be reachable, but because reachable area is approximated contour,
        # bitwise image of reachable and visible area is used to remove visible and non-reachable area
        visible_area_contours = [visible_area_contour] if (visible_area_contour is not None) and (len(visible_area_contour)>0) else []
        binary_visible_area_contour_image = costmap_utils.calc_binary_contours_image(costmap_info, visible_area_contours)
        binary_reachable_area_contour_image = costmap_utils.calc_binary_hierarchy_contours_image(costmap_info, reachable_area_contours, reachable_area_hierarchy)
        binary_reachable_visible_area_contour_image = cv2.bitwise_and(binary_visible_area_contour_image, binary_reachable_area_contour_image)

        binary_coverage_polyhedron_image = costmap_utils.calc_binary_polyhedron_image(costmap_info, coverage_polyhedron)
        binary_coverage_polyhedron_map = costmap_utils.Costmap(costmap_info, tuple(binary_coverage_polyhedron_image.flatten().astype(np.uint8)))

        visible_contours_outside_polyhedron, visible_contours_outside_polyhedron_id_map = geometry_utils.calc_contours_outside_coverage_polyhedron(binary_coverage_polyhedron_map,
                                                                                                                                                binary_reachable_visible_area_contour_image,
                                                                                                                                                ignore_small_area_square_meters=self._ignore_small_area_square_meters)
        visible_contours_outside_polyhedron_id_image = np.array(visible_contours_outside_polyhedron_id_map.occupancy_grid_data, dtype=np.float32).reshape(costmap_info.height, costmap_info.width)

        # step 2 : remove existing notified turn areas if either one of following conditions are met
        # 1. remove all notified turn areas when forward goal does not exist and any goal exist in right/left directions
        #    (for removing right/left area which was part of forward area before entering T-turn but becomes right/left area after entering T-turn)
        # 2. remove notified turn areas which are not visible from robot (for removing unnecessary area and handling looped area)
        # 3. notified turn areas which will be separated to multiple areas by coverate polyhedron (for removing right/left area which were connected with forward area)
        # 4. when not notifying new turn, remove notified turn areas which do not overwrap with right/left goal areas (for removing forward/backward goal areas which are added when notifying new turn)
        notified_turn_areas = visible_notified_turn_areas + missing_notified_turn_areas

        ## calculate float notified contours image
        float_notified_turn_contours_image = costmap_utils.calc_float_contours_image(costmap_info, [notified_turn_area.contour for notified_turn_area in notified_turn_areas])

        ## calculate binary right/left goals contours image
        binary_right_left_goals_contours_image = np.zeros(float_notified_turn_contours_image.shape, np.uint8)
        for area_idx in range(len(notified_turn_areas)):
            contour_id = area_idx + 1
            binary_notified_turn_contour_image = (float_notified_turn_contours_image==contour_id).astype(np.uint8)
            is_right_left_goal_goal_area = False
            for goal_idx, pose in enumerate(goal_candidate_poses):
                if (has_forward_goal and goal_idx==0) or (goal_idx==backward_goal_idx):
                    continue
                pose_position_map_x, pose_position_map_y = costmap_utils.world_to_map(costmap_info, pose.position.x, pose.position.y)
                goal_candidate_area_id = int(visible_contours_outside_polyhedron_id_map.get_cost(pose_position_map_x, pose_position_map_y))
                if goal_candidate_area_id>0:
                    binary_goal_candidate_area_image = (visible_contours_outside_polyhedron_id_image==goal_candidate_area_id).astype(np.uint8)
                    if np.any(cv2.bitwise_and(binary_notified_turn_contour_image, binary_goal_candidate_area_image)):
                        is_right_left_goal_goal_area = True
                        break
            if is_right_left_goal_goal_area:
                binary_right_left_goals_contours_image = cv2.bitwise_or(binary_notified_turn_contour_image, binary_right_left_goals_contours_image)

        ## select valid notified turn contours
        if (not has_forward_goal) and (((backward_goal_idx is None) and (len(goal_candidate_poses)>0)) or ((backward_goal_idx is not None) and (len(goal_candidate_poses)>1))):
            # check condition 1 : remove all notified turn areas if forward goal does not exist and any goal exist in right/left directions
            valid_notified_turn_contour_ids = []
        else:
            valid_notified_turn_contour_ids = []
            for area_idx in range(len(notified_turn_areas)):
                is_valid_notified_turn_area = True

                ### check condition 2 : remove notified turn area if it is not visible from robot
                contour_id = area_idx + 1
                binary_notified_turn_contour_image = (float_notified_turn_contours_image==contour_id).astype(np.uint8)
                binary_visible_notified_turn_contour_image = cv2.bitwise_and(binary_reachable_visible_area_contour_image, binary_notified_turn_contour_image)
                is_notified_turn_contour_visible = np.any(binary_visible_notified_turn_contour_image)
                if not is_notified_turn_contour_visible:
                    is_valid_notified_turn_area = False
                    rospy.loginfo("remove notified turn area which is not in visible area")
                else:
                    ### check condition 3 : remove notified turn area if it will be separated to multiple areas by coverage polyhedron
                    notified_turn_contour_outside_polyhedron_ids = np.unique(visible_contours_outside_polyhedron_id_image[binary_visible_notified_turn_contour_image>0])
                    if len(notified_turn_contour_outside_polyhedron_ids[notified_turn_contour_outside_polyhedron_ids>0])>1:
                        is_valid_notified_turn_area = False

                    ### check condition 4 : when not notifying new turn, remove notified turn areas which do not overwrap with right/left goal areas
                    if (not notify_new_turn) and (np.all(cv2.bitwise_and(binary_visible_notified_turn_contour_image, binary_right_left_goals_contours_image)==0)):
                        is_valid_notified_turn_area = False

                if is_valid_notified_turn_area:
                    valid_notified_turn_contour_ids.append(contour_id)

        ## find contours for valid notified turn contours outside polyhedron
        if len(valid_notified_turn_contour_ids)>0:
            valid_notified_turn_contours_mask = np.isin(float_notified_turn_contours_image, valid_notified_turn_contour_ids)
            binary_valid_notified_turn_contours_image = np.zeros(float_notified_turn_contours_image.shape, np.uint8)
            binary_valid_notified_turn_contours_image[valid_notified_turn_contours_mask] = 1

            binary_visible_contours_outside_polyhedron_image = (visible_contours_outside_polyhedron_id_image>0).astype(np.uint8)
            binary_valid_notified_turn_areas_outside_polyhedron_image = cv2.bitwise_and(binary_visible_contours_outside_polyhedron_image, binary_valid_notified_turn_contours_image)

            valid_notified_turn_areas_contours_map, _ = cv2.findContours(binary_valid_notified_turn_areas_outside_polyhedron_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            valid_notified_turn_areas_contours = [costmap_utils.map_to_world_np_array(costmap_info, contour_map.squeeze(1)) for contour_map in valid_notified_turn_areas_contours_map]
            valid_notified_turn_areas = [NotifiedTurnArea(robot_pose, contour) for contour in valid_notified_turn_areas_contours]
        else:
            valid_notified_turn_areas_contours = []
            valid_notified_turn_areas = []

        # step 3 : check if there is new turn by checking if valid notified turn areas are overlapped with goal candidate areas
        notified_goal_candidate_contours = []
        notifying_turn_goal_candidate_poses = []
        notified_turn_goal_candidate_poses = []
        new_turn_goal_candidate_poses = []
        ## calculate binary image for valid notified turn area
        binary_valid_notified_turn_contours_image = costmap_utils.calc_binary_contours_image(costmap_info, valid_notified_turn_areas_contours)
        for goal_idx, pose in enumerate(goal_candidate_poses):
            pose_position_map_x, pose_position_map_y = costmap_utils.world_to_map(costmap_info, pose.position.x, pose.position.y)
            goal_candidate_area_id = int(visible_contours_outside_polyhedron_id_map.get_cost(pose_position_map_x, pose_position_map_y))
            if goal_candidate_area_id>0:
                # rospy.loginfo("goal_candidate_area_id = " + str(goal_candidate_area_id))
                if notify_new_turn:
                    ## When notifying turn, add all goal contours to notified goal areas
                    notified_goal_candidate_contours.append(visible_contours_outside_polyhedron[goal_candidate_area_id-1])
                    if (not has_forward_goal or goal_idx!=0) and (goal_idx!=backward_goal_idx):
                        notifying_turn_goal_candidate_poses.append(pose)
                else:
                    ## When not notifying turn, add only goal contours which overwrap with right/left goals to notified goal areas
                    if (not has_forward_goal or goal_idx!=0) and (goal_idx!=backward_goal_idx):
                        ### calculate binary image for goal candidate area
                        binary_goal_candidate_area_image = (visible_contours_outside_polyhedron_id_image==goal_candidate_area_id).astype(np.uint8)

                        ### check if goal candidate area is visible
                        if np.all(cv2.bitwise_and(binary_goal_candidate_area_image, binary_reachable_visible_area_contour_image)==0):
                            ### skip goal which was visible when finding goal, but is not visible now
                            continue

                        ### check if goal candidate area and notified turn area is overlapped
                        if np.all(cv2.bitwise_and(binary_goal_candidate_area_image, binary_valid_notified_turn_contours_image)==0):
                            need_notify_new_turn = True
                            new_turn_goal_candidate_poses.append(pose)
                            rospy.loginfo("found goal in new turn contour")
                        else:
                            notified_goal_candidate_contours.append(visible_contours_outside_polyhedron[goal_candidate_area_id-1])
                            notified_turn_goal_candidate_poses.append(pose)

        ## use optional check if new detected turns are already notified using directions
        updated_notified_turn_goals = []
        if self._use_skip_notify_similar_turn_directions:
            def has_new_goal_direction(robot_pose, new_goal_poses, old_goal_poses):
                old_goal_poses_quats = [geometry_utils.calc_point_direction_quaternion(robot_pose.position, old_goal_pose.position) for old_goal_pose in old_goal_poses]

                has_new_goal_direction = False
                for new_goal_pose in new_goal_poses:
                    is_new_direction_goal = True
                    new_goal_pose_quat = geometry_utils.calc_point_direction_quaternion(robot_pose.position, new_goal_pose.position)
                    for old_goal_pose_quat in old_goal_poses_quats:
                        new_goal_old_goal_quat_diff = geometry_utils.calc_relative_orientation_quaternion(new_goal_pose_quat, old_goal_pose_quat)
                        new_goal_old_goal_abs_yaw_diff = abs(tf.transformations.euler_from_quaternion(new_goal_old_goal_quat_diff)[2])
                        if new_goal_old_goal_abs_yaw_diff<self._max_abs_yaw_diff_similar_turn_directions:
                            is_new_direction_goal = False
                            break
                    if is_new_direction_goal:
                        has_new_goal_direction = True
                        break
                return has_new_goal_direction

            if notify_new_turn:
                ### if forward goal exists and all notifying turn direction is already notified in similar directions, skip stop and notify
                if has_forward_goal and len(notified_turn_goals)>0:
                    has_new_goal_direction = has_new_goal_direction(robot_pose, notifying_turn_goal_candidate_poses, [notified_turn_goal.goal_pose for notified_turn_goal in notified_turn_goals])
                    if not has_new_goal_direction:
                        rospy.loginfo("all visiable goals are in already notified directions")
            else:
                ### if notifying turn contours do not overwrap but similar directions is already notified, suppress to start notify
                if need_notify_new_turn and len(notified_turn_goals)>0:
                    has_new_goal_direction = has_new_goal_direction(robot_pose, new_turn_goal_candidate_poses, [notified_turn_goal.goal_pose for notified_turn_goal in notified_turn_goals])
                    if not has_new_goal_direction:
                        rospy.loginfo("suppress to notify turn for similar direction")
                        need_notify_new_turn = False

            ### update notified turn goals
            #### remove notified turn goals which were observed at far positions
            notified_turn_goals_selector = []
            for notified_turn_goal in notified_turn_goals:
                if geometry_utils.calc_point_distance(robot_pose.position, notified_turn_goal.robot_pose.position)<self._distance_keep_similar_turn_directions:
                    notified_turn_goals_selector.append(True)
                else:
                    notified_turn_goals_selector.append(False)
            updated_notified_turn_goals = list(itertools.compress(notified_turn_goals, notified_turn_goals_selector))

            #### if goal candidate pose is not in similar direction with existing notified turn goals, add as new notified turn goal
            updated_notified_turn_goals_quats = [geometry_utils.calc_point_direction_quaternion(robot_pose.position, turn_goal.goal_pose.position) for turn_goal in updated_notified_turn_goals]
            notifying_notified_turn_goal_candidate_poses = notifying_turn_goal_candidate_poses + notified_turn_goal_candidate_poses
            for turn_goal_candidate_pose in notifying_notified_turn_goal_candidate_poses:
                has_similar_notified_turn_goals_direction= False

                turn_goal_candidate_pose_quat = geometry_utils.calc_point_direction_quaternion(robot_pose.position, turn_goal_candidate_pose.position)
                for updated_notified_turn_goal_quat in updated_notified_turn_goals_quats:
                    turn_goal_candidate_updated_notified_turn_quat_diff = geometry_utils.calc_relative_orientation_quaternion(turn_goal_candidate_pose_quat, updated_notified_turn_goal_quat)
                    turn_goal_candidate_updated_notified_turn_abs_yaw_diff = abs(tf.transformations.euler_from_quaternion(turn_goal_candidate_updated_notified_turn_quat_diff)[2])
                    if turn_goal_candidate_updated_notified_turn_abs_yaw_diff<self._max_abs_yaw_diff_similar_turn_directions:
                        has_similar_notified_turn_goals_direction = True

                if not has_similar_notified_turn_goals_direction:
                    updated_notified_turn_goals.append(NotifiedTurnGoal(robot_pose, turn_goal_candidate_pose))
            rospy.loginfo("number of notified turn goals = " + str(len(updated_notified_turn_goals)))

        # step 4 : update notified turn areas by drawing notified contours and finding contours

        ## create image of notified turn contours and goal candidate areas
        binary_notified_turn_goal_candidate_contours_image = costmap_utils.calc_binary_contours_image(costmap_info, valid_notified_turn_areas_contours + notified_goal_candidate_contours)

        ## create image of visible notified turn contours and goal candidate areas
        binary_visible_notified_turn_goal_candidate_contours_image = cv2.bitwise_and(binary_reachable_visible_area_contour_image, binary_notified_turn_goal_candidate_contours_image)

        ## inflate visible notified turn contours and goal candidate areas within the same overwrapping outside coverage polyhedron area to avoid repeating notification of the same area
        ### calculate float outside polyhedron image
        binary_outside_coverage_polyhedron_image = 1 - binary_coverage_polyhedron_image
        binary_reachable_outside_coverage_polyhedron_image = cv2.bitwise_and(binary_reachable_area_contour_image, binary_outside_coverage_polyhedron_image)
        reachable_outside_polyhedron_contours_map, _ = cv2.findContours(binary_reachable_outside_coverage_polyhedron_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        float_reachable_outside_polyhedron_contours_image = costmap_utils.calc_float_contours_map_image(costmap_info, reachable_outside_polyhedron_contours_map)

        ### find original visible notified turn contours and goal candidate areas
        notified_turn_goal_candidate_contours_map, _ = cv2.findContours(binary_visible_notified_turn_goal_candidate_contours_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        ### inflate each visible notified turn contours and goal candidate areas
        binary_inflate_visible_notified_turn_goal_candidate_contours_image = np.zeros(binary_visible_notified_turn_goal_candidate_contours_image.shape, np.uint8)
        for notified_turn_goal_candidate_contour_map in notified_turn_goal_candidate_contours_map:
            #### calculate binary image of one visible notified turn contours and goal candidate area
            binary_notified_turn_goal_candidate_contour_image = costmap_utils.calc_binary_contours_map_image(costmap_info, [notified_turn_goal_candidate_contour_map])

            #### find ID of overwrapping outside coverage polyhedron area
            overwrap_reachable_outside_polyhedron_contours_ids = np.unique(float_reachable_outside_polyhedron_contours_image[np.where(binary_notified_turn_goal_candidate_contour_image)])
            if len(overwrap_reachable_outside_polyhedron_contours_ids[overwrap_reachable_outside_polyhedron_contours_ids>0])==1:
                #### inflate one visible notified turn contours and goal candidate area
                inflate_binary_notified_turn_goal_candidate_contour_image = costmap_utils.inflate_binary_image(costmap_info, binary_notified_turn_goal_candidate_contour_image, self._notified_turn_visible_area_inflate_radius)

                #### find overwrapping outside coverage polyhedron area, and do not inflate to other areas
                overwrap_reachable_outside_polyhedron_contours_id = overwrap_reachable_outside_polyhedron_contours_ids[overwrap_reachable_outside_polyhedron_contours_ids>0][0]
                binary_overwrap_reachable_outside_polyhedron_contour_image = (float_reachable_outside_polyhedron_contours_image==overwrap_reachable_outside_polyhedron_contours_id).astype(np.uint8)
                inflate_binary_notified_turn_goal_candidate_contour_image = cv2.bitwise_and(inflate_binary_notified_turn_goal_candidate_contour_image, binary_overwrap_reachable_outside_polyhedron_contour_image)

                #### merge visible notified turn contours and goal candidate areas 
                binary_inflate_visible_notified_turn_goal_candidate_contours_image = cv2.bitwise_or(inflate_binary_notified_turn_goal_candidate_contour_image, binary_inflate_visible_notified_turn_goal_candidate_contours_image)
            else:
                #### this line is not expected to call, but merge original visible notified turn contours and goal candidate areas if called
                binary_inflate_visible_notified_turn_goal_candidate_contours_image = cv2.bitwise_or(binary_notified_turn_goal_candidate_contour_image, binary_inflate_visible_notified_turn_goal_candidate_contours_image)
                rospy.logwarn("This line is not expected to call. Visible notified contour is not inflated. overwrap_reachable_outside_polyhedron_contours_ids = " + str(overwrap_reachable_outside_polyhedron_contours_ids))

        ## find updated notified turn contours candidates from inflated visible notified turn contours and goal candidate areas
        notified_turn_goal_candidate_contours_map, _ = cv2.findContours(binary_inflate_visible_notified_turn_goal_candidate_contours_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        notified_turn_goal_candidate_contours = [costmap_utils.map_to_world_np_array(costmap_info, contour_map.squeeze(1)) for contour_map in notified_turn_goal_candidate_contours_map]

        ## create updated notified turn areas
        updated_notified_turn_areas = []
        if len(valid_notified_turn_areas)>0:
            ### select updated notified turn contours candidates which do not overwrap with multiple existing valid notified turn contours
            float_notified_turn_goal_candidate_contours_image = costmap_utils.calc_float_contours_image(costmap_info, notified_turn_goal_candidate_contours)
            float_valid_notified_turn_contours_image = costmap_utils.calc_float_contours_image(costmap_info, valid_notified_turn_areas_contours)
            for notified_turn_goal_candidate_idx, notified_turn_goal_candidate_contour in enumerate(notified_turn_goal_candidate_contours):
                notified_turn_goal_candidate_id = notified_turn_goal_candidate_idx + 1
                binary_notified_turn_goal_candidate_contour_image = np.zeros(float_notified_turn_goal_candidate_contours_image.shape, np.uint8)
                binary_notified_turn_goal_candidate_contour_image[float_notified_turn_goal_candidate_contours_image==notified_turn_goal_candidate_id] = 1
                overwrap_valid_notified_turn_free_contours_ids = np.unique(float_valid_notified_turn_contours_image[np.where(binary_notified_turn_goal_candidate_contour_image)])
                if len(overwrap_valid_notified_turn_free_contours_ids[overwrap_valid_notified_turn_free_contours_ids>0])<=1:
                    updated_notified_turn_areas.append(NotifiedTurnArea(robot_pose, notified_turn_goal_candidate_contour))

            ### add existing valid notified turn areas which do not overwrap with updated notified turn areas
            binary_updated_notified_turn_contours_image = costmap_utils.calc_binary_contours_image(costmap_info, [notified_turn_area.contour for notified_turn_area in updated_notified_turn_areas])
            for area_idx, valid_notified_turn_area in enumerate(valid_notified_turn_areas):
                contour_id = area_idx + 1
                binary_valid_notified_turn_free_contour_image = (float_valid_notified_turn_contours_image==contour_id).astype(np.uint8)
                if np.all(cv2.bitwise_and(binary_updated_notified_turn_contours_image, binary_valid_notified_turn_free_contour_image)==0):
                    updated_notified_turn_areas.append(valid_notified_turn_area)
        else:
            ### create notified turn areas from all candidates if there is no existing valid notified turn area
            updated_notified_turn_areas = [NotifiedTurnArea(robot_pose, contour) for contour in notified_turn_goal_candidate_contours]

        rospy.loginfo("Time to check new turn : " + str(time.perf_counter() - start_time))

        # step 5 : update missing notified turn areas

        # remove missing notified turn areas which were observed at far positions
        missing_notified_turn_areas_selector = []
        for missing_notified_turn_area in missing_notified_turn_areas:
            if geometry_utils.calc_point_distance(robot_pose.position, missing_notified_turn_area.robot_pose.position)<self._distance_keep_missing_notified_turn_areas:
                missing_notified_turn_areas_selector.append(True)
            else:
                missing_notified_turn_areas_selector.append(False)
                rospy.loginfo("remove missing notified turn which is far from robot")
        updated_missing_notified_turn_areas = list(itertools.compress(missing_notified_turn_areas, missing_notified_turn_areas_selector))

        # create binary image of all updated notified areas
        binary_updated_notified_turn_contours_image = np.zeros(binary_visible_notified_turn_goal_candidate_contours_image.shape, np.uint8)
        for updated_notified_turn_area in updated_notified_turn_areas:
            binary_updated_notified_turn_contour_image = costmap_utils.calc_binary_contours_image(costmap_info, [updated_notified_turn_area.contour])
            binary_updated_notified_turn_contours_image = cv2.bitwise_or(binary_updated_notified_turn_contour_image, binary_updated_notified_turn_contours_image)

        # remove missing notified turn areas which overwrap with updated notified areas
        missing_notified_turn_areas_selector = []
        for missing_notified_turn_area in missing_notified_turn_areas:
            binary_missing_notified_turn_contour_image = costmap_utils.calc_binary_contours_image(costmap_info, [missing_notified_turn_area.contour])
            if np.all(cv2.bitwise_and(binary_missing_notified_turn_contour_image, binary_updated_notified_turn_contours_image)==0):
                missing_notified_turn_areas_selector.append(True)
            else:
                missing_notified_turn_areas_selector.append(False)
                rospy.loginfo("remove missing notified turn which overwrap with updated notified turn")
        updated_missing_notified_turn_areas = list(itertools.compress(updated_missing_notified_turn_areas, missing_notified_turn_areas_selector))

        # add old notified areas which becomes invisible to missing list
        for visible_notified_turn_area in visible_notified_turn_areas:
            binary_visible_notified_turn_contour_image = costmap_utils.calc_binary_contours_image(costmap_info, [visible_notified_turn_area.contour])
            if np.all(cv2.bitwise_and(binary_visible_notified_turn_contour_image, binary_updated_notified_turn_contours_image)==0):
                updated_missing_notified_turn_areas.append(visible_notified_turn_area)

        # remove missing notified turn areas which overwrap with forward/backward goal
        updated_missing_notified_turn_areas_selector = []
        for updated_missing_notified_turn_area in updated_missing_notified_turn_areas:
            is_forward_backward_goal_area = False

            binary_missing_notified_turn_area_image = costmap_utils.calc_binary_contours_image(costmap_info, [updated_missing_notified_turn_area.contour])
            if has_forward_goal:
                goal_map_x, goal_map_y = costmap_utils.world_to_map(costmap_info, goal_candidate_poses[0].position.x, goal_candidate_poses[0].position.y)
                if binary_missing_notified_turn_area_image[goal_map_y][goal_map_x]>0:
                    is_forward_backward_goal_area = True
            if (not is_forward_backward_goal_area) and (backward_goal_idx is not None):
                goal_map_x, goal_map_y = costmap_utils.world_to_map(costmap_info, goal_candidate_poses[backward_goal_idx].position.x, goal_candidate_poses[backward_goal_idx].position.y)
                if binary_missing_notified_turn_area_image[goal_map_y][goal_map_x]>0:
                    is_forward_backward_goal_area = True

            if not is_forward_backward_goal_area:
                updated_missing_notified_turn_areas_selector.append(True)
            else:
                updated_missing_notified_turn_areas_selector.append(False)
        updated_missing_notified_turn_areas = list(itertools.compress(updated_missing_notified_turn_areas, updated_missing_notified_turn_areas_selector))

        # rospy.loginfo("number of original visible notified turn = " + str(len(visible_notified_turn_areas)))
        # rospy.loginfo("number of original missing notified turn = " + str(len(missing_notified_turn_areas)))
        # rospy.loginfo("number of updated missing notified turn = " + str(len(updated_missing_notified_turn_areas)))

        return need_notify_new_turn, updated_notified_turn_areas, updated_missing_notified_turn_areas, updated_notified_turn_goals, has_new_goal_direction


    ### public interfaces
    def observe_once(self):
        msg = std_msgs.msg.String()
        msg.data = "observe_once"
        self._cabot_explore_event_pub.publish(msg)

    def start_explore(self):
        msg = std_msgs.msg.String()
        msg.data = "start"
        self._cabot_explore_event_pub.publish(msg)

    def start_explore_forward(self):
        msg = std_msgs.msg.String()
        msg.data = "start_forward"
        self._cabot_explore_event_pub.publish(msg)

    def go_origin(self):
        msg = std_msgs.msg.String()
        msg.data = "go_origin"
        self._cabot_explore_event_pub.publish(msg)

    def system_cancel_explore(self):
        msg = std_msgs.msg.String()
        msg.data = "system_cancel"
        self._cabot_explore_event_pub.publish(msg)

    def user_cancel_explore(self):
        msg = std_msgs.msg.String()
        msg.data = "user_cancel"
        self._cabot_explore_event_pub.publish(msg)

    def start_turn(self, turn_direction):
        msg = std_msgs.msg.String()
        msg.data = "start_turn:" + turn_direction
        self._cabot_explore_event_pub.publish(msg)

    def finish_turn(self, turn_direction):
        msg = std_msgs.msg.String()
        msg.data = "finish_turn:" + turn_direction
        self._cabot_explore_event_pub.publish(msg)

    def move_forward_route(self):
        with self._lock_last_explore_status:
            last_explore_status = copy.copy(self._last_explore_status)

        if last_explore_status["has_forward_goal"]:
            rospy.loginfo("move forward")
            self.delegate.move_forward_route()
            self.start_explore()
        else:
            rospy.loginfo("move forward function is called, but forward route is not found. last_explore_status = " + str(last_explore_status))
            self.delegate.cannot_move_forward_route()

    def _find_current_index(self, explore_status):
        goal_candidate_poses = explore_status["goal_candidate_poses"]
        
        rospy.loginfo("all")
        for i in range(0, len(goal_candidate_poses)):
            right_point = goal_candidate_poses[i].position
            right_pose = geoutil.Pose.pose_from_points(right_point, self.current_pose)
            rospy.loginfo("index=%d angle=%.2f"%(i, right_pose.r))
        rospy.loginfo("exact")
        for i in range(0, len(goal_candidate_poses)):
            right_point = goal_candidate_poses[i].position
            right_pose = geoutil.Pose.pose_from_points(right_point, self.current_pose)
            right_diff = geoutil.diff_angle(self.current_pose.orientation, right_pose.orientation)
            rospy.loginfo("index=%d orientation=%.2f right=%.2f"%(i, self.current_pose.r, right_diff))
            rospy.loginfo(str(self.current_pose))
            rospy.loginfo(str(right_pose))
            if abs(right_diff) < math.pi/12:
                return i
        rospy.loginfo("between")
        for i in range(0, len(goal_candidate_poses)):
            right_point = goal_candidate_poses[i].position
            right_pose = geoutil.Pose.pose_from_points(right_point, self.current_pose)
            right_diff = geoutil.diff_angle(self.current_pose.orientation, right_pose.orientation)
            left_point = goal_candidate_poses[i-1].position
            left_pose =  geoutil.Pose.pose_from_points(left_point, self.current_pose)
            left_diff = geoutil.diff_angle(self.current_pose.orientation, left_pose.orientation)
            rospy.loginfo("index=%d left=%.2f right=%.2f"%(i, left_diff, right_diff))
            if left_diff > 0 and right_diff < 0:
                return i-0.5
        if len(goal_candidate_poses) == 1:
            return 0.5
        rospy.logerr("_find_current_index unexpected condition")
        return 0

    def turn_right_route(self, turn_done_callback):
        self.start_turn("right")

        with self._lock_last_explore_status:
            last_explore_status = copy.copy(self._last_explore_status)

        self.turn_route(+1, last_explore_status, turn_done_callback)

    def turn_left_route(self, turn_done_callback):
        self.start_turn("left")

        with self._lock_last_explore_status:
            last_explore_status = copy.copy(self._last_explore_status)

        self.turn_route(-1, last_explore_status, turn_done_callback)

    def turn_route(self, clockwise, explore_status, turn_done_callback):
        goal_candidate_poses = explore_status["goal_candidate_poses"]

        if len(goal_candidate_poses) == 0: 
            turn_done_callback(GoalStatus.SUCCEEDED, None)
            self._publish_explore_ui_status(explore_status)
            return
        
        index = self._find_current_index(explore_status)
        if (not self.delegate._status_manager.turned_route) and (not explore_status["has_forward_goal"]) and (index==0) and (clockwise==1):
            index = -1
            new_index = 0
        else:
            new_index = int(index + clockwise) % len(goal_candidate_poses)
        rospy.loginfo("index=%.2f new_index=%d", index, new_index)
        rospy.loginfo("goal_candidate_poses = " + str(goal_candidate_poses))
        if index != new_index:
            move_point = goal_candidate_poses[new_index].position
            pose = geoutil.Pose.pose_from_points(move_point, self.current_pose)
            self.turn_towards(pose.orientation, explore_status, turn_done_callback, clockwise = clockwise)
        else:
            rospy.loginfo("only one candidate and already heading to the candidate")
            self.delegate.cannot_turn_route()
            turn_done_callback(GoalStatus.SUCCEEDED, None)
            self._publish_explore_ui_status(explore_status)

        self.delegate._status_manager.turned_route = True

    def turn_towards(self, orientation, explore_status, callback, clockwise=0):
        rospy.loginfo("turn_towards")
        self._turn_towards(orientation, explore_status, callback, clockwise=clockwise)

        yaw_msg = std_msgs.msg.Float64()
        yaw_msg.data = geoutil.get_yaw(geoutil.q_from_msg(orientation))
        self._turn_pub.publish(yaw_msg)

    @util.setInterval(0.01, times=1)
    def _turn_towards(self, orientation, explore_status, callback, clockwise=0):
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
            self._spin_client.send_goal(goal, lambda x,y: self._turn_towards(orientation, explore_status, callback, clockwise=clockwise))
            rospy.loginfo("sent goal %s", str(goal))

            if turn_yaw>math.pi/8 or turn_yaw<-math.pi/8:
                self.delegate.notify_turn(turn=Turn(self.current_pose.to_pose_stamped_msg(self._global_map_name), turn_yaw), pose=self.current_pose)
                rospy.loginfo("notify turn %s", str(turn_yaw))
            else:
                rospy.loginfo("skip notify turn %s", str(turn_yaw))
        else:
            rospy.loginfo("turn completed {}".format(diff))
            callback(GoalStatus.SUCCEEDED, None)
            self._publish_explore_ui_status(explore_status)


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

    def publish_arrived_intersection_pose(self):
        ros_pose = self.current_2D_pose()
        self._arrived_intersection_pose_pub.publish(ros_pose)
    ## Main loop of navigation

    @util.setInterval(0.1)
    def _check_loop(self):
        if rospy.is_shutdown():
            self._stop_loop()
            return

        ## need a robot position
        try:
            self.current_pose = self.current_local_pose()
            ros_pose = self.current_2D_pose()
            self._pose_pub.publish(ros_pose)
            #rospy.logdebug_throttle(1, "current pose %s", self.current_pose)
        except RuntimeError:
            rospy.loginfo_throttle(3, "could not get position")
            return

        ## say I am ready once
        if not self.i_am_ready:
            rospy.logdebug("i am ready")
            self.delegate.i_am_ready()
            self.i_am_ready = True

        ## cabot is active now
        #rospy.logdebug_throttle(10, "cabot is active")