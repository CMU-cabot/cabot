#!/usr/bin/env python3

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
import itertools
import json
import math
import sys
import time
import threading

import actionlib
from actionlib_msgs.msg import GoalStatus
import cv2
from geometry_msgs.msg import Point, Pose, PoseStamped
import matplotlib
import nav2_msgs.msg
from nav_msgs.msg import OccupancyGrid, Path
import networkx
import numpy as np
import rospy
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan
import std_msgs.msg
import tf
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

from route_explore_msgs.srv import AddLandmark, FindClosestLandmarkId, ResetCartographerLandmarkManager, ResetExploreManager, SearchPath
from route_explore_utils import costmap_utils, geometry_utils, route_utils, visualize_utils
from route_explore_utils.status import ExploreState


class ExploreManagerNode:

    ACTION = "/cabot_explore"
    EXPLORE_BT_XML = "package://cabot_bt/behavior_trees/navigate_explore_w_replanning_and_recovery.xml"

    def __init__(self, use_scan_topic_free_area, use_dt_polyhedron_sensor_range, default_polyhedron_sensor_range, dt_polyhedron_closest_node_range, dt_polyhedron_eps,
                use_filter_forward_turn_route, use_sample_free_points, sample_free_points_min_distance, sample_free_points_by_angle_range, sample_free_points_by_angle_unknown_area_margin,
                sample_free_points_by_angle_lethal_area_margin, use_cluster_point_outside_polyhedron_free_area, costmap_topic, scan_topic, path_topic, lethal_cost_threshold, costmap_inflation_radius,
                costmap_resolution, min_frontier_size, use_pause_possible_turn, possible_turn_perpendicular_yaw_tolerance, use_frontier_proximity, frontier_proximity_threshold,
                min_distance_goal_inside_coverage_polyhedron, max_abs_yaw_forward_goal, max_landmark_interval, landmark_loop_detect_distance):
        rospy.loginfo("Start initializing ExploreManagerNode...")

        # start of constant parameters
        self._lookup_transform_duration = 1.0

        self._minimum_route_length = 3.0
        self._extract_route_gaussian_radius = 0.2
        self._extract_route_min_detect_l_corner_distance = 1.0
        self._extract_route_max_detect_l_corner_angle = math.pi*3.0/4.0
        self._sample_route_edge_points_interval = 0.5

        self._tolerance_robot_close_possible_turn = 2.0
        self._tolerance_robot_close_last_possible_turn = 2.0

        self._update_goal_move_distance = 1.0
        self._stack_time = 5.0
        self._stack_distance = 0.1

        self._num_sample_coverage_polyhedron = 36
        self._angles_sample_coverage_polyhedron = [i*(math.pi*2.0)/float(self._num_sample_coverage_polyhedron) for i in range(self._num_sample_coverage_polyhedron)]
        self._angles_sample_coverage_polyhedron = [angle if angle<=math.pi else -(math.pi*2.0-angle) for angle in self._angles_sample_coverage_polyhedron]
        self._polyhedron_buffer_length = 0.5
        self._polyhedron_buffer_samples = 12

        self._ignore_small_area_square_meters = 1.0

        self._num_sample_visible_area_points = 72
        self._angles_sample_visible_area_points = [i*(math.pi*2.0)/float(self._num_sample_visible_area_points) for i in range(self._num_sample_visible_area_points)]
        self._angles_sample_visible_area_points = [angle if angle<=math.pi else -(math.pi*2.0-angle) for angle in self._angles_sample_visible_area_points]

        self._num_sample_free_points = 36
        self._angles_sample_free_points = [i*(math.pi*2.0)/float(self._num_sample_free_points) for i in range(self._num_sample_free_points)]
        self._angles_sample_free_points = [angle if angle<=math.pi else -(math.pi*2.0-angle) for angle in self._angles_sample_free_points]

        self._update_rate_heading_yaw = 0.2
        self._tolerance_robot_arrive_pose_to_turn = 0.60 # set larger than xy_goal_tolerance to avoid setting pose to turn as a goal many times

        self._wait_resend_goal_pose_to_turn = 0.5
        self._wait_after_pause_possible_turn = 2.0
        self._wait_map_update_time_dead_end = 2.0

        self._skip_close_first_goal_poses_distance = 3.0
        # end of constant parameters

        # start initialization
        self._use_scan_topic_free_area = use_scan_topic_free_area
        self._use_dt_polyhedron_sensor_range = use_dt_polyhedron_sensor_range
        self._default_polyhedron_sensor_range = default_polyhedron_sensor_range
        self._dt_polyhedron_closest_node_range = dt_polyhedron_closest_node_range
        self._dt_polyhedron_eps = dt_polyhedron_eps
        self._use_filter_forward_turn_route = use_filter_forward_turn_route
        self._use_sample_free_points = use_sample_free_points
        self._sample_free_points_min_distance = sample_free_points_min_distance
        self._sample_free_points_by_angle_range = sample_free_points_by_angle_range
        self._sample_free_points_by_angle_unknown_area_margin = sample_free_points_by_angle_unknown_area_margin
        self._sample_free_points_by_angle_lethal_area_margin = sample_free_points_by_angle_lethal_area_margin
        self._use_cluster_point_outside_polyhedron_free_area = use_cluster_point_outside_polyhedron_free_area
        self._costmap_topic = costmap_topic
        self._scan_topic = scan_topic
        self._path_topic = path_topic
        self._lethal_cost_threshold = lethal_cost_threshold
        self._costmap_inflation_radius = costmap_inflation_radius
        self._costmap_resolution = costmap_resolution
        self._min_frontier_size = min_frontier_size
        self._use_pause_possible_turn = use_pause_possible_turn
        self._possible_turn_perpendicular_yaw_tolerance = possible_turn_perpendicular_yaw_tolerance
        self._use_frontier_proximity = use_frontier_proximity
        self._frontier_proximity_threshold = frontier_proximity_threshold
        self._min_distance_goal_inside_coverage_polyhedron = min_distance_goal_inside_coverage_polyhedron
        self._max_abs_yaw_forward_goal = max_abs_yaw_forward_goal # if absolute yaw direction from robot to goal candidate is smaller than this value, that goal is considered as forward goal
        self._min_abs_yaw_backward_goal = math.pi-self._max_abs_yaw_forward_goal # if absolute yaw direction from robot to goal candidate is larger than this value, that goal is considered as backward goal
        self._max_landmark_interval = max_landmark_interval
        self._landmark_loop_detect_distance = landmark_loop_detect_distance

        self._tf2_buffer = tf2_ros.Buffer()
        self._tf2_listener = tf2_ros.TransformListener(self._tf2_buffer)

        # initialize simple action client, action server
        self._navigate_to_pose_client = actionlib.SimpleActionClient("navigate_to_pose", nav2_msgs.msg.NavigateToPoseAction)

        # service proxy for landmark manager
        self._add_landmark_proxy = rospy.ServiceProxy('/cartographer_landmark_manager/add_landmark', AddLandmark)
        self._find_closest_landmark_id_proxy = rospy.ServiceProxy('/cartographer_landmark_manager/find_closest_landmark_id', FindClosestLandmarkId)
        self._reset_cartographer_landmark_manager_proxy = rospy.ServiceProxy('/cartographer_landmark_manager/reset_cartographer_landmark_manager', ResetCartographerLandmarkManager)
        self._search_path_proxy = rospy.ServiceProxy('/cartographer_landmark_manager/search_path', SearchPath)

        # publisher
        self._explore_status_publisher = rospy.Publisher('/cabot_explore/explore_status', std_msgs.msg.String, queue_size=1, latch=True)
        self._inflate_map_publisher = rospy.Publisher('/cabot_explore/inflate_map', OccupancyGrid, queue_size=1)
        self._frontier_proximity_map_publisher = rospy.Publisher('/cabot_explore/frontier_proximity_map', OccupancyGrid, queue_size=1)
        self._next_goal_candidates_publisher = rospy.Publisher('/cabot_explore/next_goal_candidates', MarkerArray, queue_size=1)
        self._path_publisher = rospy.Publisher(self._path_topic, Path, queue_size=1)

        # subscriber
        self._explore_event_subscriber = None
        self._occupancy_grid_subscriber = None
        self._scan_subscriber = None

        self._process_costmap_thread = None

        self._lock_costmap_msg = threading.Lock()
        self._lock_costmap = threading.Lock()
        self._lock_scan_msg = threading.Lock()
        self._lock_route_node_points = threading.Lock()
        self._lock_frontier_proximity_map = threading.Lock()
        self._lock_landmark = threading.Lock()
        self._lock_route_explore_state = threading.Lock()
        self._reset_explore_manager()


    def _reset_explore_manager(self):
        if self._explore_event_subscriber is not None:
            self._explore_event_subscriber.unregister()

        if self._occupancy_grid_subscriber is not None:
            self._occupancy_grid_subscriber.unregister()

        if self._scan_subscriber is not None:
            self._scan_subscriber.unregister()

        while (self._process_costmap_thread is not None) and (self._process_costmap_thread.is_alive()):
            rospy.loginfo("wait to finish process costmap thread")
            rospy.sleep(1.0)
        self._process_costmap_thread = None

        with self._lock_costmap_msg:
            self._costmap_msg = None

        with self._lock_costmap:
            self._costmap = None
            self._resized_inflate_costmap = None
            self._reachable_area_contours = None
            self._reachable_area_hierarchy = None
            self._time_update_costmap = None

        with self._lock_scan_msg:
            self._scan_msg = None

        with self._lock_route_node_points:
            self._route_node_points = []
            self._route_edge_points = []
            self._route_node_connections = {}
            self._route_node_points_indices_edge_idx_dict = {}
            self._route_edge_idx_node_points_indices_dict = {}
            self._route_edge_idx_edge_points_indices_dict = {}
            self._route_edge_point_idx_edge_idx_dict = {}
            self._route_node_points_np_array = None
            self._route_graph = networkx.Graph()
            self._route_dt_map = None
            self._time_update_route_graph = None

        with self._lock_frontier_proximity_map:
            self._frontier_proximity_map = None

        with self._lock_landmark:
            resp = self._reset_cartographer_landmark_manager_proxy()
            if resp.result:
                rospy.loginfo("reset cartographer landmark manager successed")
            else:
                rospy.logerr("reset cartographer landmark manager faild")

            self._next_landmark_neighbor_id = None
            self._last_landmark_pose = None

        with self._lock_route_explore_state:
            self._route_explore_state = ExploreState.WAIT
            self._heading_yaw = None
            self._pose_to_turn = None
            self._goal_candidate_poses = []
            self._has_forward_goal = False
            self._update_goal_costmap_info = None
            self._update_goal_reachable_area_contours = None
            self._update_goal_reachable_area_hierarchy = None
            self._update_goal_visible_area_contour = None
            self._update_goal_coverage_polyhedron = None
            self._time_update_goal_candidate_poses = None

            self._need_pause_possible_turn = False
            self._last_pause_possible_turn_point = None

            self._need_send_next_goal = True
            self._prev_send_goal_robot_pose = None

            self._prev_check_stack_robot_pose = None
            self._prev_check_stack_time = None

        self._explore_event_subscriber = rospy.Subscriber("/cabot_explore/explore_event", std_msgs.msg.String, self._cabot_explore_event_callback, queue_size=1)

        self._occupancy_grid_subscriber = rospy.Subscriber(self._costmap_topic, OccupancyGrid, self._costmap_callback, queue_size=1)

        self._scan_subscriber = rospy.Subscriber(self._scan_topic, LaserScan, self._scan_callback, queue_size=1)

        # publish initial state
        with self._lock_route_explore_state:
            explore_status = self._create_cabot_explore_status(self._route_explore_state, self._update_goal_costmap_info, self._pose_to_turn, self._goal_candidate_poses, self._has_forward_goal, \
                                                            self._update_goal_reachable_area_contours, self._update_goal_reachable_area_hierarchy, self._update_goal_visible_area_contour, self._update_goal_coverage_polyhedron)
            self._explore_status_publisher.publish(json.dumps(explore_status))

        rospy.loginfo("finish reset explore manager")


    def reset_explore_manager_callback(self, data):
        success = True
        try:
            self._reset_explore_manager()
        except:
            success = False
        return success


    def _costmap_callback(self, msg):
        with self._lock_costmap_msg:
            self._costmap_msg = msg


    def _scan_callback(self, msg):
        with self._lock_scan_msg:
            self._scan_msg = msg

        # start thread for process costmap
        if (self._process_costmap_thread is None) or (not self._process_costmap_thread.is_alive()):
            self._process_costmap_thread = threading.Thread(target=self._run_process_costmap)
            self._process_costmap_thread.start()


    def _run_process_costmap(self):
        start_time = time.perf_counter()

        self._run_inflate_costmap()
        self._run_route_extract()
        self._run_calc_frontier_proximity_map()

        rospy.loginfo("Time to call process costmap : " + str(time.perf_counter() - start_time))


    def _run_inflate_costmap(self):
        with self._lock_costmap_msg:
            costmap_msg = copy.copy(self._costmap_msg)
        if costmap_msg is None:
            return

        costmap_info = costmap_utils.CostmapInfo(costmap_msg.info)
        costmap_image = np.array(costmap_msg.data, dtype=np.int8).reshape(costmap_info.height, costmap_info.width)
        costmap = costmap_utils.Costmap(costmap_info, tuple(costmap_image.flatten()))

        if self._costmap_resolution!=costmap_info.resolution:
            ## resize cost map image
            resize_ratio = costmap_info.resolution / self._costmap_resolution

            ## resize cost map by inter nearest interpolation
            resized_costmap_image = cv2.resize(costmap_image, dsize=None, fx=resize_ratio, fy=resize_ratio, interpolation=cv2.INTER_NEAREST).astype(np.int8)

            ## to avoid missing lethal area by digitization error, fill as lethal area for all inter area interpolated binary lethal image
            binary_lethal_image = costmap_utils.calc_binary_lethal_image(costmap_image, self._lethal_cost_threshold)
            resized_int_lethal_image = cv2.resize(binary_lethal_image*255, dsize=None, fx=resize_ratio, fy=resize_ratio, interpolation=cv2.INTER_AREA)
            resized_costmap_image[resized_int_lethal_image>0] = 100

            ## to avoid missing unknown area by digitization error, fill as unknown area for inter area interpolated binary unknown image and free area in resized image
            binary_unknown_image = costmap_utils.calc_binary_unknown_image(costmap_image)
            resized_int_unknown_image = cv2.resize(binary_unknown_image*255, dsize=None, fx=resize_ratio, fy=resize_ratio, interpolation=cv2.INTER_AREA)
            resized_costmap_image[np.where((resized_costmap_image<self._lethal_cost_threshold) & (resized_int_unknown_image>0))] = -1

            ## crerate cost map by resized image
            resized_costmap_grid_info = copy.copy(costmap_info.occupancy_grid_info)
            resized_costmap_grid_info.resolution = self._costmap_resolution
            resized_costmap_grid_info.width = resized_costmap_image.shape[1]
            resized_costmap_grid_info.height = resized_costmap_image.shape[0]
            resized_costmap_info = costmap_utils.CostmapInfo(resized_costmap_grid_info)
        else:
            ## create cost map and inflate cost map from original size images
            resized_costmap_image = copy.copy(costmap_image)
            resized_costmap_info = copy.copy(costmap_info)

        if self._use_scan_topic_free_area:
            with self._lock_scan_msg:
                scan_msg = copy.copy(self._scan_msg)

            # merge lethal points and free area detected by scan topic to map
            if scan_msg is not None:
                try:
                    # find transformation from velodyne to map
                    transform_stamped = self._tf2_buffer.lookup_transform("map", "velodyne", scan_msg.header.stamp, rospy.Duration(self._lookup_transform_duration))
                    transform = transform_stamped.transform

                    scan_points = costmap_utils.calc_scan_points_in_world(scan_msg, transform)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.loginfo("Transform from map to velodyne is not ready")
                    scan_points = np.array([])
            else:
                scan_points = np.array([])

            if len(scan_points)>0:
                # calculate free contour from scan and merge with map
                binary_scan_contour_image = costmap_utils.calc_binary_contours_image(resized_costmap_info, np.array([scan_points]))
                resized_costmap_image[np.where((resized_costmap_image<0) & (binary_scan_contour_image==1))] = 0

        # inflate lethal area
        resized_inflate_costmap_image = costmap_utils.inflate_costmap(resized_costmap_info, resized_costmap_image, self._lethal_cost_threshold, self._costmap_inflation_radius)
        reachable_inflate_costmap_image, reachable_area_contours, reachable_area_hierarchy = costmap_utils.calc_reachable_map(resized_costmap_info, resized_inflate_costmap_image, self._lethal_cost_threshold, self.get_current_pose())
        resized_inflate_costmap = costmap_utils.Costmap(resized_costmap_info, tuple(reachable_inflate_costmap_image.flatten()))

        with self._lock_costmap:
            self._costmap = costmap
            self._resized_inflate_costmap = resized_inflate_costmap
            self._reachable_area_contours = reachable_area_contours
            self._reachable_area_hierarchy = reachable_area_hierarchy
        self._time_update_costmap = rospy.Time.now()

        # publish inflate map for visualization
        inflate_occupancy_grid = OccupancyGrid()
        inflate_occupancy_grid.header.stamp = rospy.Time.now()
        inflate_occupancy_grid.header.frame_id = "map"
        inflate_occupancy_grid.info = resized_inflate_costmap.costmap_info.occupancy_grid_info
        inflate_occupancy_grid.data = resized_inflate_costmap.occupancy_grid_data
        self._inflate_map_publisher.publish(inflate_occupancy_grid)


    def _run_route_extract(self):
        with self._lock_costmap:
            resized_inflate_costmap = copy.copy(self._resized_inflate_costmap)
        if resized_inflate_costmap is None:
            return

        current_robot_pose = self.get_current_pose()
        map_route_node_points, route_node_points, route_node_connections, map_route_points_set, route_dt_image = route_utils.extract_route(resized_inflate_costmap, self._lethal_cost_threshold, current_robot_pose,
                                                                                                                                        self._minimum_route_length, gaussian_radius=self._extract_route_gaussian_radius,
                                                                                                                                        min_detect_l_corner_distance=self._extract_route_min_detect_l_corner_distance,
                                                                                                                                        max_detect_l_corner_angle=self._extract_route_max_detect_l_corner_angle)

        route_node_points_np_array = np.array(route_node_points)

        # sample route edge points for all connections
        route_edge_points = []
        route_node_points_indices_edge_idx_dict = {}
        route_edge_idx_node_points_indices_dict = {}
        route_edge_idx_edge_points_indices_dict = {}
        route_edge_point_idx_edge_idx_dict = {}
        for node_idx in route_node_connections.keys():
            for connected_node_idx in route_node_connections[node_idx]:
                if (node_idx, connected_node_idx) not in route_node_points_indices_edge_idx_dict:
                    points = route_utils.sample_route_edge_points(resized_inflate_costmap, map_route_node_points, map_route_points_set, node_idx, connected_node_idx, sample_interval=self._sample_route_edge_points_interval)
                    route_edge_points.extend(points)

                    edge_idx = len(route_node_points_indices_edge_idx_dict.keys())
                    edge_points_indices = list(range(len(route_edge_points)-len(points), len(route_edge_points)))
                    route_node_points_indices_edge_idx_dict[(node_idx, connected_node_idx)] = edge_idx
                    route_node_points_indices_edge_idx_dict[(connected_node_idx, node_idx)] = edge_idx
                    route_edge_idx_node_points_indices_dict[edge_idx] = (node_idx, connected_node_idx)
                    route_edge_idx_edge_points_indices_dict[edge_idx] = edge_points_indices
                    for edge_point_idx in edge_points_indices:
                        route_edge_point_idx_edge_idx_dict[edge_point_idx] = edge_idx

        # convert route node points and route edge points to geometry_msgs.msg.Point
        route_node_points = [Point(point[0], point[1], 0.0) for point in route_node_points]
        route_edge_points = [Point(point[0], point[1], 0.0) for point in route_edge_points]

        # update graph to search shortest path
        route_graph = networkx.Graph()
        for node_idx in range(len(route_node_points)):
            route_graph.add_node(node_idx)
        for node_idx in route_node_connections.keys():
            for connected_node_idx in route_node_connections[node_idx]:
                if not route_graph.has_edge(node_idx, connected_node_idx):
                    edge_distance = geometry_utils.calc_point_distance(route_node_points[node_idx], route_node_points[connected_node_idx])
                    route_graph.add_edge(node_idx, connected_node_idx, distance=edge_distance)

        route_dt_map = costmap_utils.Costmap(resized_inflate_costmap.costmap_info, tuple(route_dt_image.flatten().astype(np.float32)))

        with self._lock_route_node_points:
            self._route_node_points = route_node_points
            self._route_edge_points = route_edge_points
            self._route_node_connections = route_node_connections
            self._route_node_points_indices_edge_idx_dict = route_node_points_indices_edge_idx_dict
            self._route_edge_idx_node_points_indices_dict = route_edge_idx_node_points_indices_dict
            self._route_edge_idx_edge_points_indices_dict = route_edge_idx_edge_points_indices_dict
            self._route_edge_point_idx_edge_idx_dict = route_edge_point_idx_edge_idx_dict
            self._route_node_points_np_array = route_node_points_np_array
            self._route_graph = route_graph
            self._route_dt_map = route_dt_map
            self._time_update_route_graph = rospy.Time.now()


    def _run_calc_frontier_proximity_map(self):
        with self._lock_costmap:
            resized_inflate_costmap = copy.copy(self._resized_inflate_costmap)
        if resized_inflate_costmap is None:
            return

        current_robot_pose = self.get_current_pose()
        frontier_proximity_map = costmap_utils.calc_frontier_proximity_map(resized_inflate_costmap, self._lethal_cost_threshold, self._frontier_proximity_threshold,
                                                                        min_frontier_size=self._min_frontier_size, robot_pose=current_robot_pose)

        with self._lock_frontier_proximity_map:
            self._frontier_proximity_map = frontier_proximity_map

        # publish frontier proximity map for visualization
        frontier_proximity_occupancy_grid = OccupancyGrid()
        frontier_proximity_occupancy_grid.header.stamp = rospy.Time.now()
        frontier_proximity_occupancy_grid.header.frame_id = "map"
        frontier_proximity_occupancy_grid.info = frontier_proximity_map.costmap_info.occupancy_grid_info
        frontier_proximity_occupancy_grid.data = tuple(np.array(frontier_proximity_map.occupancy_grid_data, dtype=np.int8)*100)
        self._frontier_proximity_map_publisher.publish(frontier_proximity_occupancy_grid)


    def get_current_pose(self):
        while not rospy.is_shutdown():
            try:
                t = self._tf2_buffer.lookup_transform("map", "base_footprint", rospy.Time(0), rospy.Duration(self._lookup_transform_duration))

                current_pose = Pose()
                current_pose.position.x = t.transform.translation.x
                current_pose.position.y = t.transform.translation.y
                current_pose.position.z = t.transform.translation.z
                current_pose.orientation.x = t.transform.rotation.x
                current_pose.orientation.y = t.transform.rotation.y
                current_pose.orientation.z = t.transform.rotation.z
                current_pose.orientation.w = t.transform.rotation.w
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.loginfo("Transform from map to robot is not ready")
            rospy.sleep(0.01)
        return current_pose


    def is_pose_far_forward(self, robot_pose, target_pose, distance_threshold):
        is_pose_far_forward = False

        robot_pose_quat = (robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w)
        pose_to_turn_quat = geometry_utils.calc_point_direction_quaternion(robot_pose.position, target_pose.position)
        pose_to_turn_quat_diff = geometry_utils.calc_relative_orientation_quaternion(robot_pose_quat, pose_to_turn_quat)
        pose_to_turn_yaw_diff = tf.transformations.euler_from_quaternion(pose_to_turn_quat_diff)[2]
        if math.cos(pose_to_turn_yaw_diff)>0:
            pose_distance = geometry_utils.calc_point_distance(robot_pose.position, target_pose.position)
            if pose_distance>distance_threshold:
                is_pose_far_forward = True
        return is_pose_far_forward


    def create_goal_path(self, robot_pose, target_pose_list, heading_yaw):
        # make goal pose
        goal = nav2_msgs.msg.NavigateToPoseGoal()
        goal.behavior_tree = ExploreManagerNode.EXPLORE_BT_XML
        goal.pose = PoseStamped()
        goal.pose.header.stamp = rospy.Time.now()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = target_pose_list[-1].position.x
        goal.pose.pose.position.y = target_pose_list[-1].position.y
        goal.pose.pose.position.z = 0
        if len(target_pose_list)>1:
            target_pose_yaw = geometry_utils.calc_point_direction_yaw(target_pose_list[-2].position, target_pose_list[-1].position)
        else:
            target_pose_yaw = geometry_utils.calc_point_direction_yaw(robot_pose.position, target_pose_list[-1].position)
        target_pose_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, target_pose_yaw)
        goal.pose.pose.orientation.x = target_pose_quat[0]
        goal.pose.pose.orientation.y = target_pose_quat[1]
        goal.pose.pose.orientation.z = target_pose_quat[2]
        goal.pose.pose.orientation.w = target_pose_quat[3]

        # make ros path to goal pose
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"
        path.poses = []

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose = Pose()
        pose.pose.position.x = robot_pose.position.x
        pose.pose.position.y = robot_pose.position.y
        pose.pose.position.z = 0
        target_pose_yaw = geometry_utils.calc_point_direction_yaw(robot_pose.position, target_pose_list[0].position)
        target_pose_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, target_pose_yaw)
        pose.pose.orientation.x = target_pose_quat[0]
        pose.pose.orientation.y = target_pose_quat[1]
        pose.pose.orientation.z = target_pose_quat[2]
        pose.pose.orientation.w = target_pose_quat[3]
        path.poses.append(pose)

        for idx in range(len(target_pose_list)):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose = Pose()
            pose.pose.position.x = target_pose_list[idx].position.x
            pose.pose.position.y = target_pose_list[idx].position.y
            pose.pose.position.z = 0
            if idx==0:
                target_pose_yaw = geometry_utils.calc_point_direction_yaw(robot_pose.position, target_pose_list[idx].position)
            else:
                target_pose_yaw = geometry_utils.calc_point_direction_yaw(target_pose_list[idx-1].position, target_pose_list[idx].position)
            target_pose_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, target_pose_yaw)
            pose.pose.orientation.x = target_pose_quat[0]
            pose.pose.orientation.y = target_pose_quat[1]
            pose.pose.orientation.z = target_pose_quat[2]
            pose.pose.orientation.w = target_pose_quat[3]
            path.poses.append(pose)

        # update heading direction
        heading_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, heading_yaw)
        updated_heading_quat = tf.transformations.quaternion_slerp(heading_quat, target_pose_quat, self._update_rate_heading_yaw)
        updated_heading_yaw = tf.transformations.euler_from_quaternion(updated_heading_quat)[2]

        return goal, path, updated_heading_yaw


    def get_goal_candidates(self, robot_pose, mapmsg, scanmsg, velodyne_to_map_transform):
        #  mimic
        self._costmap_callback(mapmsg)
        self._scan_callback(scanmsg)

        with self._lock_route_explore_state:
            if self._heading_yaw is None:
                robot_pose_quat = (robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w)
                self._heading_yaw = tf.transformations.euler_from_quaternion(robot_pose_quat)[2]
            heading_yaw = copy.copy(self._heading_yaw)

        with self._lock_costmap:
            resized_inflate_costmap = copy.copy(self._resized_inflate_costmap)
            self._update_goal_costmap_info = resized_inflate_costmap.costmap_info
            self._update_goal_reachable_area_contours = copy.copy(self._reachable_area_contours)
            self._update_goal_reachable_area_hierarchy = copy.copy(self._reachable_area_hierarchy)

        with self._lock_route_node_points:
            route_node_points = copy.copy(self._route_node_points)
            route_edge_points = copy.copy(self._route_edge_points)
            route_node_connections = copy.copy(self._route_node_connections)
            route_node_points_indices_edge_idx_dict = copy.copy(self._route_node_points_indices_edge_idx_dict)
            route_edge_idx_node_points_indices_dict = copy.copy(self._route_edge_idx_node_points_indices_dict)
            route_edge_idx_edge_points_indices_dict = copy.copy(self._route_edge_idx_edge_points_indices_dict)
            route_edge_point_idx_edge_idx_dict = copy.copy(self._route_edge_point_idx_edge_idx_dict)
            route_node_points_np_array = copy.copy(self._route_node_points_np_array)
            route_graph = copy.copy(self._route_graph)
            route_dt_map = copy.copy(self._route_dt_map)

        with self._lock_frontier_proximity_map:
            frontier_proximity_map = copy.copy(self._frontier_proximity_map)

        if (route_node_points_np_array is not None) and (len(route_node_points_np_array)>0):
            route_node_points_kd_tree = cKDTree(route_node_points_np_array)
        else:
            route_node_points_kd_tree = None

        _, _, _, _, _, _, _, _, goal_candidate_poses, _ = self.find_goal_candidates(robot_pose, heading_yaw, resized_inflate_costmap, self._update_goal_reachable_area_contours, self._update_goal_reachable_area_hierarchy, \
                                                                                route_node_points, route_edge_points, route_node_connections, route_node_points_indices_edge_idx_dict, route_edge_idx_node_points_indices_dict, \
                                                                                route_edge_idx_edge_points_indices_dict, route_edge_point_idx_edge_idx_dict, route_node_points_np_array, route_node_points_kd_tree, \
                                                                                route_graph, route_dt_map, frontier_proximity_map)
        return goal_candidate_poses


    def execute(self):
        start_time = time.perf_counter()

        if rospy.is_shutdown():
            rospy.logerr("ROS is not running.")
            return
        if not self._navigate_to_pose_client.wait_for_server():
            rospy.logerr("Cannot connect to move_base server.")
            return

        robot_pose = self.get_current_pose()

        with self._lock_route_explore_state:
            if (self._heading_yaw is None) or (self._route_explore_state==ExploreState.GO_ORIGIN):
                robot_pose_quat = (robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w)
                self._heading_yaw = tf.transformations.euler_from_quaternion(robot_pose_quat)[2]
            heading_yaw = copy.copy(self._heading_yaw)

        with self._lock_costmap:
            resized_inflate_costmap = copy.copy(self._resized_inflate_costmap)
            reachable_area_contours = copy.copy(self._reachable_area_contours)
            reachable_area_hierarchy = copy.copy(self._reachable_area_hierarchy)
            time_update_costmap = copy.copy(self._time_update_costmap)

        if time_update_costmap is None:
            rospy.loginfo("Cost map is not ready")
            return

        with self._lock_route_node_points:
            route_node_points = copy.copy(self._route_node_points)
            route_edge_points = copy.copy(self._route_edge_points)
            route_node_connections = copy.copy(self._route_node_connections)
            route_node_points_indices_edge_idx_dict = copy.copy(self._route_node_points_indices_edge_idx_dict)
            route_edge_idx_node_points_indices_dict = copy.copy(self._route_edge_idx_node_points_indices_dict)
            route_edge_idx_edge_points_indices_dict = copy.copy(self._route_edge_idx_edge_points_indices_dict)
            route_edge_point_idx_edge_idx_dict = copy.copy(self._route_edge_point_idx_edge_idx_dict)
            route_node_points_np_array = copy.copy(self._route_node_points_np_array)
            route_graph = copy.copy(self._route_graph)
            route_dt_map = copy.copy(self._route_dt_map)
            time_update_route_graph = copy.copy(self._time_update_route_graph)

        if time_update_route_graph is None:
            rospy.loginfo("route graph is not ready")
            return

        with self._lock_frontier_proximity_map:
            frontier_proximity_map = copy.copy(self._frontier_proximity_map)

        if frontier_proximity_map is None:
            rospy.loginfo("frontier proximity map is not ready")
            return

        # find goal candidates
        if (route_node_points_np_array is not None) and (len(route_node_points_np_array)>0):
            route_node_points_kd_tree = cKDTree(route_node_points_np_array)
        else:
            route_node_points_kd_tree = None

        update_goal_heading_pose, update_goal_coverage_polyhedron, update_goal_visible_area_contour, sample_points_in_polyhedron, sample_points_out_polyhedron, forward_branch_node_index, \
            next_forward_branch_node_index, pose_to_turn, goal_candidate_poses, has_forward_goal \
                = self.find_goal_candidates(robot_pose, heading_yaw, resized_inflate_costmap, reachable_area_contours, reachable_area_hierarchy, route_node_points, route_edge_points, route_node_connections, \
                                            route_node_points_indices_edge_idx_dict, route_edge_idx_node_points_indices_dict, route_edge_idx_edge_points_indices_dict, \
                                            route_edge_point_idx_edge_idx_dict, route_node_points_np_array, route_node_points_kd_tree, route_graph, route_dt_map, frontier_proximity_map)

        # debug visualization
        # networkx.draw(route_graph, with_labels=True)
        # matplotlib.pyplot.show(route_graph)

        with self._lock_route_explore_state:
            route_explore_state = copy.copy(self._route_explore_state)

            need_pause_possible_turn = copy.copy(self._need_pause_possible_turn)
            last_pause_possible_turn_point = copy.copy(self._last_pause_possible_turn_point)

            need_send_next_goal = copy.copy(self._need_send_next_goal)
            prev_send_goal_robot_pose = copy.copy(self._prev_send_goal_robot_pose)

            prev_check_stack_robot_pose = copy.copy(self._prev_check_stack_robot_pose)
            prev_check_stack_time = copy.copy(self._prev_check_stack_time)

        # check if pause possible turn is necessary
        if (route_explore_state==ExploreState.EXPLORE) and (self._use_pause_possible_turn) and (self._use_frontier_proximity) and (not need_pause_possible_turn) and (route_node_points_kd_tree is not None):
            _, robot_closest_route_node_index = route_node_points_kd_tree.query(np.array([robot_pose.position.x, robot_pose.position.y]), k=1)
            if route_graph.has_node(robot_closest_route_node_index):
                # project closest route node to robot forward moving path
                robot_closest_route_node_point = route_node_points[robot_closest_route_node_index]
                fixed_robot_closest_route_node_point = geometry_utils.project_point_on_robot_moving_path(robot_pose, robot_closest_route_node_point)

                # check closest route node is close to robot and far from last paused route node
                if (geometry_utils.calc_point_distance(fixed_robot_closest_route_node_point, robot_pose.position) < self._tolerance_robot_close_possible_turn) \
                        and ((last_pause_possible_turn_point is None)  \
                            or (geometry_utils.calc_point_distance(fixed_robot_closest_route_node_point, last_pause_possible_turn_point) > self._tolerance_robot_close_last_possible_turn)):

                    # check any frontier is close to closest route node
                    is_frontier_close = False
                    if frontier_proximity_map is not None:
                        map_x, map_y = costmap_utils.world_to_map(frontier_proximity_map.costmap_info, fixed_robot_closest_route_node_point.x, fixed_robot_closest_route_node_point.y)
                        if frontier_proximity_map.get_cost(map_x, map_y) > 0:
                            is_frontier_close = True

                    if is_frontier_close:
                        is_branch_route_node_close = False
                        robot_closest_route_node_neighbors_id_list = list(route_graph.neighbors(robot_closest_route_node_index))
                        if len(list(robot_closest_route_node_neighbors_id_list))==2:
                            # if route node has two edges and corner is close to perpendicular, pause and check route
                            edge_quat1 = geometry_utils.calc_point_direction_quaternion(fixed_robot_closest_route_node_point, route_node_points[robot_closest_route_node_neighbors_id_list[0]])
                            edge_quat2 = geometry_utils.calc_point_direction_quaternion(fixed_robot_closest_route_node_point, route_node_points[robot_closest_route_node_neighbors_id_list[1]])
                            edge_quat_diff =geometry_utils.calc_relative_orientation_quaternion(edge_quat1, edge_quat2)
                            edge_yaw_diff = tf.transformations.euler_from_quaternion(edge_quat_diff)[2]
                            if abs(math.pi/2.0 - abs(edge_yaw_diff))<self._possible_turn_perpendicular_yaw_tolerance:
                                is_branch_route_node_close = True
                        elif len(list(robot_closest_route_node_neighbors_id_list))>2:
                            # if route node has more than two edges, pause and check route
                            is_branch_route_node_close = True

                        if is_branch_route_node_close:
                            rospy.loginfo("call cancel goal because branch route node is close")
                            self._navigate_to_pose_client.cancel_all_goals()
                            self._navigate_to_pose_client.wait_for_result(timeout = rospy.Duration(2.0))

                            with self._lock_route_explore_state:
                                self._need_pause_possible_turn = True
                                self._last_pause_possible_turn_point = fixed_robot_closest_route_node_point

                            def pause_possible_turn():
                                self._wait_map_goal_update()
                                # wait constant time to make sure checking route finished after pausing possible turn
                                rospy.sleep(self._wait_after_pause_possible_turn)
                                with self._lock_route_explore_state:
                                    self._need_pause_possible_turn = False
                                    self._need_send_next_goal = True

                            # create thread to wait for map and goal update at possible turn
                            thread = threading.Thread(target=pause_possible_turn)
                            thread.start()
            else:
                rospy.logerr("Cannot find closest route node in graph, closest route node index = " + str(robot_closest_route_node_index))

        if (self._route_explore_state==ExploreState.EXPLORE) or (self._route_explore_state==ExploreState.EXPLORE_FORWARD) or (self._route_explore_state==ExploreState.GO_ORIGIN):
            # check if robot moved certain distance
            if (not need_send_next_goal) and (prev_send_goal_robot_pose is not None) and (has_forward_goal):
                prev_send_goal_move_distance = geometry_utils.calc_point_distance(prev_send_goal_robot_pose.position, robot_pose.position)
                if prev_send_goal_move_distance>self._update_goal_move_distance:
                    # rospy.loginfo("update and send goal, moved distance since last update goal = " + str(prev_send_goal_move_distance))
                    need_send_next_goal = True

            # check if robot stack certain time at close position
            if (not need_send_next_goal) and (prev_check_stack_robot_pose is not None) and (has_forward_goal):
                distance_since_last_check = geometry_utils.calc_point_distance(prev_check_stack_robot_pose.position, robot_pose.position)
                # rospy.loginfo("distance since last stack check = " + str(distance_since_last_check))
                if distance_since_last_check<self._stack_distance:
                    current_time = rospy.Time.now().to_sec()
                    time_since_last_check = current_time-prev_check_stack_time
                    # rospy.loginfo("time since last stack check = " + str(time_since_last_check))
                    if time_since_last_check>self._stack_time:
                        rospy.loginfo("robot stacked for target pose, try sending goal")
                        need_send_next_goal = True
                else:
                    # rospy.loginfo("robot does not stack for target pose")
                    prev_check_stack_robot_pose = robot_pose
                    prev_check_stack_time = rospy.Time.now().to_sec()

        # check if necessary to add landmark
        if self._route_explore_state==ExploreState.WAIT:
            with self._lock_landmark:
                if self._last_landmark_pose is None:
                    # add landmark at initial pose
                    resp = self._add_landmark_proxy(robot_pose, "")
                    self._next_landmark_neighbor_id = resp.landmark_id
                    self._last_landmark_pose = robot_pose
        elif (self._route_explore_state==ExploreState.EXPLORE) or (self._route_explore_state==ExploreState.EXPLORE_FORWARD):
            with self._lock_landmark:
                if self._last_landmark_pose is not None:
                    distance_since_last_add_landmark = geometry_utils.calc_point_distance(self._last_landmark_pose.position, robot_pose.position)
                    if distance_since_last_add_landmark>self._max_landmark_interval:
                        resp = self._add_landmark_proxy(robot_pose, self._next_landmark_neighbor_id)
                        self._next_landmark_neighbor_id = resp.landmark_id
                        self._last_landmark_pose = robot_pose

        # publish explore status
        with self._lock_route_explore_state:
            self._pose_to_turn = copy.copy(pose_to_turn)
            self._goal_candidate_poses = copy.copy(goal_candidate_poses)
            self._has_forward_goal = copy.copy(has_forward_goal)
            self._update_goal_costmap_info = copy.copy(resized_inflate_costmap.costmap_info)
            self._update_goal_reachable_area_contours = copy.copy(reachable_area_contours)
            self._update_goal_reachable_area_hierarchy = copy.copy(reachable_area_hierarchy)
            self._update_goal_visible_area_contour = copy.copy(update_goal_visible_area_contour)
            self._update_goal_coverage_polyhedron = copy.copy(update_goal_coverage_polyhedron)
            self._time_update_goal_candidate_poses = rospy.Time.now()

            self._need_send_next_goal = copy.copy(need_send_next_goal)

            self._prev_check_stack_robot_pose = copy.copy(prev_check_stack_robot_pose)
            self._prev_check_stack_time = copy.copy(prev_check_stack_time)

            if (self._route_explore_state==ExploreState.EXPLORE) or (self._route_explore_state==ExploreState.EXPLORE_FORWARD) or (self._route_explore_state==ExploreState.GO_ORIGIN) \
                    or (self._route_explore_state==ExploreState.OBSERVE_ONCE) or (self._route_explore_state==ExploreState.PAUSING):
                self.visualize_goal_candidates(update_goal_heading_pose, self._update_goal_coverage_polyhedron, sample_points_in_polyhedron, sample_points_out_polyhedron, route_node_points, \
                                            route_graph, forward_branch_node_index, next_forward_branch_node_index)

            # rospy.loginfo("explore status : " + str(self._route_explore_state))
            if (self._route_explore_state==ExploreState.EXPLORE) or (self._route_explore_state==ExploreState.EXPLORE_FORWARD):
                if self._need_send_next_goal and (not self._need_pause_possible_turn):
                    # rospy.loginfo("need to send next goal : " + str(self._need_send_next_goal))
                    if (self._has_forward_goal) and (len(self._goal_candidate_poses)>0):
                        def move_base_done_callback(status, result):
                            if status == GoalStatus.SUCCEEDED:
                                rospy.loginfo("move_base goal successed for target pose")
                            elif status == GoalStatus.PREEMPTED:
                                rospy.loginfo("move_base goal is canceled after it started executing for target pose")
                            else:
                                rospy.loginfo("move_base goal did not success for target pose")
                            with self._lock_route_explore_state:
                                self._need_send_next_goal = True

                        if (self._pose_to_turn is not None) and (self.is_pose_far_forward(robot_pose, self._pose_to_turn, self._tolerance_robot_arrive_pose_to_turn)) and (next_forward_branch_node_index is None):
                            # if pose to turn is far forward and do not have next forward route node, set pose to turn as next goal to stop at L-turn or T-turn
                            goal, ros_path, self._heading_yaw = self.create_goal_path(robot_pose, [self._pose_to_turn], self._heading_yaw)
                        else:
                            goal, ros_path, self._heading_yaw = self.create_goal_path(robot_pose, [self._goal_candidate_poses[0]], self._heading_yaw)
                        rospy.loginfo("call send_goal to " + str(goal.pose))
                        self._path_publisher.publish(ros_path)
                        self._navigate_to_pose_client.send_goal(goal, done_cb=move_base_done_callback)

                        self._need_send_next_goal = False
                        self._prev_send_goal_robot_pose = robot_pose

                        self._prev_check_stack_robot_pose = robot_pose
                        self._prev_check_stack_time = rospy.Time.now().to_sec()
                        # rospy.loginfo("sent next goal")
                    else:
                        rospy.loginfo("could not find next goal, try again later...")

                explore_status = self._create_cabot_explore_status(self._route_explore_state, self._update_goal_costmap_info, self._pose_to_turn, self._goal_candidate_poses, self._has_forward_goal, \
                                                                self._update_goal_reachable_area_contours, self._update_goal_reachable_area_hierarchy, self._update_goal_visible_area_contour, self._update_goal_coverage_polyhedron)
                # rospy.loginfo("Publish explore_status : " + json.dumps(explore_status))
                self._explore_status_publisher.publish(json.dumps(explore_status))
            elif self._route_explore_state==ExploreState.GO_ORIGIN:
                if self._need_send_next_goal:
                    def move_landmark_done_callback(status, result):
                        robot_pose = self.get_current_pose()
                        resp = self._search_path_proxy(robot_pose.position, Point(0,0,0))
                        path_poses = resp.path_pose_list

                        if len(path_poses)>1 or self.is_pose_far_forward(robot_pose, path_poses[0], self._tolerance_robot_arrive_pose_to_turn):
                            with self._lock_route_explore_state:
                                self._need_send_next_goal = True
                        else:
                            with self._lock_landmark:
                                resp = self._find_closest_landmark_id_proxy(robot_pose.position)
                                self._next_landmark_neighbor_id = resp.closest_landmark_id
                                self._last_landmark_pose = robot_pose

                            with self._lock_route_explore_state:
                                if status == GoalStatus.SUCCEEDED:
                                    self._route_explore_state = ExploreState.ARRIVE_ORIGIN
                                else:
                                    self._route_explore_state = ExploreState.WAIT
                                explore_status = self._create_cabot_explore_status(self._route_explore_state, self._update_goal_costmap_info, self._pose_to_turn, self._goal_candidate_poses, self._has_forward_goal, \
                                                                                self._update_goal_reachable_area_contours, self._update_goal_reachable_area_hierarchy, self._update_goal_visible_area_contour, self._update_goal_coverage_polyhedron)
                                # rospy.loginfo("Publish explore_status : " + json.dumps(explore_status))
                                self._explore_status_publisher.publish(json.dumps(explore_status))

                    resp = self._search_path_proxy(robot_pose.position, Point(0,0,0))
                    path_poses = resp.path_pose_list
                    if (len(path_poses)>1) and (geometry_utils.calc_point_distance(robot_pose.position, path_poses[0].position)<self._skip_close_first_goal_poses_distance):
                        goal, ros_path, self._heading_yaw = self.create_goal_path(robot_pose, path_poses[:2], self._heading_yaw)
                    else:
                        goal, ros_path, self._heading_yaw = self.create_goal_path(robot_pose, path_poses[:1], self._heading_yaw)
                    rospy.loginfo("call send_goal to " + str(goal.pose))
                    self._path_publisher.publish(ros_path)
                    self._navigate_to_pose_client.send_goal(goal, done_cb=move_landmark_done_callback)

                    self._need_send_next_goal = False
                    self._prev_send_goal_robot_pose = robot_pose

                    self._prev_check_stack_robot_pose = robot_pose
                    self._prev_check_stack_time = rospy.Time.now().to_sec()

                explore_status = self._create_cabot_explore_status(self._route_explore_state, self._update_goal_costmap_info, self._pose_to_turn, self._goal_candidate_poses, self._has_forward_goal, \
                                                                self._update_goal_reachable_area_contours, self._update_goal_reachable_area_hierarchy, self._update_goal_visible_area_contour, self._update_goal_coverage_polyhedron)
                # rospy.loginfo("Publish explore_status : " + json.dumps(explore_status))
                self._explore_status_publisher.publish(json.dumps(explore_status))
            elif self._route_explore_state==ExploreState.OBSERVE_ONCE:
                self._route_explore_state=ExploreState.WAIT
                explore_status = self._create_cabot_explore_status(self._route_explore_state, self._update_goal_costmap_info, self._pose_to_turn, self._goal_candidate_poses, self._has_forward_goal, \
                                                                self._update_goal_reachable_area_contours, self._update_goal_reachable_area_hierarchy, self._update_goal_visible_area_contour, self._update_goal_coverage_polyhedron)
                # rospy.loginfo("Publish explore_status : " + json.dumps(explore_status))
                self._explore_status_publisher.publish(json.dumps(explore_status))

        rospy.loginfo("Time to call execute : " + str(time.perf_counter() - start_time))


    def _create_cabot_explore_status(self, route_explore_status, costmap_info, pose_to_turn, goal_candidate_poses, has_forward_goal, reachable_area_contours, reachable_area_hierarchy, visible_area_contour, coverage_polyhedron):
        explore_status = {"status": route_explore_status.value, "costmap_info": {}, "pose_to_turn": {}, "goal_candidate_poses": [], "has_forward_goal": False, "reachable_area_contours": None, "reachable_area_hierarchy": None, \
                        "visible_area_contour": None, "polyhedron_points": None}

        if costmap_info is not None:
            explore_status["costmap_info"]["resolution"] = costmap_info.resolution
            explore_status["costmap_info"]["width"] = costmap_info.width
            explore_status["costmap_info"]["height"] = costmap_info.height
            explore_status["costmap_info"]["origin"] = {"position":{"x":costmap_info.origin.position.x, "y":costmap_info.origin.position.y}, \
                                                        "orientation":{"x":costmap_info.origin.orientation.x, "y":costmap_info.origin.orientation.y, "z":costmap_info.origin.orientation.z, "w":costmap_info.origin.orientation.w}}

        if pose_to_turn is not None:
            explore_status["pose_to_turn"]["position"] = {"x":pose_to_turn.position.x, "y":pose_to_turn.position.y}
            explore_status["pose_to_turn"]["orientation"] = {"x":pose_to_turn.orientation.x, "y":pose_to_turn.orientation.y, "z":pose_to_turn.orientation.z, "w":pose_to_turn.orientation.w}

        explore_status["goal_candidate_poses"] = [{"position":{"x":pose.position.x, "y":pose.position.y}, "orientation":{"x":pose.orientation.x, "y":pose.orientation.y, "z":pose.orientation.z, "w":pose.orientation.w}} for pose in goal_candidate_poses]

        explore_status["has_forward_goal"] = has_forward_goal

        if reachable_area_contours is not None:
            explore_status["reachable_area_contours"] = np.array(reachable_area_contours).tolist()

        if reachable_area_hierarchy is not None:
            explore_status["reachable_area_hierarchy"] = np.array(reachable_area_hierarchy).tolist()

        if visible_area_contour is not None:
            explore_status["visible_area_contour"] = visible_area_contour.tolist()

        if coverage_polyhedron is not None:
            explore_status["polyhedron_points"] = coverage_polyhedron.points.tolist()

        return explore_status


    def find_goal_candidates(self, robot_pose, heading_yaw, costmap, reachable_area_contours, reachable_area_hierarchy, route_node_points, route_edge_points, route_node_connections, route_node_points_indices_edge_idx_dict,
                            route_edge_idx_node_points_indices_dict, route_edge_idx_edge_points_indices_dict, route_edge_point_idx_edge_idx_dict, route_node_points_np_array, route_node_points_kd_tree, route_graph,
                            route_dt_map, frontier_proximity_map):
        heading_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, heading_yaw)

        heading_pose = copy.copy(robot_pose)
        heading_pose.orientation.x = heading_quat[0]
        heading_pose.orientation.y = heading_quat[1]
        heading_pose.orientation.z = heading_quat[2]
        heading_pose.orientation.w = heading_quat[3]

        # find node which is closest to robot pose
        if route_node_points_kd_tree is not None:
            _, robot_closest_route_node_index = route_node_points_kd_tree.query(np.array([robot_pose.position.x, robot_pose.position.y]), k=1)
        else:
            robot_closest_route_node_index = None

        # calculate polyhedron sensor range by using distance transform
        polyhedron_sensor_range = None
        if (self._use_dt_polyhedron_sensor_range) and (route_dt_map is not None) and (route_node_points_kd_tree is not None):
            polyhedron_sensor_range = geometry_utils.calc_dt_coverage_polyhedron_range(route_dt_map, route_node_points_kd_tree, route_node_points_np_array, robot_pose,
                                                                                        self._dt_polyhedron_closest_node_range, self._dt_polyhedron_eps)
        if polyhedron_sensor_range is None:
            polyhedron_sensor_range = self._default_polyhedron_sensor_range

        # calculate coverage polyhedron from pose to find goal
        coverage_polyhedron = geometry_utils.calc_coverage_polyhedron(costmap, self._lethal_cost_threshold, robot_pose, polyhedron_sensor_range, self._angles_sample_coverage_polyhedron,
                                                                    polyhedron_buffer_length=self._polyhedron_buffer_length, polyhedron_buffer_samples=self._polyhedron_buffer_samples)

        # calculate binary image of approximated reachable and visible area from where finding goal candidates
        # Note that all visible area should be reachable, but because reachable area is approximated contour,
        # bitwise image of reachable and visible area is used to remove visible and non-reachable area
        visible_area_contour = geometry_utils.calc_visible_area_contour(costmap, self._lethal_cost_threshold, robot_pose, self._sample_free_points_by_angle_range,
                                                                        self._angles_sample_visible_area_points)
        visible_area_contours = [visible_area_contour] if (visible_area_contour is not None) and (len(visible_area_contour)>0) else []
        binary_visible_area_contour_image = costmap_utils.calc_binary_contours_image(costmap.costmap_info, visible_area_contours)
        binary_reachable_area_contour_image = costmap_utils.calc_binary_hierarchy_contours_image(costmap.costmap_info, reachable_area_contours, reachable_area_hierarchy)
        binary_reachable_visible_area_contour_image = cv2.bitwise_and(binary_visible_area_contour_image, binary_reachable_area_contour_image)
        binary_reachable_visible_area_map = costmap_utils.Costmap(costmap.costmap_info, tuple(binary_reachable_visible_area_contour_image.flatten().astype(np.uint8)))

        # start sample free points to find next target
        if self._use_sample_free_points:
            free_points = geometry_utils.sample_visible_free_points_by_angles(costmap, self._lethal_cost_threshold, binary_reachable_visible_area_map, robot_pose, self._sample_free_points_by_angle_range,
                                                                            self._angles_sample_free_points, min_distance=self._sample_free_points_min_distance,
                                                                            unknown_area_margin=self._sample_free_points_by_angle_unknown_area_margin,
                                                                            lethal_area_margin=self._sample_free_points_by_angle_lethal_area_margin)
            # rospy.loginfo("finish sample free points for fixed angles, number of free_points = " + str(len(free_points)))

        # find goal canndidate points from sampled points
        has_forward_goal = False
        forward_branch_node_index = None # route node index which is in convex polyhedron and farthest shared node in shotest paths for all goals
        next_forward_branch_node_index = None # route node index which is connected with forward branch node in forward direction from robot
        pose_to_turn = None # pose to turn for found goal candidates
        if coverage_polyhedron is not None:
            binary_coverage_polyhedron_image = costmap_utils.calc_binary_polyhedron_image(costmap.costmap_info, coverage_polyhedron)
            binary_coverage_polyhedron_map = costmap_utils.Costmap(costmap.costmap_info, tuple(binary_coverage_polyhedron_image.flatten().astype(np.uint8)))
            ## calculate IDs of visible outside coverage polyhedron areas
            visible_contours_outside_polyhedron, visible_contours_outside_polyhedron_id_map = geometry_utils.calc_contours_outside_coverage_polyhedron(binary_coverage_polyhedron_map,
                                                                                                                                                    binary_reachable_visible_area_contour_image,
                                                                                                                                                    ignore_small_area_square_meters=self._ignore_small_area_square_meters)
            visible_contours_outside_polyhedron_id_image = np.array(visible_contours_outside_polyhedron_id_map.occupancy_grid_data, dtype=np.float32).reshape(costmap.height, costmap.width)
            ## calculate binary image for visible outside coverage polyhedron areas
            binary_reachable_visible_outside_coverage_polyhedron_image = (visible_contours_outside_polyhedron_id_image>0).astype(np.uint8)
            binary_reachable_visible_outside_coverage_polyhedron_map = costmap_utils.Costmap(costmap.costmap_info, tuple(binary_reachable_visible_outside_coverage_polyhedron_image.flatten().astype(np.uint8)))

            def _select_in_out_polyhedron_sample_points(points):
                points_in_polyhedron_selector = [False] * len(points)
                points_out_polyhedron_selector = [False] * len(points)
                for point_idx, point in enumerate(points):
                    if geometry_utils.is_point_in_area(binary_reachable_visible_outside_coverage_polyhedron_map, point):
                        points_out_polyhedron_selector[point_idx] = True
                    elif geometry_utils.is_point_in_area(binary_coverage_polyhedron_map, point):
                        points_in_polyhedron_selector[point_idx] = True
                points_in_polyhedron = list(itertools.compress(points, points_in_polyhedron_selector))
                points_out_polyhedron = list(itertools.compress(points, points_out_polyhedron_selector))
                return points_in_polyhedron, points_out_polyhedron

            ## select free sample points only for inside coverage polyhedron
            ## Note : free sample points are not necessary for outside coverage polyhedron, 
            ##        if visible area of outside coverage polyhdron do not have route sample points, centroid points of visible areas will be used.
            if self._use_sample_free_points:
                free_sample_points_in_polyhedron, _ = _select_in_out_polyhedron_sample_points(free_points)
            else:
                free_sample_points_in_polyhedron = []
            free_sample_points_out_polyhedron = []

            if self._use_filter_forward_turn_route and (robot_closest_route_node_index is not None):
                ## filter turn routes in forward direction from robot

                ### prepare variable to find turn routes in forward direction
                route_node_points_in_polyhedron_indices_set = set()
                for point_idx, point in enumerate(route_node_points):
                    if geometry_utils.is_point_in_area(binary_coverage_polyhedron_map, point):
                        route_node_points_in_polyhedron_indices_set.add(point_idx)

                route_node_points_indices_heading_route_node_yaw_diff_dict = {}
                for point_idx, point in enumerate(route_node_points):
                    robot_pose_route_node_quat = geometry_utils.calc_point_direction_quaternion(robot_pose.position, point)
                    heading_route_node_quat_diff = geometry_utils.calc_relative_orientation_quaternion(heading_quat, robot_pose_route_node_quat)
                    heading_route_node_yaw_diff = tf.transformations.euler_from_quaternion(heading_route_node_quat_diff)[2]
                    route_node_points_indices_heading_route_node_yaw_diff_dict[point_idx] = heading_route_node_yaw_diff

                ### start traverse routes from robot closest route node
                open_route_node_indices_set = set([robot_closest_route_node_index])
                close_route_node_indices_set = set()
                filter_forward_turn_route_node_indices_set = set([robot_closest_route_node_index])
                filter_forward_turn_route_edge_indices_set = set()
                while len(open_route_node_indices_set)>0:
                    open_route_node_index = open_route_node_indices_set.pop()

                    for open_route_node_connected_index in route_node_connections[open_route_node_index]:
                        if open_route_node_connected_index in close_route_node_indices_set:
                            continue

                        #### select routes that satisfy following conditions as routes in forward direction
                        #### 1. both start and end points of route are not in coverage polyhedron
                        #### 2. either start or end points of route is within forward angle from robot
                        is_forward_open_route = False
                        if (open_route_node_index not in route_node_points_in_polyhedron_indices_set) and (open_route_node_connected_index not in route_node_points_in_polyhedron_indices_set):
                            heading_open_route_start_yaw_diff = route_node_points_indices_heading_route_node_yaw_diff_dict[open_route_node_index]
                            heading_open_route_end_yaw_diff = route_node_points_indices_heading_route_node_yaw_diff_dict[open_route_node_connected_index]

                            if (math.cos(heading_open_route_start_yaw_diff)>0) and (math.cos(heading_open_route_end_yaw_diff)>0) \
                                    and (abs(heading_open_route_start_yaw_diff)<self._max_abs_yaw_forward_goal) or (abs(heading_open_route_end_yaw_diff)<self._max_abs_yaw_forward_goal):
                                is_forward_open_route = True

                        if is_forward_open_route:
                            #### if route is in forward direction, check if the route direction is close to robot pose direction
                            open_route_quat = geometry_utils.calc_point_direction_quaternion(route_node_points[open_route_node_index], route_node_points[open_route_node_connected_index])
                            heading_open_route_quat_diff = geometry_utils.calc_relative_orientation_quaternion(heading_quat, open_route_quat)
                            abs_heading_open_route_yaw_diff = abs(tf.transformations.euler_from_quaternion(heading_open_route_quat_diff)[2])
                            if abs_heading_open_route_yaw_diff<self._max_abs_yaw_forward_goal:
                                filter_forward_turn_route_node_indices_set.add(open_route_node_connected_index)
                                filter_forward_turn_route_edge_indices_set.add(route_node_points_indices_edge_idx_dict[(open_route_node_index, open_route_node_connected_index)])
                        else:
                            #### sample all routes which are not in forward direction
                            filter_forward_turn_route_node_indices_set.add(open_route_node_connected_index)
                            filter_forward_turn_route_edge_indices_set.add(route_node_points_indices_edge_idx_dict[(open_route_node_index, open_route_node_connected_index)])

                        open_route_node_indices_set.add(open_route_node_connected_index)

                    close_route_node_indices_set.add(open_route_node_index)

                ### sample route points without turn routes in forward direction
                route_node_sample_points = [route_node_points[node_point_idx] for node_point_idx in filter_forward_turn_route_node_indices_set]
                route_edge_sample_points = [route_edge_points[edge_point_idx] for edge_idx in filter_forward_turn_route_edge_indices_set for edge_point_idx in route_edge_idx_edge_points_indices_dict[edge_idx]]
            else:
                ## sample all route points
                route_node_sample_points = route_node_points
                route_edge_sample_points = route_edge_points

            ## select route sample points for inside and outside coverage polyhedron
            route_node_sample_points_in_polyhedron, route_node_sample_points_out_polyhedron = _select_in_out_polyhedron_sample_points(route_node_sample_points)
            route_edge_sample_points_in_polyhedron, route_edge_sample_points_out_polyhedron = _select_in_out_polyhedron_sample_points(route_edge_sample_points)

            route_sample_points_out_polyhedron = route_node_sample_points_out_polyhedron + route_edge_sample_points_out_polyhedron

            if self._use_cluster_point_outside_polyhedron_free_area:
                ## cluster sample points by using visible area outside coverage polyhedron
                clustered_sample_points_list, clustered_sample_points_contour_id_set = geometry_utils.cluster_point_by_contour_id_map(route_sample_points_out_polyhedron, visible_contours_outside_polyhedron_id_map)
                # rospy.loginfo("finish cluster_point_by_contour_id_map, number of clustered_sample_points_list = " + str(len(clustered_sample_points_list)))

                visible_contours_outside_polyhedron_id_list = list(range(1, len(visible_contours_outside_polyhedron)+1))
                unsampled_visible_contours_outside_polyhedron_id_set = set(visible_contours_outside_polyhedron_id_list) - clustered_sample_points_contour_id_set
                if len(unsampled_visible_contours_outside_polyhedron_id_set)>0:
                    ## collect visible area outside coverage polyhedron which do not have any sample points
                    unsampled_visible_contours_outside_polyhedron = []
                    for visible_contours_outside_polyhedron_idx in range(len(visible_contours_outside_polyhedron)):
                        visible_contours_outside_polyhedron_id = visible_contours_outside_polyhedron_idx + 1
                        if visible_contours_outside_polyhedron_id in unsampled_visible_contours_outside_polyhedron_id_set:
                            unsampled_visible_contours_outside_polyhedron.append(visible_contours_outside_polyhedron[visible_contours_outside_polyhedron_idx])

                    if len(unsampled_visible_contours_outside_polyhedron)>0:
                        ## calculate centroid of visible area outside coverage polyhedron which do not have any sample points, and add to cluster sample points
                        unsampled_visible_contours_outside_polyhedron_centroid_list = geometry_utils.calc_contours_centroids(costmap.costmap_info, unsampled_visible_contours_outside_polyhedron)
                        for point in unsampled_visible_contours_outside_polyhedron_centroid_list:
                            clustered_sample_points_list.append([point])

                ## remove sample points in coverage polyhedron which are connected with found goals in forward contours outside polyhedron (for preventing reading forward goal at L-turn or T-turn)

                ## calculate contours outside polyhedron which are in forward direction from robot
                forward_visible_contours_outside_polyhedron_indices = geometry_utils.select_forward_contours(visible_contours_outside_polyhedron, robot_pose)
                forward_visible_contours_outside_polyhedron = [visible_contours_outside_polyhedron[idx] for idx in forward_visible_contours_outside_polyhedron_indices]

                binary_forward_visible_contours_outside_polyhedron_image = costmap_utils.calc_binary_contours_image(costmap.costmap_info, forward_visible_contours_outside_polyhedron)
                binary_forward_visible_contours_outside_polyhedron_map = costmap_utils.Costmap(costmap.costmap_info, tuple(binary_forward_visible_contours_outside_polyhedron_image.flatten().astype(np.uint8)))

                ### remove route nodes and edge points which are connected with forward outside of polyhedron

                #### find route edges passing inside of polyhedron or passing forward outside of polyhedron
                route_edge_indices_in_polyhedron_set = set()
                route_edge_indices_forward_out_polyhedron_set = set()
                for route_edge_point_idx, route_edge_point in enumerate(route_edge_points):
                    edge_idx = route_edge_point_idx_edge_idx_dict[route_edge_point_idx]

                    if (edge_idx not in route_edge_indices_in_polyhedron_set) and (geometry_utils.is_point_in_area(binary_coverage_polyhedron_map, route_edge_point)):
                        route_edge_indices_in_polyhedron_set.add(edge_idx)
                    if (edge_idx not in route_edge_indices_forward_out_polyhedron_set) and (geometry_utils.is_point_in_area(binary_forward_visible_contours_outside_polyhedron_map, route_edge_point)):
                        route_edge_indices_forward_out_polyhedron_set.add(edge_idx)

                #### find route node points and route edge points inside of polyhedron that satisfy one of following conditions and are in forward direction from robot
                # 1. find route nodes inside of polyhedron which are connected with route nodes forward outside of polyhedron
                # 2. find route nodes inside of polyhedron and passing forward outside of polyhedron
                # 3. find route nodes/edges inside of polyhedron which are connected with route nodes found in step 1 or step 2 without route branch
                # 4. find route edges passing inside of polyhedron which pass forward outside of polyhedron or connect with route nodes outside of polyhedron
                remove_route_node_indices_in_polyhedron_set = set()
                remove_route_edge_indices_in_polyhedron_set = set()
                for route_node_idx, route_node_point in enumerate(route_node_points):
                    if geometry_utils.is_point_in_area(binary_coverage_polyhedron_map, route_node_point):
                        connected_route_node_indices = route_node_connections[route_node_idx]

                        ##### 1. find route nodes inside of polyhedron which are connected with route nodes forward outside of polyhedron
                        connected_route_node_outside_polyhedron_indices = []
                        for connected_route_node_index in connected_route_node_indices:
                            connected_route_node = route_node_points[connected_route_node_index]
                            if geometry_utils.is_point_in_area(binary_forward_visible_contours_outside_polyhedron_map, connected_route_node):
                                connected_route_node_outside_polyhedron_indices.append(connected_route_node_index)
                        if len(connected_route_node_outside_polyhedron_indices)>0:
                            remove_route_node_indices_in_polyhedron_set.add(route_node_idx)
                            for connected_route_node_outside_polyhedron_index in connected_route_node_outside_polyhedron_indices:
                                remove_route_edge_indices_in_polyhedron_set.add(route_node_points_indices_edge_idx_dict[(route_node_idx, connected_route_node_outside_polyhedron_index)])

                        ##### 2. find route nodes inside of polyhedron and passing forward outside of polyhedron
                        for connected_route_node_index in connected_route_node_indices:
                            connected_route_edge_index = route_node_points_indices_edge_idx_dict[(route_node_idx, connected_route_node_index)]
                            if connected_route_edge_index in route_edge_indices_forward_out_polyhedron_set:
                                remove_route_node_indices_in_polyhedron_set.add(route_node_idx)
                                remove_route_edge_indices_in_polyhedron_set.add(connected_route_edge_index)
                ##### 3. find route nodes/edges inside of polyhedron which are connected with route nodes found in step 1 or step 2 without route branch
                updated_remove_route_node_indices_in_polyhedron_set = copy.copy(remove_route_node_indices_in_polyhedron_set)
                while len(updated_remove_route_node_indices_in_polyhedron_set)>0:
                    prev_updated_remove_route_node_indices_in_polyhedron_set = copy.copy(updated_remove_route_node_indices_in_polyhedron_set)
                    updated_remove_route_node_indices_in_polyhedron_set = set()

                    updated_remove_route_edge_indices_in_polyhedron_set = set()
                    for remove_route_node_index in prev_updated_remove_route_node_indices_in_polyhedron_set:
                        connected_route_node_indices = route_node_connections[remove_route_node_index]
                        if len(connected_route_node_indices)==2:
                            for connected_route_node_index in connected_route_node_indices:
                                connected_route_node = route_node_points[connected_route_node_index]
                                if geometry_utils.is_point_in_area(binary_coverage_polyhedron_map, connected_route_node):
                                    if connected_route_node_index not in remove_route_node_indices_in_polyhedron_set:
                                        updated_remove_route_node_indices_in_polyhedron_set.add(connected_route_node_index)

                                connected_route_edge_index = route_node_points_indices_edge_idx_dict[(remove_route_node_index, connected_route_node_index)]
                                if connected_route_edge_index not in remove_route_edge_indices_in_polyhedron_set:
                                    updated_remove_route_edge_indices_in_polyhedron_set.add(connected_route_edge_index)

                    remove_route_node_indices_in_polyhedron_set |= updated_remove_route_node_indices_in_polyhedron_set
                    remove_route_edge_indices_in_polyhedron_set |= updated_remove_route_edge_indices_in_polyhedron_set
                ##### 4. find route edges passing inside of polyhedron which pass forward outside of polyhedron or connect with route nodes outside of polyhedron
                for in_polyhedron_edge_idx in route_edge_indices_in_polyhedron_set:
                    if (in_polyhedron_edge_idx not in remove_route_edge_indices_in_polyhedron_set) and (in_polyhedron_edge_idx in route_edge_indices_forward_out_polyhedron_set):
                        remove_route_edge_indices_in_polyhedron_set.add(in_polyhedron_edge_idx)

                    if (in_polyhedron_edge_idx not in remove_route_edge_indices_in_polyhedron_set):
                        in_polyhedron_edge_node_indices = route_edge_idx_node_points_indices_dict[in_polyhedron_edge_idx]
                        for in_polyhedron_edge_node_index in in_polyhedron_edge_node_indices:
                            in_polyhedron_edge_node_point = route_node_points[in_polyhedron_edge_node_index]
                            if geometry_utils.is_point_in_area(binary_forward_visible_contours_outside_polyhedron_map, in_polyhedron_edge_node_point):
                                remove_route_edge_indices_in_polyhedron_set.add(in_polyhedron_edge_idx)
                                break

                #### if found route node points are in forward direction from robot, remove them from route sample points inside polyhedron
                # rospy.loginfo("number of route node sample points in polyhedron before removing points connected with found goals = " + str(len(route_node_sample_points_in_polyhedron)))
                for remove_route_node_index_in_polyhedron in remove_route_node_indices_in_polyhedron_set:
                    remove_route_node_point = route_node_points[remove_route_node_index_in_polyhedron]
                    if remove_route_node_point in route_node_sample_points_in_polyhedron:
                        ##### confirm remove sample points are forward direction from robot
                        robot_to_node_quat = geometry_utils.calc_point_direction_quaternion(robot_pose.position, remove_route_node_point)
                        heading_quat_robot_to_node_quat_diff = geometry_utils.calc_relative_orientation_quaternion(heading_quat, robot_to_node_quat)
                        heading_yaw_robot_to_node_yaw_diff = tf.transformations.euler_from_quaternion(heading_quat_robot_to_node_quat_diff)[2]
                        if math.cos(heading_yaw_robot_to_node_yaw_diff)>0:
                            route_node_sample_points_in_polyhedron.remove(remove_route_node_point)
                # rospy.loginfo("number of route node sample points in polyhedron after removing points connected with found goals = " + str(len(route_node_sample_points_in_polyhedron)))

                #### if found route edge points are in forward direction from robot, remove them from route sample points inside polyhedron
                # rospy.loginfo("number of route edge sample points in polyhedron before removing points connected with found goals = " + str(len(route_edge_sample_points_in_polyhedron)))
                for remove_route_edge_index_in_polyhedron in remove_route_edge_indices_in_polyhedron_set:
                    remove_route_edge_points_indices = route_edge_idx_edge_points_indices_dict[remove_route_edge_index_in_polyhedron]
                    for remove_route_edge_point_index in remove_route_edge_points_indices:
                        remove_route_edge_point = route_edge_points[remove_route_edge_point_index]
                        if remove_route_edge_point in route_edge_sample_points_in_polyhedron:
                            ##### confirm remove sample points are forward direction from robot
                            robot_to_edge_point_quat = geometry_utils.calc_point_direction_quaternion(robot_pose.position, remove_route_edge_point)
                            heading_quat_robot_to_edge_point_quat_diff = geometry_utils.calc_relative_orientation_quaternion(heading_quat, robot_to_edge_point_quat)
                            heading_yaw_robot_to_edge_point_yaw_diff = tf.transformations.euler_from_quaternion(heading_quat_robot_to_edge_point_quat_diff)[2]
                            if math.cos(heading_yaw_robot_to_edge_point_yaw_diff)>0:
                                route_edge_sample_points_in_polyhedron.remove(remove_route_edge_point)
                # rospy.loginfo("number of route edge sample points in polyhedron after removing points connected with found goals = " + str(len(route_edge_sample_points_in_polyhedron)))
            else:
                ## cluster sample points by using coverage oplyhedron
                clustered_sample_points_list = geometry_utils.cluster_point_by_coverage_polyhedron(costmap, self._lethal_cost_threshold, route_sample_points_out_polyhedron, coverage_polyhedron)
                ## rospy.loginfo("finish cluster_point_by_coverage_polyhedron, number of clustered_sample_points_list = " + str(len(clustered_sample_points_list)))

            ### find farthest point for heading route direction in each cluster
            goal_candidate_point_list = []
            for points_list in clustered_sample_points_list:
                farthest_point_idx = geometry_utils.select_farthest_point_idx_with_forward_priority(heading_pose, points_list, max_abs_yaw_forward_point=self._max_abs_yaw_forward_goal)
                goal_candidate_point_list.append(points_list[farthest_point_idx])
            # rospy.loginfo("finish select goal candidate point in each clustered points, number of goal candidate points = " + str(len(goal_candidate_point_list)))

            ## check if pose to turn exists for found goal candidate points
            if (len(goal_candidate_point_list)>0) and (route_node_points_kd_tree is not None) and (robot_closest_route_node_index is not None):
                ### find nodes which are closest to goal candidate points
                goal_candidate_points_close_node_indices = []
                for goal_candidate_point in goal_candidate_point_list:
                    _, close_route_node_point_index = route_node_points_kd_tree.query(np.array([goal_candidate_point.x, goal_candidate_point.y]), k=1)
                    goal_candidate_points_close_node_indices.append(close_route_node_point_index)

                ### find shared nodes in shotest paths from robot to all goals
                shortest_paths_shared_nodes_set = set()
                for goal_candidate_point_close_node_index in goal_candidate_points_close_node_indices:
                    try:
                        # rospy.loginfo("find shortest path from " + str(robot_closest_route_node_index) + " to " + str(goal_candidate_point_close_node_index))
                        shortest_path = networkx.shortest_path(route_graph, source=robot_closest_route_node_index, target=goal_candidate_point_close_node_index, weight="distance")
                        # rospy.loginfo("found shortest path : " + str(shortest_path))

                        if len(shortest_paths_shared_nodes_set)==0:
                            shortest_paths_shared_nodes_set = set(shortest_path)
                        else:
                            shortest_paths_shared_nodes_set = shortest_paths_shared_nodes_set & set(shortest_path)
                    except Exception as e:
                        rospy.logerr("Exception to find shortest path " + str(e))
                        shortest_paths_shared_nodes_set = set()
                        break
                # rospy.loginfo("shared nodes set in shortest paths  : " + str(shortest_paths_shared_nodes_set))

                ### find forward branch node which is in convex polyhedron and farthest shared node in shotest paths for all goals
                if len(shortest_paths_shared_nodes_set)>0:
                    max_forward_shared_node_distance = sys.float_info.min
                    for shortest_paths_shared_node in shortest_paths_shared_nodes_set:
                        shortest_paths_shared_node_point = route_node_points[shortest_paths_shared_node]
                        if geometry_utils.is_point_in_area(binary_reachable_visible_area_map, shortest_paths_shared_node_point) \
                                and geometry_utils.is_point_in_area(binary_coverage_polyhedron_map, shortest_paths_shared_node_point):
                            node_distance = geometry_utils.calc_point_distance(robot_pose.position, shortest_paths_shared_node_point)

                            robot_to_node_quat = geometry_utils.calc_point_direction_quaternion(robot_pose.position, shortest_paths_shared_node_point)
                            heading_quat_robot_to_node_quat_diff = geometry_utils.calc_relative_orientation_quaternion(heading_quat, robot_to_node_quat)
                            heading_yaw_robot_to_node_yaw_diff = tf.transformations.euler_from_quaternion(heading_quat_robot_to_node_quat_diff)[2]

                            if (math.cos(heading_yaw_robot_to_node_yaw_diff)>0) or (node_distance<self._tolerance_robot_arrive_pose_to_turn):
                                forward_node_distance = node_distance * math.cos(heading_yaw_robot_to_node_yaw_diff)
                                if forward_node_distance>max_forward_shared_node_distance:
                                    forward_branch_node_index = shortest_paths_shared_node
                                    max_forward_shared_node_distance = forward_node_distance

                    if (forward_branch_node_index is not None) and (geometry_utils.is_point_in_area(binary_reachable_visible_area_map, route_node_points[forward_branch_node_index])):
                        ### project forward branch node to robot forward moving path
                        fixed_forward_branch_node_point = geometry_utils.project_point_on_robot_moving_path(robot_pose, route_node_points[forward_branch_node_index])
                        if not geometry_utils.is_point_in_area(binary_reachable_visible_area_map, fixed_forward_branch_node_point):
                            # if projected point is not visible, use original point
                            fixed_forward_branch_node_point = route_node_points[forward_branch_node_index]
                        # rospy.loginfo("fixed_forward_branch_node_point = " + str(fixed_forward_branch_node_point))

                        ### find route node which is connected with forward branch node in forward direction from robot
                        min_abs_yaw_from_forward_branch_node = sys.float_info.max
                        for connected_node_idx in route_node_connections[forward_branch_node_index]:
                            connected_node_point = route_node_points[connected_node_idx]
                            connected_node_quat = geometry_utils.calc_point_direction_quaternion(fixed_forward_branch_node_point, connected_node_point)
                            heading_connected_node_quat_diff = geometry_utils.calc_relative_orientation_quaternion(heading_quat, connected_node_quat)
                            heading_connected_node_yaw_diff = tf.transformations.euler_from_quaternion(heading_connected_node_quat_diff)[2]

                            if math.cos(heading_connected_node_yaw_diff)>0:
                                abs_yaw_diff = abs(heading_connected_node_yaw_diff)
                                if abs_yaw_diff<self._max_abs_yaw_forward_goal and abs_yaw_diff<min_abs_yaw_from_forward_branch_node:
                                    next_forward_branch_node_index = connected_node_idx
                                    min_abs_yaw_from_forward_branch_node = abs_yaw_diff

                        pose_to_turn = Pose()
                        pose_to_turn.position.x = fixed_forward_branch_node_point.x
                        pose_to_turn.position.y = fixed_forward_branch_node_point.y
                        q = geometry_utils.calc_point_direction_quaternion(robot_pose.position, fixed_forward_branch_node_point)
                        pose_to_turn.orientation.x = q[0]
                        pose_to_turn.orientation.y = q[1]
                        pose_to_turn.orientation.z = q[2]
                        pose_to_turn.orientation.w = q[3]
                        # rospy.loginfo("pose_to_turn : " + str(pose_to_turn))

            ## if forward or backward goal are not found from outside of coverage polyhedron, find forward or/and backward goal from inside of coverage polyhedron
            forward_goal_candidate_point_idx, backward_goal_candidate_point_idx, _ = geometry_utils.select_forward_backward_farthest_point_idx(robot_pose, goal_candidate_point_list,
                                                                                                                                            max_abs_yaw_forward_point=self._max_abs_yaw_forward_goal,
                                                                                                                                            min_abs_yaw_backward_point=self._min_abs_yaw_backward_goal)
            if (forward_goal_candidate_point_idx is None) or (backward_goal_candidate_point_idx is None):
                def _select_in_polyhedron_far_sample_points(points):
                    points_selector = []
                    for point in points:
                        is_far_point = False

                        ### check if point is goal candidate by frontier proximity
                        if self._use_frontier_proximity:
                            sample_point_map_x, sample_point_map_y = costmap_utils.world_to_map(frontier_proximity_map.costmap_info, point.x, point.y)
                            if frontier_proximity_map.get_cost(sample_point_map_x, sample_point_map_y) > 0:
                                is_far_point = True

                        ### check if point is goal candidate by distance from robot
                        if not is_far_point:
                            point_quat = geometry_utils.calc_point_direction_quaternion(robot_pose.position, point)
                            heading_point_quat_diff = geometry_utils.calc_relative_orientation_quaternion(heading_quat, point_quat)
                            heading_point_yaw_diff = tf.transformations.euler_from_quaternion(heading_point_quat_diff)[2]
                            point_distance = geometry_utils.calc_point_distance(robot_pose.position, point)
                            abs_cos_point_distance = point_distance * abs(math.cos(heading_point_yaw_diff))

                            if abs_cos_point_distance>self._min_distance_goal_inside_coverage_polyhedron:
                               is_far_point = True

                        points_selector.append(is_far_point)
                    far_points = list(itertools.compress(points, points_selector))
                    return far_points

                ### find goal from route node sample points at first
                far_route_node_sample_points_in_polyhedron = _select_in_polyhedron_far_sample_points(route_node_sample_points_in_polyhedron)
                forward_point_idx, backward_point_idx, _ = geometry_utils.select_forward_backward_farthest_point_idx(robot_pose, far_route_node_sample_points_in_polyhedron,
                                                                                                                    max_abs_yaw_forward_point=self._max_abs_yaw_forward_goal,
                                                                                                                    min_abs_yaw_backward_point=self._min_abs_yaw_backward_goal)
                forward_point = far_route_node_sample_points_in_polyhedron[forward_point_idx] if forward_point_idx is not None else None
                backward_point = far_route_node_sample_points_in_polyhedron[backward_point_idx] if backward_point_idx is not None else None

                ### find goal from route edge sample points if forward/backward goals are not found from route node sample points
                if ((forward_goal_candidate_point_idx is None) and (forward_point is None)) or ((backward_goal_candidate_point_idx is None) and (backward_point is None)):
                    far_route_edge_sample_points_in_polyhedron = _select_in_polyhedron_far_sample_points(route_edge_sample_points_in_polyhedron)
                    forward_point_idx, backward_point_idx, _ = geometry_utils.select_forward_backward_farthest_point_idx(robot_pose, far_route_edge_sample_points_in_polyhedron,
                                                                                                                        max_abs_yaw_forward_point=self._max_abs_yaw_forward_goal,
                                                                                                                        min_abs_yaw_backward_point=self._min_abs_yaw_backward_goal)
                    forward_point = far_route_edge_sample_points_in_polyhedron[forward_point_idx] if forward_point_idx is not None else None
                    backward_point = far_route_edge_sample_points_in_polyhedron[backward_point_idx] if backward_point_idx is not None else None

                ### find goal from free sample points if forward/backward goals are not found from route sample points
                if (self._use_sample_free_points) and ((forward_goal_candidate_point_idx is None) and (forward_point is None)) or ((backward_goal_candidate_point_idx is None) and (backward_point is None)):
                    far_free_sample_points_in_polyhedron = _select_in_polyhedron_far_sample_points(free_sample_points_in_polyhedron)
                    forward_point_idx, backward_point_idx, _ = geometry_utils.select_forward_backward_farthest_point_idx(robot_pose, far_free_sample_points_in_polyhedron,
                                                                                                                        max_abs_yaw_forward_point=self._max_abs_yaw_forward_goal,
                                                                                                                        min_abs_yaw_backward_point=self._min_abs_yaw_backward_goal)
                    if (forward_goal_candidate_point_idx is None) and (forward_point is None):
                        forward_point = far_free_sample_points_in_polyhedron[forward_point_idx] if forward_point_idx is not None else None
                    if (backward_goal_candidate_point_idx is None) and (backward_point is None):
                        backward_point = far_free_sample_points_in_polyhedron[backward_point_idx] if backward_point_idx is not None else None

                if (forward_goal_candidate_point_idx is None) and (forward_point is not None):
                    goal_candidate_point_list.append(forward_point)
                    # rospy.loginfo("forward goal inside coverage polyhedron is added.")
                if (backward_goal_candidate_point_idx is None) and (backward_point is not None):
                    goal_candidate_point_list.append(backward_point)
                    # rospy.loginfo("backward goal inside coverage polyhedron is added.")

            ## sort goal candidate points
            sorted_goal_candidate_point_list, has_forward_goal = geometry_utils.sort_point_list_clockwise(robot_pose, goal_candidate_point_list,
                                                                                                        max_abs_yaw_forward_point=self._max_abs_yaw_forward_goal,
                                                                                                        min_abs_yaw_backward_point=self._min_abs_yaw_backward_goal,
                                                                                                        filter_forward_points=True)
        else:
            route_node_sample_points = [route_node_points[idx] for idx in geometry_utils.select_points_in_area(binary_reachable_visible_area_map, route_node_points)]
            route_edge_sample_points = [route_edge_points[idx] for idx in geometry_utils.select_points_in_area(binary_reachable_visible_area_map, route_edge_points)]

            ## if coverage polyhedron is not found, find only forward and backward goal
            route_node_sample_points_in_polyhedron = []
            route_edge_sample_points_in_polyhedron = []
            route_sample_points_out_polyhedron = route_node_sample_points + route_edge_sample_points
            free_sample_points_in_polyhedron = []
            if self._use_sample_free_points:
                free_sample_points_out_polyhedron = free_points
            else:
                free_sample_points_out_polyhedron = []

            ## find goal from route sample points at first
            forward_point_idx, backward_point_idx, _ = geometry_utils.select_forward_backward_farthest_point_idx(heading_pose, route_sample_points_out_polyhedron,
                                                                                                                max_abs_yaw_forward_point=self._max_abs_yaw_forward_goal,
                                                                                                                min_abs_yaw_backward_point=self._min_abs_yaw_backward_goal)
            forward_point = route_sample_points_out_polyhedron[forward_point_idx] if forward_point_idx is not None else None
            backward_point = route_sample_points_out_polyhedron[backward_point_idx] if backward_point_idx is not None else None

            ## find goal from free sample points if forward/backward goals are not found from route sample points
            if (forward_point is None) or (backward_point is None):
                forward_point_idx, backward_point_idx, _ = geometry_utils.select_forward_backward_farthest_point_idx(heading_pose, free_sample_points_out_polyhedron,
                                                                                                                    max_abs_yaw_forward_point=self._max_abs_yaw_forward_goal,
                                                                                                                    min_abs_yaw_backward_point=self._min_abs_yaw_backward_goal)
                if forward_point is None:
                    forward_point = free_sample_points_out_polyhedron[forward_point_idx] if forward_point_idx is not None else None
                if backward_point is None:
                    backward_point = free_sample_points_out_polyhedron[backward_point_idx] if backward_point_idx is not None else None

            sorted_goal_candidate_point_list = []
            if forward_point is not None:
                sorted_goal_candidate_point_list.append(forward_point)
                has_forward_goal = True
            if backward_point is not None:
                sorted_goal_candidate_point_list.append(backward_point)

        # create goal candidate pose
        sorted_goal_candidate_poses = []
        for goal_candidate_point in sorted_goal_candidate_point_list:
            goal_candidate_pose = Pose()
            goal_candidate_pose.position.x = goal_candidate_point.x
            goal_candidate_pose.position.y = goal_candidate_point.y
            q = geometry_utils.calc_point_direction_quaternion(robot_pose.position, goal_candidate_pose.position)
            goal_candidate_pose.orientation.x = q[0]
            goal_candidate_pose.orientation.y = q[1]
            goal_candidate_pose.orientation.z = q[2]
            goal_candidate_pose.orientation.w = q[3]
            sorted_goal_candidate_poses.append(goal_candidate_pose)

        sample_points_in_polyhedron = route_node_sample_points_in_polyhedron + route_edge_sample_points_in_polyhedron + free_sample_points_in_polyhedron
        sample_points_out_polyhedron = route_sample_points_out_polyhedron + free_sample_points_out_polyhedron

        return heading_pose, coverage_polyhedron, visible_area_contour, sample_points_in_polyhedron, sample_points_out_polyhedron, forward_branch_node_index, \
            next_forward_branch_node_index, pose_to_turn, sorted_goal_candidate_poses, has_forward_goal


    def update_goal_candidates_after_turn(self, turn_direction):
        rospy.loginfo("update goal candidates after turn, turn_direction = " + str(turn_direction))

        with self._lock_route_explore_state:
            self._heading_yaw = None

            if turn_direction=="right":
                first_goal = self._goal_candidate_poses.pop(0)
                self._goal_candidate_poses.append(first_goal)
            elif turn_direction=="left":
                last_goal = self._goal_candidate_poses.pop()
                self._goal_candidate_poses.insert(0, last_goal)
            else:
                rospy.logerr("invalid input : " + str(turn_direction))

            self._route_explore_state = ExploreState.WAIT

            explore_status = self._create_cabot_explore_status(self._route_explore_state, self._update_goal_costmap_info, self._pose_to_turn, self._goal_candidate_poses, self._has_forward_goal, \
                                                            self._update_goal_reachable_area_contours, self._update_goal_reachable_area_hierarchy, self._update_goal_visible_area_contour, self._update_goal_coverage_polyhedron)
            self._explore_status_publisher.publish(json.dumps(explore_status))


    def visualize_goal_candidates(self, heading_pose, hull_polyhedron, sample_points_in_polyhedron, sample_points_out_polyhedron, route_node_points, route_graph,
                                forward_branch_node_index, next_forward_branch_node_index):
        marker_array = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)

        # draw heading route robot pose
        if heading_pose is not None:
            marker_rgb = matplotlib.colors.to_rgb('pink')

            heading_pose_quat = (heading_pose.orientation.x, heading_pose.orientation.y, heading_pose.orientation.z, heading_pose.orientation.w)
            heading_pose_yaw = tf.transformations.euler_from_quaternion(heading_pose_quat)[2]

            arrow_length = 0.7
            arrow_start = heading_pose.position
            arrow_end = Point(arrow_start.x+arrow_length*math.cos(heading_pose_yaw), arrow_start.y+arrow_length*math.sin(heading_pose_yaw), 0.0)

            marker = visualize_utils.create_rviz_marker("explore-manager-heading-pose", 0, Marker.ARROW, scale_x=0.1, scale_y=0.2,
                                                        color_r=marker_rgb[0], color_g=marker_rgb[1], color_b=marker_rgb[2])
            marker.points.append(arrow_start)
            marker.points.append(arrow_end)
            marker_array.markers.append(marker)

        # draw hull polyhedron
        if hull_polyhedron is not None:
            marker_rgb = matplotlib.colors.to_rgb('lightblue')

            hull_polyhedron_points = hull_polyhedron.points[hull_polyhedron.vertices]
            for point_idx, point in enumerate(hull_polyhedron_points):
                marker = visualize_utils.create_rviz_marker("explore-manager-hull-polyhedron-point", point_idx, Marker.CYLINDER, position=Point(point[0], point[1], 0.0),
                                                            color_r=marker_rgb[0], color_g=marker_rgb[1], color_b=marker_rgb[2])
                marker_array.markers.append(marker)

            for simplex_idx, simplex in enumerate(hull_polyhedron.simplices):
                assert(len(simplex)==2)

                marker = visualize_utils.create_rviz_marker("explore-manager-hull-polyhedron-line", simplex_idx, Marker.LINE_STRIP, scale_x=0.05, scale_y=0.05,
                                                            color_r=marker_rgb[0], color_g=marker_rgb[1], color_b=marker_rgb[2])
                marker.points.append(Point(hull_polyhedron.points[simplex[0], 0], hull_polyhedron.points[simplex[0], 1], 0.0))
                marker.points.append(Point(hull_polyhedron.points[simplex[1], 0], hull_polyhedron.points[simplex[1], 1], 0.0))
                marker_array.markers.append(marker)

        # draw sample points inside polyhedron
        for point_idx, point in enumerate(sample_points_in_polyhedron):
            marker = visualize_utils.create_rviz_marker("explore-manager-close-sample-point", point_idx, Marker.CYLINDER, position=point,
                                                        color_r=1.0, scale_x=0.1, scale_y=0.1, scale_z=0.1)
            marker_array.markers.append(marker)

        # draw sample points outside polyhedron
        for point_idx, point in enumerate(sample_points_out_polyhedron):
            marker = visualize_utils.create_rviz_marker("explore-manager-far-sample-point", point_idx, Marker.CYLINDER, position=point,
                                                        color_b=1.0, scale_x=0.1, scale_y=0.1, scale_z=0.1)
            marker_array.markers.append(marker)

        # draw route nodes
        for node_idx, node_point in enumerate(route_node_points):
            if node_idx==forward_branch_node_index:
                marker_rgb = matplotlib.colors.to_rgb('red')
            elif node_idx==next_forward_branch_node_index:
                marker_rgb = matplotlib.colors.to_rgb('orange')
            else:
                marker_rgb = matplotlib.colors.to_rgb('blue')

            marker = visualize_utils.create_rviz_marker("explore-manager-route-node-point", node_idx, Marker.CYLINDER, position=node_point,
                                                        color_r=marker_rgb[0], color_g=marker_rgb[1], color_b=marker_rgb[2], scale_x=0.2, scale_y=0.2, scale_z=0.2)
            marker_array.markers.append(marker)

            marker = visualize_utils.create_rviz_marker("explore-manager-route-node-text", node_idx, Marker.TEXT_VIEW_FACING, position=Point(node_point.x, node_point.y, 1.0), text=str(node_idx))
            marker_array.markers.append(marker)

        # draw route edges
        edge_idx = 0
        for node_idx, node_point in enumerate(route_node_points):
            edge_start = node_point
            try:
                for neighbor_node_idx in route_graph.neighbors(node_idx):
                    if node_idx>neighbor_node_idx:
                        edge_end = route_node_points[neighbor_node_idx]

                        marker = visualize_utils.create_rviz_marker("explore-manager-route-edge", edge_idx, Marker.LINE_LIST, scale_x=0.05, scale_y=0.05,
                                                                    color_a=0.5, color_g=1.0)
                        marker.points.append(edge_start)
                        marker.points.append(edge_end)
                        marker_array.markers.append(marker)
                        edge_idx += 1
            except IndexError as e:
                rospy.logerr("cannot find route node in graph, node_idx = " + str(node_idx))

        self._next_goal_candidates_publisher.publish(marker_array)


    def _wait_map_goal_update(self):
        # wait until find goal candidate is called after costmap update and route graph update to get the latest information
        r = rospy.Rate(10)
        start_time = rospy.Time.now()
        while (self._time_update_costmap is None) or (self._time_update_costmap<start_time):
            r.sleep()
        start_time = rospy.Time.now()
        while (self._time_update_route_graph is None) or (self._time_update_route_graph<start_time):
            r.sleep()
        start_time = rospy.Time.now()
        while (self._time_update_goal_candidate_poses is None) or (self._time_update_goal_candidate_poses<start_time):
            r.sleep()


    def _system_cancel_explore_done_callback(self, status, result):
        # add landmark at system cancel position (i.e. detected intersection)
        robot_pose = self.get_current_pose()
        with self._lock_landmark:
            resp = self._add_landmark_proxy(robot_pose, self._next_landmark_neighbor_id)
            self._next_landmark_neighbor_id = resp.landmark_id
            self._last_landmark_pose = robot_pose

        # finish cancel explore
        self._user_cancel_explore_done_callback(status, result)


    def _user_cancel_explore_done_callback(self, status, result):
        rospy.loginfo("cancel explore is done")

        # if robot stopped at the dead end, wait constant time for map update to make sure there is no route
        if (not self._has_forward_goal) and len(self._goal_candidate_poses)==1:
            rospy.sleep(self._wait_map_update_time_dead_end)

        self._wait_map_goal_update()

        with self._lock_route_explore_state:
            self._route_explore_state = ExploreState.WAIT
            explore_status = self._create_cabot_explore_status(self._route_explore_state, self._update_goal_costmap_info, self._pose_to_turn, self._goal_candidate_poses, self._has_forward_goal, \
                                                            self._update_goal_reachable_area_contours, self._update_goal_reachable_area_hierarchy, self._update_goal_visible_area_contour, self._update_goal_coverage_polyhedron)
            # rospy.loginfo("Publish explore_status : " + json.dumps(explore_status))
            self._explore_status_publisher.publish(json.dumps(explore_status))


    # process public API event
    def _cabot_explore_event_callback(self, msg):
        # rospy.loginfo("explore status : " + str(self._route_explore_state) + ", msg.data : " + str(msg.data))
        if msg.data=="start":
            with self._lock_route_explore_state:
                self._need_send_next_goal = True
                self._route_explore_state = ExploreState.EXPLORE
        elif msg.data=="start_forward":
            with self._lock_route_explore_state:
                self._need_send_next_goal = True
                self._route_explore_state = ExploreState.EXPLORE_FORWARD
        elif msg.data=="go_origin":
            with self._lock_route_explore_state:
                self._need_send_next_goal = True
                self._route_explore_state = ExploreState.GO_ORIGIN
        elif msg.data=="observe_once":
            with self._lock_route_explore_state:
                self._heading_yaw = None
                self._route_explore_state = ExploreState.OBSERVE_ONCE
        elif msg.data=="system_cancel":
            with self._lock_route_explore_state:
                # by setting status pausing, execute function will publish status after finishing pause
                self._route_explore_state = ExploreState.PAUSING
                pose_to_turn = copy.copy(self._pose_to_turn)

            # try to stop by canceling goal
            rospy.loginfo("call cancel goal because pose to turn is close")
            self._navigate_to_pose_client.cancel_all_goals()
            self._navigate_to_pose_client.wait_for_result(timeout = rospy.Duration(2.0))

            # check if robot is already close enough to pose to turn after stop
            is_pose_to_turn_far_forward = False
            if pose_to_turn is not None:
                rospy.sleep(self._wait_resend_goal_pose_to_turn)
                robot_pose = self.get_current_pose()
                is_pose_to_turn_far_forward = self.is_pose_far_forward(robot_pose, pose_to_turn, self._tolerance_robot_arrive_pose_to_turn)

            if is_pose_to_turn_far_forward:
                # move to pose to turn before finish stopping
                rospy.loginfo("send goal to pose to turn")
                with self._lock_route_explore_state:
                    goal, ros_path, self._heading_yaw = self.create_goal_path(robot_pose, [pose_to_turn], self._heading_yaw)
                rospy.loginfo("call send_goal to " + str(goal.pose))
                self._path_publisher.publish(ros_path)
                self._navigate_to_pose_client.send_goal(goal, done_cb=self._system_cancel_explore_done_callback)
            else:
                # finish stopping
                self._system_cancel_explore_done_callback(None, None)
        elif msg.data=="user_cancel":
            with self._lock_route_explore_state:
                # by setting status pausing, execute function will publish status after finishing pause
                self._route_explore_state = ExploreState.PAUSING

            # stop immediately
            rospy.loginfo("call cancel goal because of user input")
            self._navigate_to_pose_client.cancel_all_goals()
            self._navigate_to_pose_client.wait_for_result(timeout = rospy.Duration(2.0))
            self._user_cancel_explore_done_callback(None, None)
        elif msg.data.startswith("start_turn:"):
            with self._lock_route_explore_state:
                self._heading_yaw = None
                self._route_explore_state = ExploreState.TURNING

                explore_status = self._create_cabot_explore_status(self._route_explore_state, self._update_goal_costmap_info, self._pose_to_turn, self._goal_candidate_poses, self._has_forward_goal, \
                                                                self._update_goal_reachable_area_contours, self._update_goal_reachable_area_hierarchy, self._update_goal_visible_area_contour, self._update_goal_coverage_polyhedron)
                self._explore_status_publisher.publish(json.dumps(explore_status))
        elif msg.data.startswith("finish_turn:"):
            turn_direction = msg.data[len("finish_turn:"):]
            self.update_goal_candidates_after_turn(turn_direction)
        else:
            rospy.loginfo("received invalid explore event : " + msg.data)


def main():
    rospy.init_node('explore_manager_node')

    use_scan_topic_free_area = rospy.get_param("~use_scan_topic_free_area", True)
    use_dt_polyhedron_sensor_range = rospy.get_param("~use_dt_polyhedron_sensor_range", True)
    default_polyhedron_sensor_range = rospy.get_param("~default_polyhedron_sensor_range", 8.0)
    dt_polyhedron_closest_node_range = rospy.get_param("~dt_polyhedron_closest_node_range", 5.0)
    dt_polyhedron_eps = rospy.get_param("~dt_polyhedron_eps", 1.0)
    use_filter_forward_turn_route = rospy.get_param("~use_filter_forward_turn_route", True)
    use_sample_free_points = rospy.get_param("~use_sample_free_points", True)
    sample_free_points_min_distance = rospy.get_param("~sample_free_points_min_distance", 1.0)
    sample_free_points_by_angle_range = rospy.get_param("~sample_free_points_by_angle_range", 12.0)
    sample_free_points_by_angle_unknown_area_margin = rospy.get_param("~sample_free_points_by_angle_unknown_area_margin", 1.0)
    sample_free_points_by_angle_lethal_area_margin = rospy.get_param("~sample_free_points_by_angle_lethal_area_margin", 1.0)
    use_cluster_point_outside_polyhedron_free_area = rospy.get_param("~use_cluster_point_outside_polyhedron_free_area", True)
    costmap_topic = rospy.get_param("~costmap_topic", "/cabot_explore/local_map")
    scan_topic = rospy.get_param("~scan_topic", "/cabot_explore/scan")
    path_topic = rospy.get_param("~path_topic", "/cabot_explore/path")
    lethal_cost_threshold = rospy.get_param("~lethal_cost_threshold", 65)
    costmap_inflation_radius = rospy.get_param("~costmap_inflation_radius", 0.3)
    costmap_resolution = rospy.get_param("~costmap_resolution", 0.20)
    min_frontier_size = rospy.get_param("~min_frontier_size", 2.0)
    use_pause_possible_turn = rospy.get_param("~use_pause_possible_turn", True)
    possible_turn_perpendicular_yaw_tolerance = rospy.get_param("~possible_turn_perpendicular_yaw_tolerance", math.pi/18.0)
    use_frontier_proximity = rospy.get_param("~use_frontier_proximity", True)
    frontier_proximity_threshold = rospy.get_param("~frontier_proximity_threshold", 3.0)
    min_distance_goal_inside_coverage_polyhedron = rospy.get_param("~min_distance_goal_inside_coverage_polyhedron", 1.0)
    max_abs_yaw_forward_goal = rospy.get_param("~max_abs_yaw_forward_goal", math.pi/6.0)
    max_landmark_interval = rospy.get_param("~max_landmark_interval", 5.0)
    landmark_loop_detect_distance = rospy.get_param("~landmark_loop_detect_distance", 10.0)

    explore_manager_node = ExploreManagerNode(use_scan_topic_free_area, use_dt_polyhedron_sensor_range, default_polyhedron_sensor_range, dt_polyhedron_closest_node_range,
                                            dt_polyhedron_eps, use_filter_forward_turn_route, use_sample_free_points, sample_free_points_min_distance, sample_free_points_by_angle_range,
                                            sample_free_points_by_angle_unknown_area_margin, sample_free_points_by_angle_lethal_area_margin, use_cluster_point_outside_polyhedron_free_area, costmap_topic, scan_topic,
                                            path_topic, lethal_cost_threshold, costmap_inflation_radius, costmap_resolution, min_frontier_size, use_pause_possible_turn, possible_turn_perpendicular_yaw_tolerance,
                                            use_frontier_proximity, frontier_proximity_threshold, min_distance_goal_inside_coverage_polyhedron, max_abs_yaw_forward_goal, max_landmark_interval, landmark_loop_detect_distance)

    reset_explore_manager_service = rospy.Service("reset_explore_manager", ResetExploreManager, explore_manager_node.reset_explore_manager_callback)

    try:
        # 10 Hz
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            explore_manager_node.execute()

            r.sleep()
    except rospy.ROSInterruptException as e: pass


if __name__ == "__main__":
    main()
