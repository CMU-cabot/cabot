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

from collections import deque
import copy
import itertools
import math
import sys
import threading
import uuid

from cartographer_ros_msgs.msg import SubmapList
from cartographer_ros_msgs.srv import SubmapQuery
from route_explore_msgs.srv import SearchPathResponse
from geometry_msgs.msg import Point
import matplotlib
from nav_msgs.msg import OccupancyGrid
import networkx
import numpy as np
import rospy
from scipy.spatial import cKDTree
import tf
from visualization_msgs.msg import Marker, MarkerArray

from . import costmap_utils, geometry_utils, visualize_utils
from .cartographer_utils import CartographerSubmap


class CartographerLandmark:

    def __init__(self, id, submap_index, local_landmark_pose, global_landmark_pose):
        self._id = id
        self._submap_index = submap_index
        self._local_landmark_pose = local_landmark_pose
        self._global_landmark_pose = global_landmark_pose


    @property
    def id(self):
        return self._id


    @property
    def submap_index(self):
        return self._submap_index


    @property
    def local_landmark_pose(self):
        return self._local_landmark_pose


    @property
    def global_landmark_pose(self):
        return self._global_landmark_pose


    @global_landmark_pose.setter
    def global_landmark_pose(self, global_landmark_pose):
        self._global_landmark_pose = global_landmark_pose


class CartographerLandmarkManager:

    def __init__(self, costmap_topic, submap_list_topic, lethal_cost_threshold, landmark_loop_detect_distance):
        rospy.loginfo("Start initializing CartographerLandmarkManager...")

        # start of constant parameters
        self._threshold_add_landmark_loop_edge_distance_ratio = 0.5
        # end of constant parameters

        self._costmap_topic = costmap_topic
        self._submap_list_topic = submap_list_topic
        self._lethal_cost_threshold = lethal_cost_threshold
        self._landmark_loop_detect_distance = landmark_loop_detect_distance

        # services
        self._submap_query = rospy.ServiceProxy("/submap_query", SubmapQuery)

        # publisher
        self._global_route_publisher = rospy.Publisher('/cabot_explore/global_route', MarkerArray, queue_size=1)

        # subscriber
        self._occupancy_grid_subscriber = None
        self._submap_list_subscriber = None

        self._update_submap_thread = None

        self._lock_costmap_msg = threading.Lock()
        self._lock_submap_list_msg = threading.Lock()
        self._lock_submap_dict = threading.Lock()
        self._lock_add_landmark_queue = threading.Lock()
        self._lock_landmark_list = threading.Lock()
        self._reset()


    def _reset(self):
        if self._occupancy_grid_subscriber is not None:
            self._occupancy_grid_subscriber.unregister()

        if self._submap_list_subscriber is not None:
            self._submap_list_subscriber.unregister()

        with self._lock_costmap_msg:
            self._costmap_msg = None

        while (self._update_submap_thread is not None) and (self._update_submap_thread.is_alive()):
            rospy.loginfo("wait to finish update submap thread")
            rospy.sleep(1.0)
        self._update_submap_thread = None

        with self._lock_submap_list_msg:
            self._submap_list_msg = None

        with self._lock_submap_dict:
            self._submap_dict = {}
            self._submap_pose_kd_tree = None
            self._submap_pose_kd_tree_index_submap_index_dict = {}

        with self._lock_add_landmark_queue:
            self._add_landmark_queue = deque()
            self._process_add_landmark_thread = None
            self._is_process_add_landmark_running = False

        with self._lock_landmark_list:
            self._landmark_list = []
            self._landmark_global_position_array = []
            self._landmark_global_position_kd_tree = None
            self._landmark_id_index_dict = {}
            self._landmark_graph = networkx.Graph()

        self._occupancy_grid_subscriber = rospy.Subscriber(self._costmap_topic, OccupancyGrid, self._costmap_callback, queue_size=1)

        self._submap_list_subscriber = rospy.Subscriber(self._submap_list_topic, SubmapList, self._submap_list_callback, queue_size=1)

        rospy.loginfo("finish reset cartographer manager")


    def reset_callback(self, data):
        success = True
        try:
            self._reset()
        except:
            success = False
        return success


    def _costmap_callback(self, msg):
        with self._lock_costmap_msg:
            self._costmap_msg = msg


    def _submap_list_callback(self, msg):
        with self._lock_submap_list_msg:
            self._submap_list_msg = msg

        # start thread for update submap
        if (self._update_submap_thread is None) or (not self._update_submap_thread.is_alive()):
            self._update_submap_thread = threading.Thread(target=self._run_update_submap)
            self._update_submap_thread.start()


    def _run_update_submap(self):
        with self._lock_submap_list_msg:
            submap_list_msg = copy.copy(self._submap_list_msg)

        with self._lock_submap_dict:
            submap_dict = copy.copy(self._submap_dict)

        # update submaps which have new version numbers
        updated_submap_indices = []
        for submap_entry in submap_list_msg.submap:
            if submap_entry.trajectory_id==0:
                if (submap_entry.submap_index not in submap_dict) or (submap_entry.submap_version>submap_dict[submap_entry.submap_index].submap_version):
                    submap_dict[submap_entry.submap_index] = submap_entry

                    submap_query_response = self._submap_query(0, submap_entry.submap_index)

                    ## get highest resolution of texture
                    submap_texture = submap_query_response.textures[0]

                    # update submap which have new version number
                    submap_dict[submap_entry.submap_index] = CartographerSubmap(submap_entry, submap_texture)

                    # rospy.loginfo("updated submap " + str(submap_entry.submap_index))
                    # submap_texture_intensity = submap_dict[submap_entry.submap_index].texture_intensity
                    # cv2.imshow("updated submap " + str(submap_entry.submap_index), submap_texture_intensity)
                    # cv2.waitKey(1)

                    updated_submap_indices.append(submap_entry.submap_index)
            else:
                rospy.logerr("submap trajectory ID should be 0")

        if len(updated_submap_indices)>0:
            discarded_landmarks = []

            with self._lock_submap_dict:
                # get submap indices which are not in the latest message
                submap_indices = [submap_entry.submap_index for submap_entry in submap_list_msg.submap]
                removed_submap_indices = [submap_dict_key for submap_dict_key in submap_dict.keys() if submap_dict_key not in submap_indices]

                # copy updated submaps, and discard submaps which are not listed in the latest message
                self._submap_dict = {submap_entry.submap_index: submap_dict[submap_entry.submap_index] for submap_entry in submap_list_msg.submap}

                # update submap kd-tree
                self._submap_pose_kd_tree_index_submap_index_dict = {}
                submap_pose_array = []
                for submap_index in self._submap_dict.keys():
                    submap_pose = self._submap_dict[submap_index].pose
                    submap_pose_array.append([submap_pose.position.x, submap_pose.position.y])
                    self._submap_pose_kd_tree_index_submap_index_dict[len(submap_pose_array)-1] = submap_index
                self._submap_pose_kd_tree = cKDTree(submap_pose_array)

                with self._lock_landmark_list:
                    # update global landmark pose in updated submaps
                    updated_landmark_id_list = []
                    for idx in range(len(self._landmark_list)):
                        if self._landmark_list[idx].submap_index in updated_submap_indices:
                            self._landmark_list[idx].global_landmark_pose = self._submap_dict[self._landmark_list[idx].submap_index].local_to_global_pose(self._landmark_list[idx].local_landmark_pose)
                            self._landmark_global_position_array[idx] = [self._landmark_list[idx].global_landmark_pose.position.x, self._landmark_list[idx].global_landmark_pose.position.y]
                            updated_landmark_id_list.append(self._landmark_list[idx].id)

                    # discard landmarks which are in discarded submaps
                    landmark_selector = [True] * len(self._landmark_list)
                    for landmark_idx, landmark in enumerate(self._landmark_list):
                        if landmark.submap_index in removed_submap_indices:
                            landmark_selector[landmark_idx] = False
                            discarded_landmarks.append({"id": landmark.id, "pose": landmark.global_landmark_pose, "neighbors": list(self._landmark_graph.neighbors(landmark.id))})
                    self._landmark_list = list(itertools.compress(self._landmark_list, landmark_selector))
                    self._landmark_global_position_array = list(itertools.compress(self._landmark_global_position_array, landmark_selector))
                    for discarded_landmark in discarded_landmarks:
                        self._landmark_id_index_dict.pop(discarded_landmark["id"])
                        self._landmark_graph.remove_node(discarded_landmark["id"])

                    # update edge distance for updated landmark
                    for updated_landmark_id in updated_landmark_id_list:
                        updated_landmark = self._landmark_list[self._landmark_id_index_dict[updated_landmark_id]]
                        try:
                            for neighbor_landmark_id in self._landmark_graph.neighbors(updated_landmark_id):
                                neighbor_landmark = self._landmark_list[self._landmark_id_index_dict[neighbor_landmark_id]]
                                neighbor_landmark_distance = geometry_utils.calc_point_distance(updated_landmark.global_landmark_pose.position, neighbor_landmark.global_landmark_pose.position)
                                self._landmark_graph[updated_landmark_id][neighbor_landmark_id]["distance"] = neighbor_landmark_distance
                        except IndexError as e:
                            rospy.logerr("cannot find updated landmark in graph, updated_landmark_id = " + str(updated_landmark_id))

                    # update kd-tree of landmarks global positions
                    if len(self._landmark_global_position_array)>0:
                        self._landmark_global_position_kd_tree = cKDTree(self._landmark_global_position_array)

            # add discarded landmarks again to the latest submap
            for discarded_landmark in discarded_landmarks:
                self._add_landmark(discarded_landmark["id"], discarded_landmark["pose"], discarded_landmark["neighbors"])

        # map_image = self._calc_map_image(submap_dict)
        # cv2.imshow("global map", map_image)
        # cv2.waitKey(1)

        with self._lock_landmark_list:
            landmark_list = copy.copy(self._landmark_list)
            landmark_id_index_dict = copy.copy(self._landmark_id_index_dict)
            landmark_graph = copy.copy(self._landmark_graph)
        self._visualize_global_route(landmark_list, landmark_id_index_dict, landmark_graph)


    def _calc_map_image(self, submap_dict, padding_pixels=5):
        map_min_x = sys.float_info.max
        map_min_y = sys.float_info.max
        map_max_x = sys.float_info.min
        map_max_y = sys.float_info.min
        submap_max_x_dict = {}
        submap_max_y_dict = {}
        for submap_index in submap_dict.keys():
            submap_entry = submap_dict[submap_index]
            submap_max_transform = np.dot(submap_entry.transform, submap_entry.slice_transform)
            submap_max_x_dict[submap_index] = submap_max_transform[0][3]
            submap_max_y_dict[submap_index] = submap_max_transform[1][3]

            submap_min_x = submap_max_x_dict[submap_index] - submap_entry.height*submap_entry.resolution
            submap_min_y = submap_max_y_dict[submap_index] - submap_entry.width*submap_entry.resolution
            submap_max_x = submap_max_x_dict[submap_index]
            submap_max_y = submap_max_y_dict[submap_index]

            if submap_min_x < map_min_x:
                map_min_x = submap_min_x
            if submap_min_y < map_min_y:
                map_min_y = submap_min_y
            if submap_max_x > map_max_x:
                map_max_x = submap_max_x
            if submap_max_y > map_max_y:
                map_max_y = submap_max_y

        map_min_x -= padding_pixels*submap_entry.resolution
        map_min_y -= padding_pixels*submap_entry.resolution
        map_max_x += padding_pixels*submap_entry.resolution
        map_max_y += padding_pixels*submap_entry.resolution

        map_image_width = int((map_max_x-map_min_x)/submap_entry.resolution)
        map_image_height = int((map_max_y-map_min_y)/submap_entry.resolution)

        map_image = np.full((map_image_height, map_image_width), -1, dtype=np.int8)
        for submap_index in submap_dict.keys():
            submap_entry = submap_dict[submap_index]

            submap_texture_intensity = np.rot90(submap_dict[submap_entry.submap_index].texture_intensity, k=1, axes=(1,0))
            submap_texture_alpha = np.rot90(submap_dict[submap_entry.submap_index].texture_alpha, k=1, axes=(1,0))

            submap_image_max_x = int((submap_max_x_dict[submap_index] - map_min_x)/submap_entry.resolution)
            submap_image_min_y = int((map_max_y - submap_max_y_dict[submap_index])/submap_entry.resolution)

            submap_image_min_x = submap_image_max_x - submap_entry.height
            submap_image_max_y = submap_image_min_y + submap_entry.width

            global_submap_texture_intensity = np.zeros((map_image_height, map_image_width), np.int8)
            global_submap_texture_intensity[submap_image_min_y:submap_image_max_y, submap_image_min_x:submap_image_max_x] = submap_texture_intensity

            global_submap_texture_alpha = np.zeros((map_image_height, map_image_width), np.int8)
            global_submap_texture_alpha[submap_image_min_y:submap_image_max_y, submap_image_min_x:submap_image_max_x] = submap_texture_alpha

            map_image = np.where((global_submap_texture_intensity>map_image) | (global_submap_texture_alpha!=0), global_submap_texture_intensity, map_image)
        return map_image


    def _visualize_global_route(self, landmark_list, landmark_id_index_dict, landmark_graph):
        marker_array = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)

        # draw landmark
        if landmark_list is not None:
            marker_rgb = matplotlib.colors.to_rgb('lightblue')

            landmark_poses = [landmark.global_landmark_pose for landmark in landmark_list]
            for landmark_idx, landmark_pose in enumerate(landmark_poses):
                marker = visualize_utils.create_rviz_marker("cartographer-utils-landmark-point", landmark_idx, Marker.CYLINDER, position=landmark_pose.position,
                                                            color_r=marker_rgb[0], color_g=marker_rgb[1], color_b=marker_rgb[2], scale_x=0.2, scale_y=0.2, scale_z=0.2)
                marker_array.markers.append(marker)

        # draw landmark edges
        if (landmark_list is not None) and (landmark_id_index_dict is not None) and (landmark_graph is not None):
            marker_rgb = matplotlib.colors.to_rgb('lightgreen')

            edge_idx = 0
            for landmark in landmark_list:
                edge_start = landmark.global_landmark_pose.position
                landmark_idx = landmark_id_index_dict[landmark.id]
                try:
                    for neighbor_node_id in landmark_graph.neighbors(landmark.id):
                        neighbor_node_idx = landmark_id_index_dict[neighbor_node_id]
                        if landmark_idx>neighbor_node_idx:
                            edge_end = landmark_list[neighbor_node_idx].global_landmark_pose.position

                            marker = visualize_utils.create_rviz_marker("cartographer-utils-landmark-edge", edge_idx, Marker.LINE_LIST, scale_x=0.05, scale_y=0.05,
                                                                        color_a=0.5, color_r=marker_rgb[0], color_g=marker_rgb[1], color_b=marker_rgb[2])
                            marker.points.append(edge_start)
                            marker.points.append(edge_end)
                            marker_array.markers.append(marker)

                            edge_distance_position = Point((edge_start.x+edge_end.x)/2.0, (edge_start.y+edge_end.y)/2.0, 0.0)
                            edge_distance = self._landmark_graph[landmark.id][neighbor_node_id]["distance"]
                            marker = visualize_utils.create_rviz_marker("cartographer-utils-landmark-edge-text", edge_idx, Marker.TEXT_VIEW_FACING,
                                                                        position=edge_distance_position, text="{:.2f}".format(edge_distance))
                            marker_array.markers.append(marker)

                            edge_idx += 1
                except IndexError as e:
                    rospy.logerr("cannot find landmark in graph, landmark_idx = " + str(landmark_idx))

        self._global_route_publisher.publish(marker_array)



    def _process_add_landmark(self):
        while not rospy.is_shutdown():
            is_submap_ready = False
            with self._lock_submap_dict:
                if self._submap_pose_kd_tree is not None:
                    is_submap_ready = True
            if is_submap_ready:
                rospy.loginfo("submap pose kd-tree is ready")
                break
            else:
                rospy.loginfo("submap pose kd-tree not ready")
            rospy.sleep(0.01)

        costmap_msg = None
        while not rospy.is_shutdown():
            with self._lock_costmap_msg:
                costmap_msg = copy.copy(self._costmap_msg)
            if costmap_msg is not None:
                rospy.loginfo("costmap msg is ready")
                break
            else:
                rospy.loginfo("costmap msg not ready")
            rospy.sleep(0.01)

        costmap_info = costmap_utils.CostmapInfo(costmap_msg.info)
        costmap_image = np.array(costmap_msg.data, dtype=np.int8).reshape(costmap_info.height, costmap_info.width)
        binary_non_free_image = costmap_utils.calc_binary_non_free_image(costmap_image, self._lethal_cost_threshold)

        while True:
            add_landmark_id = None
            add_landmark_pose = None
            add_landmark_neighbor_ids = None
            with self._lock_add_landmark_queue:
                if len(self._add_landmark_queue)>0:
                    add_landmark_dict = self._add_landmark_queue.popleft()
                    add_landmark_id = add_landmark_dict["id"]
                    add_landmark_pose = add_landmark_dict["pose"]
                    add_landmark_neighbor_ids = add_landmark_dict["neighbors"]

                if (add_landmark_id is None) or (add_landmark_pose is None):
                    self._is_process_add_landmark_running = False
                    break

            add_landmark_position_array = np.array([add_landmark_pose.position.x, add_landmark_pose.position.y])

            with self._lock_submap_dict:
                _, closest_submap_pose_kd_tree_index = self._submap_pose_kd_tree.query(add_landmark_position_array, k=1)
                closest_submap_index = self._submap_pose_kd_tree_index_submap_index_dict[closest_submap_pose_kd_tree_index]

                local_add_landmark_pose = self._submap_dict[closest_submap_index].global_to_local_pose(add_landmark_pose)

            with self._lock_landmark_list:
                add_landmark = CartographerLandmark(add_landmark_id, closest_submap_index, local_add_landmark_pose, add_landmark_pose)
                self._landmark_list.append(add_landmark)
                self._landmark_global_position_array.append([add_landmark.global_landmark_pose.position.x, add_landmark.global_landmark_pose.position.y])
                self._landmark_global_position_kd_tree = cKDTree(self._landmark_global_position_array)
                self._landmark_id_index_dict[add_landmark.id] = len(self._landmark_list)-1

                # add landmark to graph
                self._landmark_graph.add_node(add_landmark.id)

                # add landmark edge between new node and its successive node
                for add_landmark_neighgbor_id in add_landmark_neighbor_ids:
                    if not self._landmark_graph.has_edge(add_landmark_id, add_landmark_neighgbor_id):
                        add_landmark_neighgbor_idx = self._landmark_id_index_dict[add_landmark_neighgbor_id]
                        edge_distance = geometry_utils.calc_point_distance(add_landmark_pose.position, self._landmark_list[add_landmark_neighgbor_idx].global_landmark_pose.position)
                        self._landmark_graph.add_edge(add_landmark_id, add_landmark_neighgbor_id, distance=edge_distance)

                # add looped landmark edges for new node
                ## find close landmarks
                close_landmark_indices = self._landmark_global_position_kd_tree.query_ball_point(add_landmark_position_array, self._landmark_loop_detect_distance)
                for idx in close_landmark_indices:
                    ## check if close landmark is visible from new node
                    if (not self._landmark_graph.has_edge(add_landmark_id, self._landmark_list[idx].id)) and \
                            (geometry_utils.check_is_in_line_of_sight(costmap_info, binary_non_free_image, add_landmark_pose.position, self._landmark_list[idx].global_landmark_pose.position)):
                        ## check if adding a edge between new node and close visible landmark will shorten current path length
                        close_landmark_distance = geometry_utils.calc_point_distance(add_landmark_pose.position, self._landmark_list[idx].global_landmark_pose.position)
                        try:
                            close_landmark_shortest_path_distance = networkx.shortest_path_length(self._landmark_graph, source=add_landmark_id, target=self._landmark_list[idx].id, weight="distance")
                        except Exception as e:
                            close_landmark_shortest_path_distance = None

                        if (close_landmark_shortest_path_distance is None) or (close_landmark_distance < close_landmark_shortest_path_distance*self._threshold_add_landmark_loop_edge_distance_ratio):
                            self._landmark_graph.add_edge(add_landmark_id, self._landmark_list[idx].id, distance=close_landmark_distance)
                            rospy.loginfo("add looped landmark edge for close visible node, close_landmark_distance = " + str(close_landmark_distance) + ", close_landmark_shortest_path_distance = " + str(close_landmark_shortest_path_distance))
                        else:
                            rospy.loginfo("skip add looped landmark edge for close visible node, close_landmark_distance = " + str(close_landmark_distance) + ", close_landmark_shortest_path_distance = " + str(close_landmark_shortest_path_distance))

            rospy.loginfo("added landmark " + str(add_landmark_pose))


    def _add_landmark(self, landmark_id, landmark_pose, neighbor_ids):
        with self._lock_add_landmark_queue:
            self._add_landmark_queue.append({"id": landmark_id, "pose": landmark_pose, "neighbors": neighbor_ids})

            # start thread for adding landmark
            if not self._is_process_add_landmark_running:
                self._is_process_add_landmark_running = True
                self._process_add_landmark_thread = threading.Thread(target=self._process_add_landmark)
                self._process_add_landmark_thread.start()


    def add_landmark_callback(self, data):
        landmark_pose = data.landmark_pose
        neighbor_id = data.neighbor_id

        landmark_id = str(uuid.uuid4())
        neighbor_ids = [neighbor_id] if (neighbor_id is not None) and (len(neighbor_id)>0) else []
        self._add_landmark(landmark_id, landmark_pose, neighbor_ids)
        return landmark_id


    def find_closest_landmark_id_callback(self, data):
        point = data.point

        point_array = np.array([point.x, point.y])
        with self._lock_landmark_list:
            _, closest_landmark_index = self._landmark_global_position_kd_tree.query(point_array, k=1)
            closest_landmark_id = self._landmark_list[closest_landmark_index].id
        return closest_landmark_id


    def search_path_callback(self, data):
        start_point = data.start_point
        target_point = data.target_point

        start_point_array = np.array([start_point.x, start_point.y])
        target_point_array = np.array([target_point.x, target_point.y])

        path_pose_list = []
        with self._lock_landmark_list:
            _, start_closest_landmark_index = self._landmark_global_position_kd_tree.query(start_point_array, k=1)
            _, target_closest_landmark_index = self._landmark_global_position_kd_tree.query(target_point_array, k=1)

            if start_closest_landmark_index!=target_closest_landmark_index:
                start_closest_landmark = self._landmark_list[start_closest_landmark_index]
                target_closest_landmark = self._landmark_list[target_closest_landmark_index]

                shortest_path = networkx.shortest_path(self._landmark_graph, source=start_closest_landmark.id, target=target_closest_landmark.id, weight="distance")
                for landmark_id in shortest_path:
                    path_pose_list.append(self._landmark_list[self._landmark_id_index_dict[landmark_id]].global_landmark_pose)
            else:
                target_closest_landmark = self._landmark_list[target_closest_landmark_index]
                path_pose_list.append(target_closest_landmark.global_landmark_pose)

        if len(path_pose_list)>2:
            # check if first path pose is backward direction from start
            start_to_first_path_quat = geometry_utils.calc_point_direction_quaternion(start_point, path_pose_list[0].position)
            first_path_quat = geometry_utils.calc_point_direction_quaternion(path_pose_list[0].position, path_pose_list[1].position)
            quat_diff = geometry_utils.calc_relative_orientation_quaternion(start_to_first_path_quat, first_path_quat)
            yaw_diff = tf.transformations.euler_from_quaternion(quat_diff)[2]
            if math.cos(yaw_diff)<0:
                # remove first pose which is backward direction from start
                path_pose_list = path_pose_list[1:]

        response = SearchPathResponse()
        response.path_pose_list = path_pose_list
        return response