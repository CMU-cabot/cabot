#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

import math
import sys

import cv2
from geometry_msgs.msg import Point
import numba
import numpy as np
import rospy
import scipy.ndimage as ndimage
from scipy.spatial import cKDTree
from skimage.morphology import skeletonize
import tf

from . import costmap_utils
from . import geometry_utils


# Function for counting isolated 8-neighbor point to detect route branches and edges.
# Because 8-neighbor points are checked in the order of top-left to bottom-right,
# check if each point is isolated only from 4-neighbors of top-left direction.
@numba.njit(fastmath=True)
def _isolated_8_neighbor_point_count(values):
    sum = 0
    for idx, val in enumerate(values):
        if val==1:
            if idx==0:
                sum += 1
            elif idx==1 and values[0]==0:
                sum += 1
            elif idx==2 and values[1]==0:
                sum += 1
            elif idx==3 and values[0]==0:
                sum += 1
            elif idx==4 and values[2]==0:
                sum += 1
            elif idx==5 and values[3]==0:
                sum += 1
            elif idx==6 and values[5]==0:
                sum += 1
            elif idx==7 and values[4]==0 and values[6]==0:
                sum += 1
    return sum


def extract_route(costmap, lethal_cost_threshold, robot_pose, minimum_route_length, gaussian_radius=0.2, min_detect_l_corner_distance=1.0, max_detect_l_corner_angle=math.pi*3.0/4.0):
    """
    Extract route from occupancy grid map

    Parameters
    ----------
    costmap : costmap_utils.Costmap
        The cost map
    lethal_cost_threshold : int
        If the cost map value is equal or larther than this value, that point is considered as lethal area
    robot_pose : geometry_msgs.msg.Pose
        The robot pose
    minimum_route_length : float
        The minimum route length in meters when detecting nodes
    gaussian_radius : float
        The radius of gaussian filter to remove noise before extract route
    min_detect_l_corner_distance : float
        The minimum distance to detect L-corner route nodes using keypoints
    max_detect_l_corner_angle : float
        The maximum angle to detect L-corner route nodes using keypoints

    Returns
    -------
    route_node_points : array-like
        The positions of route nodes
    route_node_connections : dictionary
        The dictionary of connected route node indices
    map_route_points_set : set
        The set of all route points positions in map coordinate
    route_dt_image : numpy.ndarray
        The float image created by distance transform, larger value is closer to route
    """
    minimum_route_length_pixels = int(minimum_route_length / costmap.resolution)
    min_detect_l_corner_distance_pixels = int(min_detect_l_corner_distance / costmap.resolution)

    # step1 : create route binary image by skeltonization
    grid_data_image = np.array(costmap.occupancy_grid_data, dtype=np.int8).reshape(costmap.height, costmap.width)

    ## debug visualization
    # cv2.imshow("gray image", grid_data_image)
    # cv2.waitKey(1)

    ## create binary image, set both unknown area and high cost area as obstacles
    binary_image = np.zeros(grid_data_image.shape, np.uint8)
    binary_image[np.where((grid_data_image>=0) & (grid_data_image<lethal_cost_threshold))] = 1

    ## debug visualization
    # cv2.imshow("binary image", binary_image*255)
    # cv2.waitKey(1)

    ## remove noise
    gaussian_sigma = 1
    gaussian_radius_pixels = int(gaussian_radius / costmap.resolution)
    # gaussian filter will be applied to the window size (sigma*truncate + 0.5)
    gaussian_truncate = (gaussian_radius_pixels - 0.5) / gaussian_sigma
    binary_image = ndimage.gaussian_filter(binary_image, sigma=gaussian_sigma, mode='constant', cval=0.0, truncate=gaussian_truncate)

    ## debug visualization
    # cv2.imshow("binary image without noise", binary_image*255)
    # cv2.waitKey(1)

    ## skeltonize
    route_image = skeletonize(binary_image).astype(np.uint8)

    ## debug visualization
    # cv2.imshow("route image", route_image*255)
    # cv2.waitKey(1)

    if not np.any(route_image):
        return [], [], {}, set([]), np.zeros(grid_data_image.shape, np.float32)

    ## remove routes which are disconnected from robot closest route

    ### calculate route point which is closest from robot pose
    map_route_points = np.argwhere(route_image==1)
    map_route_points = map_route_points[:, [1, 0]].tolist()
    map_route_points_kd_tree = cKDTree(map_route_points)
    robot_pose_map_x, robot_pose_map_y = costmap_utils.world_to_map(costmap.costmap_info, robot_pose.position.x, robot_pose.position.y)
    _, closest_route_point_index = map_route_points_kd_tree.query(np.array([robot_pose_map_x, robot_pose_map_y]), k=1)

    ### calculate stats of route
    route_num_labels, route_labels = cv2.connectedComponents(route_image)

    if route_num_labels==1:
        return [], [], {}, set([]), np.zeros(grid_data_image.shape, np.float32)

    ### remove routes which have different label from route closest route
    closest_route_label = route_labels[map_route_points[closest_route_point_index][1]][map_route_points[closest_route_point_index][0]]
    for label in range(1, route_num_labels):
        if label!=closest_route_label:
            route_image[np.where(route_labels==label)] = 0.0

    ## debug visualization
    # cv2.imshow("route image after removing disconnected routes", route_image*255)
    # cv2.waitKey(1)

    # step2 : calculate route node points

    ## calclate route poins after removing disconnected route
    map_route_points = np.argwhere(route_image==1)
    map_route_points = map_route_points[:, [1, 0]].tolist()
    map_route_points_kd_tree = cKDTree(map_route_points)
    map_route_points_set = set([tuple(point) for point in map_route_points])

    ## detect T-shape, X-shape, L-shape corners by finding keypoints
    detect_keypoints_maximum_num = 10000
    detect_keypoints_quality = 0.01
    map_keypoints = cv2.goodFeaturesToTrack(route_image, detect_keypoints_maximum_num, detect_keypoints_quality, min_detect_l_corner_distance_pixels, useHarrisDetector=True)
    if map_keypoints is None:
        return [], [], {}, set([]), np.zeros(grid_data_image.shape, np.float32)
    map_keypoints = np.rint(map_keypoints).astype(np.int32)
    map_keypoints = np.squeeze(map_keypoints, axis=1).tolist()

    ## debug visualization
    # route_corner_points_image = np.zeros(grid_data_image.shape, np.uint8)
    # for point in map_keypoints:
    #     cv2.circle(route_corner_points_image, (point[0], point[1]), radius=3, color=255, thickness=cv2.FILLED)
    # cv2.imshow("route corner points image", route_corner_points_image)
    # cv2.waitKey(1)

    ## calculate number of neighbor route points
    neighbor_point_count_footprint = np.array([[1,1,1],
                                            [1,0,1],
                                            [1,1,1]])
    route_neighbor_point_count_image = ndimage.generic_filter(route_image, _isolated_8_neighbor_point_count, footprint=neighbor_point_count_footprint, mode='constant', cval=0.0)

    ## debug visualization
    # cv2.imshow("route neighbor point count image", route_neighbor_point_count_image*int(255/4))
    # cv2.waitKey(1)

    ## detect route branch points
    map_route_branch_points = np.argwhere((route_image==1) & (route_neighbor_point_count_image>=3))
    map_route_branch_points = map_route_branch_points[:, [1, 0]].tolist()

    ## debug visualization
    # route_branch_points_image = np.zeros(grid_data_image.shape, np.uint8)
    # for point in map_route_brarnch_points:
    #     cv2.circle(route_branch_points_image, (point[0], point[1]), radius=3, color=255, thickness=cv2.FILLED)
    # cv2.imshow("route branch points image", route_branch_points_image)
    # cv2.waitKey(1)

    ## detect route end points
    map_route_end_points = np.argwhere((route_image==1) & (route_neighbor_point_count_image==1))
    map_route_end_points = map_route_end_points[:, [1, 0]].tolist()

    ## debug visualization
    # route_end_points_image = np.zeros(grid_data_image.shape, np.uint8)
    # for point in map_route_end_points:
    #     cv2.circle(route_end_points_image, (point[0], point[1]), radius=3, color=255, thickness=cv2.FILLED)
    # cv2.imshow("route end points image", route_end_points_image)
    # cv2.waitKey(1)

    ## collect route branch points and route end points
    if len(map_route_branch_points)>0:
        map_route_branch_end_points = map_route_branch_points
        map_route_branch_points_kd_tree = cKDTree(map_route_branch_points)
        for point in map_route_end_points:
            route_end_close_branch_points = map_route_branch_points_kd_tree.query_ball_point(point, minimum_route_length_pixels)
            if len(route_end_close_branch_points)==0:
                map_route_branch_end_points.append(point)
    else:
        map_route_branch_end_points = map_route_end_points

    ## collect route node points and keypoints of route corners
    if len(map_route_branch_end_points)>0:
        map_route_node_points = map_route_branch_end_points
        map_route_brarnch_end_points_kd_tree = cKDTree(map_route_branch_end_points)
        for point in map_keypoints:
            route_branch_end_close_keypoints = map_route_brarnch_end_points_kd_tree.query_ball_point(point, min_detect_l_corner_distance_pixels)
            if len(route_branch_end_close_keypoints)==0:
                if (tuple(point) in map_route_points_set) and (route_neighbor_point_count_image[point[1]][point[0]]==2):
                    map_route_node_points.append(point)
                else:
                    _, closest_route_point_index = map_route_points_kd_tree.query(point, k=1)
                    if route_neighbor_point_count_image[map_route_points[closest_route_point_index][1]][map_route_points[closest_route_point_index][0]]==2:
                        map_route_node_points.append(map_route_points[closest_route_point_index])
    else:
        map_route_node_points = []
        for point in map_keypoints:
            if tuple(point) in map_route_points_set:
                map_route_node_points.append(point)
            else:
                _, closest_route_point_index = map_route_points_kd_tree.query(point, k=1)
                map_route_node_points.append(map_route_points[closest_route_point_index])

    ## debug visualization
    # route_node_points_image = np.zeros(grid_data_image.shape, np.uint8)
    # for point in map_route_node_points:
    #     cv2.circle(route_node_points_image, (point[0], point[1]), radius=3, color=255, thickness=cv2.FILLED)
    # cv2.imshow("route node points image", route_node_points_image)
    # cv2.waitKey(1)

    ## calculate route node connections

    ### find route node which is closest to robot
    map_route_node_points_kd_tree = cKDTree(map_route_node_points)
    _, closest_map_route_node_point_index = map_route_node_points_kd_tree.query(np.array([robot_pose_map_x, robot_pose_map_y]), k=1)
    closest_map_route_node_point = map_route_node_points[closest_map_route_node_point_index]

    ### traverse route points from route node which is closest to robot
    route_node_connections = {}
    map_route_node_points_set = set([tuple(point) for point in map_route_node_points])
    open_route_point_set = set([tuple(closest_map_route_node_point)])
    route_point_from_route_node_index_dict = {}
    route_point_from_route_node_index_dict[tuple(closest_map_route_node_point)] = map_route_node_points.index(closest_map_route_node_point)
    close_route_point_set = set()


    def _add_route_node_connection(route_node_index1, route_node_index2):
        # skip if input nodes indices are same, the starting route node is traversed
        if route_node_index1==route_node_index2:
            return

        if route_node_index1 not in route_node_connections:
            route_node_connections[route_node_index1] = []
        if route_node_index2 not in route_node_connections[route_node_index1]:
            route_node_connections[route_node_index1].append(route_node_index2)

        if route_node_index2 not in route_node_connections:
            route_node_connections[route_node_index2] = []
        if route_node_index1 not in route_node_connections[route_node_index2]:
            route_node_connections[route_node_index2].append(route_node_index1)


    def _traverse_route_point(neighbor_point, from_route_node_index):
        if neighbor_point in map_route_node_points_set:
            # if traversed point is route node, add route connection for the traversed route node
            neighbor_point_route_node_index = map_route_node_points.index(list(neighbor_point))

            _add_route_node_connection(from_route_node_index, neighbor_point_route_node_index)
        else:
            # if traversed point is route point, check if the route point is alredy visited from opposite direction to check looped route
            looped_route_node_index = None
            if tuple(neighbor_point) in route_point_from_route_node_index_dict:
                looped_route_node_index = route_point_from_route_node_index_dict[tuple(neighbor_point)]
            if looped_route_node_index is not None:
                # route point is already visited, add route connection to close looped route
                _add_route_node_connection(from_route_node_index, looped_route_node_index)

        # cache from which route node each route point is traversed
        route_point_from_route_node_index_dict[tuple(neighbor_point)] = from_route_node_index


    while len(open_route_point_set)>0:
        route_point = open_route_point_set.pop()
        close_route_point_set.add(route_point)

        route_point_neighbor_8 = [(route_point[0]-1, route_point[1]-1), (route_point[0], route_point[1]-1), (route_point[0]+1, route_point[1]-1), \
                                (route_point[0]-1, route_point[1]), (route_point[0]+1, route_point[1]), \
                                (route_point[0]-1, route_point[1]+1), (route_point[0], route_point[1]+1), (route_point[0]+1, route_point[1]+1)]

        if route_point in map_route_node_points_set:
            from_route_node_index = map_route_node_points.index(list(route_point))
        else:
            from_route_node_index = route_point_from_route_node_index_dict[tuple(route_point)]

        #### if route nodes exist in neighborhood, process neighbors of neighbor route nodes later
        neighbor_route_node_neighbor_8_set = set()
        for neighbor_point in route_point_neighbor_8:
            if (neighbor_point in map_route_node_points_set) and (neighbor_point not in close_route_point_set):
                neighbor_point_neighbor_8 = [(neighbor_point[0]-1, neighbor_point[1]-1), (neighbor_point[0], neighbor_point[1]-1), (neighbor_point[0]+1, neighbor_point[1]-1), \
                                            (neighbor_point[0]-1, neighbor_point[1]), (neighbor_point[0]+1, neighbor_point[1]), \
                                            (neighbor_point[0]-1, neighbor_point[1]+1), (neighbor_point[0], neighbor_point[1]+1), (neighbor_point[0]+1, neighbor_point[1]+1)]
                neighbor_route_node_neighbor_8_set |= set(neighbor_point_neighbor_8)

                open_route_point_set.add(neighbor_point)
                _traverse_route_point(neighbor_point, from_route_node_index)

        if len(neighbor_route_node_neighbor_8_set)==0:
            ##### neighbor route nodes are not found, process all neighbors
            for neighbor_point in route_point_neighbor_8:
                if (neighbor_point in map_route_points_set) and (neighbor_point not in close_route_point_set):
                    open_route_point_set.add(neighbor_point)
                    _traverse_route_point(neighbor_point, from_route_node_index)
        else:
            ##### neighbor route nodes are found, process neighbors of neighbor route nodes later
            for neighbor_point in route_point_neighbor_8:
                if (neighbor_point in map_route_points_set) and (neighbor_point not in close_route_point_set) and (neighbor_point not in neighbor_route_node_neighbor_8_set):
                    open_route_point_set.add(neighbor_point)
                    _traverse_route_point(neighbor_point, from_route_node_index)

    # rospy.loginfo("number of route nodes : " + str(len(map_route_node_points)))
    # rospy.loginfo("found route node connections : " + str(route_node_connections))

    ## create route node points in map and world
    dict_route_node_indices_map_points = {}
    dict_route_node_indices_points = {}
    for point_idx, point in enumerate(map_route_node_points):
        if point_idx in route_node_connections:
            dict_route_node_indices_map_points[point_idx] = (point[0], point[1])

            world_x, world_y = costmap_utils.map_to_world(costmap.costmap_info, point[0], point[1])
            dict_route_node_indices_points[point_idx] = (world_x, world_y)
        else:
            rospy.logerr("route node is not connected, route node index = " + str(point_idx))

    # step3 : simplify route connections by removing following redundant route nodes
    # 1. route nodes that connects two route nodes which are connected with angles larger than max_l_corner_angle, and visible each other
    # 2. route nodes that are end of routes and connected with route branch and distance is smaller than minimum_route_length

    ## find redundant route nodes, and remove the nodes and their connections
    redundant_route_node_indices = []
    node_idx_pair_angle_dict = {}
    node_idx_pair_is_visible_dict = {}
    node_idx_pair_distance_dict = {}
    binary_non_free_image = costmap_utils.calc_binary_non_free_image(grid_data_image, lethal_cost_threshold)
    while True:
        redundant_route_node_idx = None
        for route_node_idx in dict_route_node_indices_points.keys():
            route_node_point = dict_route_node_indices_points[route_node_idx]
            np_route_node_point = np.array(route_node_point)
            ### check if route node matches with condition 1
            if len(route_node_connections[route_node_idx])==2:
                connected_node_id1 = route_node_connections[route_node_idx][0]
                connected_node_id2 = route_node_connections[route_node_idx][1]
                connected_node_idx_pair = (connected_node_id1, connected_node_id2)

                #### check angle of connected nodes are larger than max_detect_l_corner_angle
                if connected_node_idx_pair not in node_idx_pair_angle_dict:
                    np_connected_route1 = np.array(dict_route_node_indices_points[connected_node_id1]) - np_route_node_point
                    np_connected_route2 = np.array(dict_route_node_indices_points[connected_node_id2]) - np_route_node_point
                    connected_routes_angle = np.arccos(np.inner(np_connected_route1, np_connected_route2) / (np.linalg.norm(np_connected_route1) * np.linalg.norm(np_connected_route2)))
                    node_idx_pair_angle_dict[(connected_node_id1, connected_node_id2)] = connected_routes_angle
                    node_idx_pair_angle_dict[(connected_node_id2, connected_node_id1)] = connected_routes_angle

                if node_idx_pair_angle_dict[connected_node_idx_pair]>max_detect_l_corner_angle:
                    #### check connected nodes are visible each other
                    if connected_node_idx_pair not in node_idx_pair_is_visible_dict:
                        connected_node_point1 = Point(dict_route_node_indices_points[connected_node_id1][0], dict_route_node_indices_points[connected_node_id1][1], 0)
                        connected_node_point2 = Point(dict_route_node_indices_points[connected_node_id2][0], dict_route_node_indices_points[connected_node_id2][1], 0)
                        is_visible = geometry_utils.check_is_in_line_of_sight(costmap.costmap_info, binary_non_free_image, connected_node_point1, connected_node_point2)
                        node_idx_pair_is_visible_dict[(connected_node_id1, connected_node_id2)] = is_visible
                        node_idx_pair_is_visible_dict[(connected_node_id2, connected_node_id1)] = is_visible

                    if node_idx_pair_is_visible_dict[connected_node_idx_pair]:
                        redundant_route_node_idx = route_node_idx
                        break
            ### check if route node matches with condition 2
            if len(route_node_connections[route_node_idx])==1:
                connected_node_id = route_node_connections[route_node_idx][0]
                if len(route_node_connections[connected_node_id])>2:
                    connected_node_idx_pair = (route_node_idx, connected_node_id)
                    #### check distance of connected nodes are larger than minimum_route_length
                    if connected_node_idx_pair not in node_idx_pair_distance_dict:
                        np_connected_node_point = np.array(dict_route_node_indices_points[connected_node_id])
                        node_idx_pair_distance_dict[connected_node_idx_pair] = np.linalg.norm(np_route_node_point - np_connected_node_point)

                    if node_idx_pair_distance_dict[connected_node_idx_pair]<minimum_route_length:
                        redundant_route_node_idx = route_node_idx
                        break

        if redundant_route_node_idx is not None:
            if len(route_node_connections[redundant_route_node_idx])==2:
                redundant_connected_node_idx1 = route_node_connections[redundant_route_node_idx][0]
                redundant_connected_node_idx2 = route_node_connections[redundant_route_node_idx][1]

                redundant_route_node_indices.append(redundant_route_node_idx)

                dict_route_node_indices_points.pop(redundant_route_node_idx, None)

                route_node_connections.pop(redundant_route_node_idx, None)
                route_node_connections[redundant_connected_node_idx1].remove(redundant_route_node_idx)
                route_node_connections[redundant_connected_node_idx2].remove(redundant_route_node_idx)

                if redundant_connected_node_idx2 not in route_node_connections[redundant_connected_node_idx1]:
                    route_node_connections[redundant_connected_node_idx1].append(redundant_connected_node_idx2)
                if redundant_connected_node_idx1 not in route_node_connections[redundant_connected_node_idx2]:
                    route_node_connections[redundant_connected_node_idx2].append(redundant_connected_node_idx1)
            else:
                redundant_connected_node_idx = route_node_connections[redundant_route_node_idx][0]

                redundant_route_node_indices.append(redundant_route_node_idx)

                dict_route_node_indices_points.pop(redundant_route_node_idx, None)

                route_node_connections.pop(redundant_route_node_idx, None)
                route_node_connections[redundant_connected_node_idx].remove(redundant_route_node_idx)
        else:
            break

    if len(redundant_route_node_indices)>0:
        ## update route node connections to use node indices after removing redundant nodes
        dict_redundant_route_indices_route_indices = {}
        map_route_node_points = []
        route_node_points = []
        sorted_route_node_indices = list(dict_route_node_indices_points.keys())
        sorted_route_node_indices.sort()
        for route_node_index in sorted_route_node_indices:
            dict_redundant_route_indices_route_indices[route_node_index] = len(route_node_points)
            map_route_node_points.append(dict_route_node_indices_map_points[route_node_index])
            route_node_points.append(dict_route_node_indices_points[route_node_index])

        redundant_route_node_connections = route_node_connections
        route_node_connections = {}
        for redundant_route_node_idx in redundant_route_node_connections.keys():
            route_node_idx = dict_redundant_route_indices_route_indices[redundant_route_node_idx]
            route_node_connections[route_node_idx] = []
            for redundant_connected_node_idx in redundant_route_node_connections[redundant_route_node_idx]:
                connected_node_idx = dict_redundant_route_indices_route_indices[redundant_connected_node_idx]
                route_node_connections[route_node_idx].append(connected_node_idx)
    else:
        ## do not need to remove redundant nodes, create route node points all route nodes
        map_route_node_points = []
        route_node_points = []
        sorted_route_node_indices = list(dict_route_node_indices_points.keys())
        sorted_route_node_indices.sort()
        for route_node_index in sorted_route_node_indices:
            map_route_node_points.append(dict_route_node_indices_map_points[route_node_index])
            route_node_points.append(dict_route_node_indices_points[route_node_index])

    # step4 : create route closeness image and node closeness image

    ## apply distance transform to calculate route closeness image, then convert to range [0, 1]
    # fast version
    # route_dt_image = cv2.distanceTransform(binary_image, distanceType=cv2.DIST_L1, maskSize=cv2.DIST_MASK_3)
    # slow version
    route_dt_image = cv2.distanceTransform(binary_image, distanceType=cv2.DIST_L2, maskSize=cv2.DIST_MASK_5)

    route_dt_image[np.where((grid_data_image<0) | (grid_data_image>=lethal_cost_threshold))] = 0.0

    ## debug visualization
    # cv2.imshow("route distance transform image", cv2.normalize(route_dt_image, None, alpha=0.0, beta=1.0, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F))
    # cv2.waitKey(1)

    return map_route_node_points, route_node_points, route_node_connections, map_route_points_set, route_dt_image


def sample_route_edge_points(costmap, map_route_node_points, map_route_points_set, start_node_index, end_node_index, sample_interval=1.0):
    """
    Sample route edge points

    Parameters
    ----------
    costmap : costmap_utils.Costmap
        The cost map
    map_route_node_points : array-like
        The positions of route nodes in map coordinate
    map_route_points_set : set
        The set of all route points positions in map coordinate
    start_node_index : int
        The start route node index
    end_node_index : int
        The end route node index
    sample_interval : float
        The interval to sample route points

    Returns
    -------
    array-like
        The positions of sampled route edge points from start route node to end route node
    """
    start_route_node_point = map_route_node_points[start_node_index]
    end_route_node_point = map_route_node_points[end_node_index]
    other_map_route_node_points_set = set(map_route_node_points) - set([start_route_node_point, end_route_node_point])

    # traverse from end node to start node, and store distance from end node for all found route points
    point_to_end_distance_dict = {end_route_node_point: 0}
    open_route_point_set = set([end_route_node_point])
    close_route_point_set = set()
    while len(open_route_point_set)>0:
        route_point = open_route_point_set.pop()
        close_route_point_set.add(route_point)

        neighbor_8_points = [(route_point[0]-1, route_point[1]-1), (route_point[0], route_point[1]-1), (route_point[0]+1, route_point[1]-1), \
                        (route_point[0]-1, route_point[1]), (route_point[0]+1, route_point[1]), \
                        (route_point[0]-1, route_point[1]+1), (route_point[0], route_point[1]+1), (route_point[0]+1, route_point[1]+1)]

        has_neighbor_start_route_node_point = False
        neighbor_to_end_distance = point_to_end_distance_dict[route_point] + 1
        for neighbor_point in neighbor_8_points:
            if (neighbor_point in map_route_points_set) and (neighbor_point not in close_route_point_set):
                if neighbor_point not in other_map_route_node_points_set:
                    point_to_end_distance_dict[neighbor_point] = neighbor_to_end_distance
                    open_route_point_set.add(neighbor_point)
                if neighbor_point==start_route_node_point:
                    has_neighbor_start_route_node_point = True

        if has_neighbor_start_route_node_point:
            break

    if start_route_node_point not in point_to_end_distance_dict:
        return []

    # traverse from start node to end node in shortest path
    start_to_end_route_points = [start_route_node_point]
    open_route_point_set = set([start_route_node_point])
    close_route_point_set = set()
    while len(open_route_point_set)>0:
        route_point = open_route_point_set.pop()
        close_route_point_set.add(route_point)

        route_point_to_end_distance = point_to_end_distance_dict[route_point]

        neighbor_8_points = [(route_point[0]-1, route_point[1]-1), (route_point[0], route_point[1]-1), (route_point[0]+1, route_point[1]-1), \
                        (route_point[0]-1, route_point[1]), (route_point[0]+1, route_point[1]), \
                        (route_point[0]-1, route_point[1]+1), (route_point[0], route_point[1]+1), (route_point[0]+1, route_point[1]+1)]

        has_neighbor_end_route_node_point = False
        for neighbor_point in neighbor_8_points:
            if (neighbor_point in point_to_end_distance_dict) and (point_to_end_distance_dict[neighbor_point]<route_point_to_end_distance) and (neighbor_point not in close_route_point_set):
                start_to_end_route_points.append(neighbor_point)
                open_route_point_set.add(neighbor_point)
                if neighbor_point==end_route_node_point:
                    has_neighbor_end_route_node_point = True
                    break

        if has_neighbor_end_route_node_point:
            break

    if len(start_to_end_route_points)<=2:
        return []

    # sample start to end route points including start route node and end route node
    sample_interval_pixels = max(1, int(sample_interval / costmap.resolution))
    sample_start_to_end_route_points = start_to_end_route_points[::sample_interval_pixels]
    if sample_start_to_end_route_points[-1]!=start_to_end_route_points[-1]:
        sample_start_to_end_route_points.append(start_to_end_route_points[-1])

    # remove start route node and end route node from sampled points
    sample_start_to_end_route_points = sample_start_to_end_route_points[1:-1]

    # convert sampled route edge points to world coordinates
    results = []
    for point in sample_start_to_end_route_points:
        world_x, world_y = costmap_utils.map_to_world(costmap.costmap_info, point[0], point[1])
        results.append((world_x, world_y))
    return results


def calc_heading_route_edge_quat(route_node_points, route_node_connections, robot_pose, route_node_index):
    """
    Calculate the closest route edge direction which is connected with input route node and closest to robot heading direction
    First, calculate directions from robot to all route node connected with input route node,
    than select closest direction as heading route direction

    Parameters
    ----------
    route_node_points : array-like
        The positions of route nodes
    route_node_connections : dictionary
        The dictionary of connected route node indices
    robot_pose : geometry_msgs.msg.Pose
        The robot pose
    route_node_index : int
        The route node index which is used to check connected route edges

    Returns
    -------
    geometry_msgs.Quaternion
        Direction of route edge which is closest to robot heading direction
    """
    robot_pose_position = robot_pose.position
    robot_pose_quat = (robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w)

    robot_closest_edge_quat = None
    min_abs_pose_to_edge_yaw_diff = sys.float_info.max
    for connected_node_idx in route_node_connections[route_node_index]:
        edge_quat = geometry_utils.calc_point_direction_quaternion(robot_pose_position, route_node_points[connected_node_idx])

        pose_to_edge_quat_diff = geometry_utils.calc_relative_orientation_quaternion(robot_pose_quat, edge_quat)
        abs_pose_to_edge_yaw_diff = abs(tf.transformations.euler_from_quaternion(pose_to_edge_quat_diff)[2])
        if abs_pose_to_edge_yaw_diff<min_abs_pose_to_edge_yaw_diff:
            robot_closest_edge_quat = edge_quat
            min_abs_pose_to_edge_yaw_diff = abs_pose_to_edge_yaw_diff

        inv_edge_quat = geometry_utils.calc_point_direction_quaternion(route_node_points[connected_node_idx], robot_pose_position)
        pose_to_inv_edge_quat_diff = geometry_utils.calc_relative_orientation_quaternion(robot_pose_quat, inv_edge_quat)
        abs_pose_to_inv_edge_yaw_diff = abs(tf.transformations.euler_from_quaternion(pose_to_inv_edge_quat_diff)[2])
        if abs_pose_to_inv_edge_yaw_diff<min_abs_pose_to_edge_yaw_diff:
            robot_closest_edge_quat = inv_edge_quat
            min_abs_pose_to_edge_yaw_diff = abs_pose_to_inv_edge_yaw_diff

    return robot_closest_edge_quat