#!/usr/bin/python
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

import argparse
import math

import cv2
from geometry_msgs.msg import Point
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree
import tf

from route_explore_utils import costmap_utils, route_utils, test_utils


def test_extract_route(map_name, lethal_cost_threshold=65, skip_plot=False):
    test_costmap = costmap_utils.load_costmap_file(map_name)

    robot_pose, robot_yaw = test_utils.get_random_robot_pose(test_costmap, lethal_cost_threshold)
    print("robot_pose : " + str(robot_pose))

    inflation_radius = 0.3
    minimum_route_length = 1.0
    gaussian_radius = 0.2
    min_detect_l_corner_distance = 1.0
    max_detect_l_corner_angle = math.pi*3.0/4.0

    # inflate obstacle region and frontier unknown region before extract route
    test_costmap_image = np.array(test_costmap.occupancy_grid_data).reshape(test_costmap.height, test_costmap.width)
    test_inflate_costmap_image = costmap_utils.inflate_costmap(test_costmap.costmap_info, test_costmap_image, lethal_cost_threshold, inflation_radius)
    test_inflate_costmap = costmap_utils.Costmap(test_costmap.costmap_info, tuple(test_inflate_costmap_image.flatten()))

    # extract route
    _, route_node_points, route_node_connections, _, route_dt_image = route_utils.extract_route(test_inflate_costmap, lethal_cost_threshold, robot_pose, minimum_route_length,
                                                                                                gaussian_radius=gaussian_radius, min_detect_l_corner_distance=min_detect_l_corner_distance,
                                                                                                max_detect_l_corner_angle=max_detect_l_corner_angle)
    print("finish extract route, len(route_node_points) = " + str(len(route_node_points)) + ", route_node_connections = " + str(route_node_connections))

    if not skip_plot:
        map_resolution = test_costmap.resolution
        map_width = test_costmap.width
        map_height = test_costmap.height
        map_origin_x = test_costmap.origin.position.x
        map_origin_y = test_costmap.origin.position.y

        vis_grid_data = np.array(test_costmap.occupancy_grid_data).reshape(test_costmap.height, test_costmap.width).astype(float)
        vis_grid_data = np.where(vis_grid_data==-1, 0.5, vis_grid_data/100)
        vis_grid_data = ((1.0-vis_grid_data) * 255).astype(np.uint8)
        vis_grid_data = cv2.cvtColor(vis_grid_data, cv2.COLOR_GRAY2RGB)

        vis_route_node_image = np.copy(vis_grid_data)
        for point in route_node_points:
            point_x, point_y = costmap_utils.world_to_map(test_costmap.costmap_info, point[0], point[1])
            cv2.circle(vis_route_node_image, (point_x, point_y), radius=3, color=(255, 0, 0), thickness=cv2.FILLED)
        for node_index1 in route_node_connections.keys():
            for node_index2 in route_node_connections[node_index1]:
                if node_index1<node_index2:
                    point_x1, point_y1 = costmap_utils.world_to_map(test_costmap.costmap_info, route_node_points[node_index1][0], route_node_points[node_index1][1])
                    point_x2, point_y2 = costmap_utils.world_to_map(test_costmap.costmap_info, route_node_points[node_index2][0], route_node_points[node_index2][1])
                    cv2.line(vis_route_node_image, (point_x1, point_y1), (point_x2, point_y2), color=(0, 0, 255), thickness=2)

        vis_route_closeness_image = cv2.normalize(route_dt_image, None, alpha=0.0, beta=1.0, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
        vis_route_closeness_image = (vis_route_closeness_image * 255).astype(np.uint8)
        vis_route_closeness_image = cv2.cvtColor(vis_route_closeness_image, cv2.COLOR_GRAY2RGB)

        extent = map_origin_x, map_origin_x + map_width*map_resolution, map_origin_y, map_origin_y + map_height*map_resolution

        fig, ax = plt.subplots(nrows=1, ncols=3, figsize=(10,10))

        ax[0].imshow(np.flipud(vis_grid_data), extent=extent)
        ax[0].add_patch(matplotlib.patches.Circle(xy=(robot_pose.position.x, robot_pose.position.y), radius=0.3, fill=False, color='b'))
        ax[0].plot([robot_pose.position.x, robot_pose.position.x+0.3*math.cos(robot_yaw)], [robot_pose.position.y, robot_pose.position.y+0.3*math.sin(robot_yaw)], color='b')

        ax[1].imshow(np.flipud(vis_route_node_image), extent=extent)
        ax[1].add_patch(matplotlib.patches.Circle(xy=(robot_pose.position.x, robot_pose.position.y), radius=0.3, fill=False, color='b'))
        ax[1].plot([robot_pose.position.x, robot_pose.position.x+0.3*math.cos(robot_yaw)], [robot_pose.position.y, robot_pose.position.y+0.3*math.sin(robot_yaw)], color='b')

        ax[2].imshow(np.flipud(vis_route_closeness_image), extent=extent)
        ax[2].add_patch(matplotlib.patches.Circle(xy=(robot_pose.position.x, robot_pose.position.y), radius=0.3, fill=False, color='b'))
        ax[2].plot([robot_pose.position.x, robot_pose.position.x+0.3*math.cos(robot_yaw)], [robot_pose.position.y, robot_pose.position.y+0.3*math.sin(robot_yaw)], color='b')

        plt.show()


def test_sample_route_edge_points(map_name, lethal_cost_threshold=65, skip_plot=False):
    test_costmap = costmap_utils.load_costmap_file(map_name)

    robot_pose, robot_yaw = test_utils.get_random_robot_pose(test_costmap, lethal_cost_threshold)
    print("robot_pose : " + str(robot_pose))

    inflation_radius = 0.3
    minimum_route_length = 1.0
    gaussian_radius = 0.2
    min_detect_l_corner_distance = 1.0
    max_detect_l_corner_angle = math.pi*3.0/4.0
    sample_interval = 1.0

    # inflate obstacle region and frontier unknown region before extract route
    test_costmap_image = np.array(test_costmap.occupancy_grid_data).reshape(test_costmap.height, test_costmap.width)
    test_inflate_costmap_image = costmap_utils.inflate_costmap(test_costmap.costmap_info, test_costmap_image, lethal_cost_threshold, inflation_radius)
    test_inflate_costmap = costmap_utils.Costmap(test_costmap.costmap_info, tuple(test_inflate_costmap_image.flatten()))

    # extract route
    map_route_node_points, route_node_points, route_node_connections, map_route_points_set, _ = route_utils.extract_route(test_inflate_costmap, lethal_cost_threshold, robot_pose, minimum_route_length,
                                                                                                                        gaussian_radius=gaussian_radius, min_detect_l_corner_distance=min_detect_l_corner_distance,
                                                                                                                        max_detect_l_corner_angle=max_detect_l_corner_angle)
    route_node_points_kd_tree = cKDTree(route_node_points)
    _, start_node_index = route_node_points_kd_tree.query([robot_pose.position.x, robot_pose.position.y], k=1)
    end_node_index = route_node_connections[start_node_index][0]
    sample_route_edge_points = route_utils.sample_route_edge_points(test_inflate_costmap, map_route_node_points, map_route_points_set, start_node_index, end_node_index, sample_interval=sample_interval)
    print("finish sample route edge points, sample_route_edge_points = " + str(sample_route_edge_points))
    print("finish sample route edge points, len(sample_route_points) = " + str(len(sample_route_edge_points)))

    if not skip_plot:
        map_resolution = test_costmap.resolution
        map_width = test_costmap.width
        map_height = test_costmap.height
        map_origin_x = test_costmap.origin.position.x
        map_origin_y = test_costmap.origin.position.y

        vis_grid_data = np.array(test_costmap.occupancy_grid_data).reshape(test_costmap.height, test_costmap.width).astype(float)
        vis_grid_data = np.where(vis_grid_data==-1, 0.5, vis_grid_data/100)
        vis_grid_data = ((1.0-vis_grid_data) * 255).astype(np.uint8)
        vis_grid_data = cv2.cvtColor(vis_grid_data, cv2.COLOR_GRAY2RGB)

        vis_route_node_image = np.copy(vis_grid_data)
        for point in route_node_points:
            point_x, point_y = costmap_utils.world_to_map(test_costmap.costmap_info, point[0], point[1])
            cv2.circle(vis_route_node_image, (point_x, point_y), radius=3, color=(255, 0, 0), thickness=cv2.FILLED)
        for node_index1 in route_node_connections.keys():
            for node_index2 in route_node_connections[node_index1]:
                if node_index1<node_index2:
                    point_x1, point_y1 = costmap_utils.world_to_map(test_costmap.costmap_info, route_node_points[node_index1][0], route_node_points[node_index1][1])
                    point_x2, point_y2 = costmap_utils.world_to_map(test_costmap.costmap_info, route_node_points[node_index2][0], route_node_points[node_index2][1])
                    cv2.line(vis_route_node_image, (point_x1, point_y1), (point_x2, point_y2), color=(0, 0, 255), thickness=2)

        vis_sample_route_point_image = np.copy(vis_grid_data)
        for point in route_node_points:
            point_x, point_y = costmap_utils.world_to_map(test_costmap.costmap_info, point[0], point[1])
            cv2.circle(vis_sample_route_point_image, (point_x, point_y), radius=3, color=(255, 0, 0), thickness=cv2.FILLED)
        for point in sample_route_edge_points:
            point_x, point_y = costmap_utils.world_to_map(test_costmap.costmap_info, point[0], point[1])
            cv2.circle(vis_sample_route_point_image, (point_x, point_y), radius=3, color=(0, 255, 0), thickness=cv2.FILLED)

        extent = map_origin_x, map_origin_x + map_width*map_resolution, map_origin_y, map_origin_y + map_height*map_resolution

        fig, ax = plt.subplots(nrows=1, ncols=3, figsize=(10,10))

        ax[0].imshow(np.flipud(vis_grid_data), extent=extent)
        ax[0].add_patch(matplotlib.patches.Circle(xy=(robot_pose.position.x, robot_pose.position.y), radius=0.3, fill=False, color='b'))
        ax[0].plot([robot_pose.position.x, robot_pose.position.x+0.3*math.cos(robot_yaw)], [robot_pose.position.y, robot_pose.position.y+0.3*math.sin(robot_yaw)], color='b')

        ax[1].imshow(np.flipud(vis_route_node_image), extent=extent)
        ax[1].add_patch(matplotlib.patches.Circle(xy=(robot_pose.position.x, robot_pose.position.y), radius=0.3, fill=False, color='b'))
        ax[1].plot([robot_pose.position.x, robot_pose.position.x+0.3*math.cos(robot_yaw)], [robot_pose.position.y, robot_pose.position.y+0.3*math.sin(robot_yaw)], color='b')

        ax[2].imshow(np.flipud(vis_sample_route_point_image), extent=extent)
        ax[2].add_patch(matplotlib.patches.Circle(xy=(robot_pose.position.x, robot_pose.position.y), radius=0.3, fill=False, color='b'))
        ax[2].plot([robot_pose.position.x, robot_pose.position.x+0.3*math.cos(robot_yaw)], [robot_pose.position.y, robot_pose.position.y+0.3*math.sin(robot_yaw)], color='b')

        plt.show()


def test_calc_heading_route_edge_quat(map_name, lethal_cost_threshold=65, skip_plot=False):
    test_costmap = costmap_utils.load_costmap_file(map_name)

    robot_pose, robot_yaw = test_utils.get_random_robot_pose(test_costmap, lethal_cost_threshold)
    print("robot_pose : " + str(robot_pose))

    inflation_radius = 0.3
    minimum_route_length = 1.0
    gaussian_radius = 0.2
    min_detect_l_corner_distance = 1.0
    max_detect_l_corner_angle = math.pi*3.0/4.0

    # inflate obstacle region and frontier unknown region before extract route
    test_costmap_image = np.array(test_costmap.occupancy_grid_data).reshape(test_costmap.height, test_costmap.width)
    test_inflate_costmap_image = costmap_utils.inflate_costmap(test_costmap.costmap_info, test_costmap_image, lethal_cost_threshold, inflation_radius)
    test_inflate_costmap = costmap_utils.Costmap(test_costmap.costmap_info, tuple(test_inflate_costmap_image.flatten()))

    # extract route
    map_route_node_points, route_node_points, route_node_connections, map_route_points_set, _ = route_utils.extract_route(test_inflate_costmap, lethal_cost_threshold, robot_pose, minimum_route_length,
                                                                                                                        gaussian_radius=gaussian_radius, min_detect_l_corner_distance=min_detect_l_corner_distance,
                                                                                                                        max_detect_l_corner_angle=max_detect_l_corner_angle)
    route_node_points_kd_tree = cKDTree(route_node_points)
    _, robot_closest_node_index = route_node_points_kd_tree.query([robot_pose.position.x, robot_pose.position.y], k=1)
    heading_route_edge_quat = route_utils.calc_heading_route_edge_quat([Point(point[0], point[1], 0.0) for point in route_node_points], route_node_connections, robot_pose, robot_closest_node_index)
    heading_route_edge_yaw = tf.transformations.euler_from_quaternion(heading_route_edge_quat)[2]
    print("finish calculate heading route edge quaternion, heading_route_edge_yaw = " + str(heading_route_edge_yaw))

    if not skip_plot:
        map_resolution = test_costmap.resolution
        map_width = test_costmap.width
        map_height = test_costmap.height
        map_origin_x = test_costmap.origin.position.x
        map_origin_y = test_costmap.origin.position.y

        vis_grid_data = np.array(test_costmap.occupancy_grid_data).reshape(test_costmap.height, test_costmap.width).astype(float)
        vis_grid_data = np.where(vis_grid_data==-1, 0.5, vis_grid_data/100)
        vis_grid_data = ((1.0-vis_grid_data) * 255).astype(np.uint8)
        vis_grid_data = cv2.cvtColor(vis_grid_data, cv2.COLOR_GRAY2RGB)

        vis_route_node_image = np.copy(vis_grid_data)
        for point in route_node_points:
            point_x, point_y = costmap_utils.world_to_map(test_costmap.costmap_info, point[0], point[1])
            cv2.circle(vis_route_node_image, (point_x, point_y), radius=3, color=(255, 0, 0), thickness=cv2.FILLED)
        for node_index1 in route_node_connections.keys():
            for node_index2 in route_node_connections[node_index1]:
                if node_index1<node_index2:
                    point_x1, point_y1 = costmap_utils.world_to_map(test_costmap.costmap_info, route_node_points[node_index1][0], route_node_points[node_index1][1])
                    point_x2, point_y2 = costmap_utils.world_to_map(test_costmap.costmap_info, route_node_points[node_index2][0], route_node_points[node_index2][1])
                    cv2.line(vis_route_node_image, (point_x1, point_y1), (point_x2, point_y2), color=(0, 0, 255), thickness=2)

        extent = map_origin_x, map_origin_x + map_width*map_resolution, map_origin_y, map_origin_y + map_height*map_resolution

        fig, ax = plt.subplots(nrows=1, ncols=3, figsize=(10,10))

        ax[0].imshow(np.flipud(vis_grid_data), extent=extent)
        ax[0].add_patch(matplotlib.patches.Circle(xy=(robot_pose.position.x, robot_pose.position.y), radius=0.3, fill=False, color='b'))
        ax[0].plot([robot_pose.position.x, robot_pose.position.x+0.3*math.cos(robot_yaw)], [robot_pose.position.y, robot_pose.position.y+0.3*math.sin(robot_yaw)], color='b')

        ax[1].imshow(np.flipud(vis_route_node_image), extent=extent)
        ax[1].add_patch(matplotlib.patches.Circle(xy=(robot_pose.position.x, robot_pose.position.y), radius=0.3, fill=False, color='b'))
        ax[1].plot([robot_pose.position.x, robot_pose.position.x+0.3*math.cos(robot_yaw)], [robot_pose.position.y, robot_pose.position.y+0.3*math.sin(robot_yaw)], color='b')

        ax[2].imshow(np.flipud(vis_route_node_image), extent=extent)
        ax[2].add_patch(matplotlib.patches.Circle(xy=(robot_pose.position.x, robot_pose.position.y), radius=0.3, fill=False, color='g'))
        ax[2].plot([robot_pose.position.x, robot_pose.position.x+0.3*math.cos(heading_route_edge_yaw)], [robot_pose.position.y, robot_pose.position.y+0.3*math.sin(heading_route_edge_yaw)], color='g')

        plt.show()


def main():
    parser = argparse.ArgumentParser(description='Test route_utils.py')
    parser.add_argument('-s', '--skip-plot', action='store_true')
    args = parser.parse_args()
    print("skip plot : " + str(args.skip_plot))

    test_extract_route("./test/testmap_route_x", skip_plot=args.skip_plot)
    test_sample_route_edge_points("./test/testmap_route_x", skip_plot=args.skip_plot)
    test_calc_heading_route_edge_quat("./test/testmap_route_x", skip_plot=args.skip_plot)


if __name__=="__main__":
    main()
