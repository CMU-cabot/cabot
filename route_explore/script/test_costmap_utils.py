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
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import tf

from route_explore_utils import costmap_utils, test_utils


def test_map_to_world(map_name, point_list=None, skip_plot=False):
    test_costmap = costmap_utils.load_costmap_file(map_name)

    if point_list is None:
        point_list = np.random.uniform(0, min(test_costmap.width, test_costmap.height), (100, 2))
    print("point_list : " + str(point_list))

    world_point_list = []
    for point in point_list:
        world_x, world_y = costmap_utils.map_to_world(test_costmap.costmap_info, point[0], point[1])
        world_point_list.append([world_x, world_y])
    world_point_list = np.array(world_point_list)
    print("finish map_to_world, world_point_list = " + str(world_point_list))

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

        fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(10,10))

        extent = 0, map_width, 0, map_height
        ax[0].imshow(np.flipud(vis_grid_data), extent=extent)
        ax[0].scatter(point_list[:, 0], point_list[:, 1], s=5, marker='x', color='b')

        extent = map_origin_x, map_origin_x + map_width*map_resolution, map_origin_y, map_origin_y + map_height*map_resolution
        ax[1].imshow(np.flipud(vis_grid_data), extent=extent)
        ax[1].scatter(world_point_list[:, 0], world_point_list[:, 1], s=5, marker='x', color='r')

        plt.show()


def test_world_to_map(map_name, point_list=None, skip_plot=False):
    test_costmap = costmap_utils.load_costmap_file(map_name)

    if point_list is None:
        point_list = np.random.uniform(-3, 3, (100, 2))
    print("point_list : " + str(point_list))

    map_point_list = []
    for point in point_list:
        costmap_x, costmap_y = costmap_utils.world_to_map(test_costmap.costmap_info, point[0], point[1])
        map_point_list.append([costmap_x, costmap_y])
    map_point_list = np.array(map_point_list)
    print("finish world_to_map, map_point_list = " + str(map_point_list))

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

        fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(10,10))

        extent = map_origin_x, map_origin_x + map_width*map_resolution, map_origin_y, map_origin_y + map_height*map_resolution
        ax[0].imshow(np.flipud(vis_grid_data), extent=extent)
        ax[0].scatter(point_list[:, 0], point_list[:, 1], s=5, marker='x', color='b')

        extent = 0, map_width, 0, map_height
        ax[1].imshow(np.flipud(vis_grid_data), extent=extent)
        ax[1].scatter(map_point_list[:, 0], map_point_list[:, 1], s=5, marker='x', color='r')

        plt.show()


def test_map_to_world_np_array(map_name, point_list=None, skip_plot=False):
    test_costmap = costmap_utils.load_costmap_file(map_name)

    if point_list is None:
        point_list = np.random.uniform(0, min(test_costmap.width, test_costmap.height), (100, 2))
    print("point_list : " + str(point_list))

    world_point_list = costmap_utils.map_to_world_np_array(test_costmap.costmap_info, point_list)
    print("finish map_to_world_np_array, world_point_list = " + str(world_point_list))

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

        fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(10,10))

        extent = 0, map_width, 0, map_height
        ax[0].imshow(np.flipud(vis_grid_data), extent=extent)
        ax[0].scatter(point_list[:, 0], point_list[:, 1], s=5, marker='x', color='b')

        extent = map_origin_x, map_origin_x + map_width*map_resolution, map_origin_y, map_origin_y + map_height*map_resolution
        ax[1].imshow(np.flipud(vis_grid_data), extent=extent)
        ax[1].scatter(world_point_list[:, 0], world_point_list[:, 1], s=5, marker='x', color='r')

        plt.show()


def test_world_to_map_np_array(map_name, point_list=None, skip_plot=False):
    test_costmap = costmap_utils.load_costmap_file(map_name)

    if point_list is None:
        point_list = np.random.uniform(-3, 3, (100, 2))
    print("point_list : " + str(point_list))

    map_point_list = costmap_utils.world_to_map_np_array(test_costmap.costmap_info, point_list)
    print("finish world_to_map_np_array, map_point_list = " + str(map_point_list))

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

        fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(10,10))

        extent = map_origin_x, map_origin_x + map_width*map_resolution, map_origin_y, map_origin_y + map_height*map_resolution
        ax[0].imshow(np.flipud(vis_grid_data), extent=extent)
        ax[0].scatter(point_list[:, 0], point_list[:, 1], s=5, marker='x', color='b')

        extent = 0, map_width, 0, map_height
        ax[1].imshow(np.flipud(vis_grid_data), extent=extent)
        ax[1].scatter(map_point_list[:, 0], map_point_list[:, 1], s=5, marker='x', color='r')

        plt.show()


def test_calc_reachable_map(map_name, lethal_cost_threshold=65, skip_plot=False):
    test_costmap = costmap_utils.load_costmap_file(map_name)

    robot_pose, _ = test_utils.get_random_robot_pose(test_costmap, lethal_cost_threshold)
    print("robot_pose : " + str(robot_pose))

    test_costmap_image = np.array(test_costmap.occupancy_grid_data).reshape(test_costmap.height, test_costmap.width)
    test_reachable_costmap_image, _, _ = costmap_utils.calc_reachable_map(test_costmap.costmap_info, test_costmap_image, lethal_cost_threshold, robot_pose)
    print("finish inflate lethal area, test_costmap.costmap_info.occupancy_grid_info = " + str(test_costmap.costmap_info.occupancy_grid_info) \
        + ", test_reachable_costmap_image.shape = " + str(test_reachable_costmap_image.shape))

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

        vis_reachable_grid_data = np.array(test_reachable_costmap_image).astype(float)
        vis_reachable_grid_data = np.where(vis_reachable_grid_data==-1, 0.5, vis_reachable_grid_data/100)
        vis_reachable_grid_data = ((1.0-vis_reachable_grid_data) * 255).astype(np.uint8)
        vis_reachable_grid_data = cv2.cvtColor(vis_reachable_grid_data, cv2.COLOR_GRAY2RGB)

        extent = map_origin_x, map_origin_x + map_width*map_resolution, map_origin_y, map_origin_y + map_height*map_resolution

        fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(10,10))

        ax[0].imshow(np.flipud(vis_grid_data), extent=extent)

        ax[1].imshow(np.flipud(vis_reachable_grid_data), extent=extent)

        plt.show()


def test_inflate_costmap(map_name, lethal_cost_threshold=65, skip_plot=False):
    test_costmap = costmap_utils.load_costmap_file(map_name)

    inflation_radius = 0.3

    test_costmap_image = np.array(test_costmap.occupancy_grid_data).reshape(test_costmap.height, test_costmap.width)
    test_inflate_costmap_image = costmap_utils.inflate_costmap(test_costmap.costmap_info, test_costmap_image, lethal_cost_threshold, inflation_radius)
    print("finish inflate lethal area, test_costmap.costmap_info.occupancy_grid_info = " + str(test_costmap.costmap_info.occupancy_grid_info) \
        + ", test_inflate_costmap_image.shape = " + str(test_inflate_costmap_image.shape))

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

        vis_inflate_grid_data = np.array(test_inflate_costmap_image).astype(float)
        vis_inflate_grid_data = np.where(vis_inflate_grid_data==-1, 0.5, vis_inflate_grid_data/100)
        vis_inflate_grid_data = ((1.0-vis_inflate_grid_data) * 255).astype(np.uint8)
        vis_inflate_grid_data = cv2.cvtColor(vis_inflate_grid_data, cv2.COLOR_GRAY2RGB)

        extent = map_origin_x, map_origin_x + map_width*map_resolution, map_origin_y, map_origin_y + map_height*map_resolution

        fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(10,10))

        ax[0].imshow(np.flipud(vis_grid_data), extent=extent)

        ax[1].imshow(np.flipud(vis_inflate_grid_data), extent=extent)

        plt.show()


def test_calc_frontier_proximity_map(map_name, lethal_cost_threshold=65, skip_plot=False):
    test_costmap = costmap_utils.load_costmap_file(map_name)

    proximity_threshold = 1.0
    min_frontier_size = 1.0

    test_frontier_proximity_map = costmap_utils.calc_frontier_proximity_map(test_costmap, lethal_cost_threshold, proximity_threshold, min_frontier_size=min_frontier_size)
    print("finish inflate lethal area, test_costmap.costmap_info.occupancy_grid_info = " + str(test_costmap.costmap_info.occupancy_grid_info) \
        + ", test_frontier_proximity_map.costmap_info.occupancy_grid_info = " + str(test_frontier_proximity_map.costmap_info.occupancy_grid_info))

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

        vis_frontier_proximity_grid_data = np.array(test_frontier_proximity_map.occupancy_grid_data).reshape(test_frontier_proximity_map.height, test_frontier_proximity_map.width).astype(int)
        assert np.all((vis_frontier_proximity_grid_data==1) | (vis_frontier_proximity_grid_data==0))
        vis_frontier_proximity_grid_data = (vis_frontier_proximity_grid_data * 255).astype(np.uint8)
        vis_frontier_proximity_grid_data = cv2.cvtColor(vis_frontier_proximity_grid_data, cv2.COLOR_GRAY2RGB)

        extent = map_origin_x, map_origin_x + map_width*map_resolution, map_origin_y, map_origin_y + map_height*map_resolution

        fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(10,10))

        ax[0].imshow(np.flipud(vis_grid_data), extent=extent)

        ax[1].imshow(np.flipud(vis_frontier_proximity_grid_data), extent=extent)

        plt.show()


def test_calc_reachable_frontier_proximity_map(map_name, lethal_cost_threshold=65, skip_plot=False):
    test_costmap = costmap_utils.load_costmap_file(map_name)

    robot_pose, _ = test_utils.get_random_robot_pose(test_costmap, lethal_cost_threshold)
    print("robot_pose : " + str(robot_pose))

    proximity_threshold = 1.0
    min_frontier_size = 1.0

    test_frontier_proximity_map = costmap_utils.calc_frontier_proximity_map(test_costmap, lethal_cost_threshold, proximity_threshold, min_frontier_size=min_frontier_size, robot_pose=robot_pose)
    print("finish inflate lethal area, test_costmap.costmap_info.occupancy_grid_info = " + str(test_costmap.costmap_info.occupancy_grid_info) \
        + ", test_frontier_proximity_map.costmap_info.occupancy_grid_info = " + str(test_frontier_proximity_map.costmap_info.occupancy_grid_info))

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

        vis_frontier_proximity_grid_data = np.array(test_frontier_proximity_map.occupancy_grid_data).reshape(test_frontier_proximity_map.height, test_frontier_proximity_map.width).astype(int)
        assert np.all((vis_frontier_proximity_grid_data==1) | (vis_frontier_proximity_grid_data==0))
        vis_frontier_proximity_grid_data = (vis_frontier_proximity_grid_data * 255).astype(np.uint8)
        vis_frontier_proximity_grid_data = cv2.cvtColor(vis_frontier_proximity_grid_data, cv2.COLOR_GRAY2RGB)

        extent = map_origin_x, map_origin_x + map_width*map_resolution, map_origin_y, map_origin_y + map_height*map_resolution

        fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(10,10))

        ax[0].imshow(np.flipud(vis_grid_data), extent=extent)

        ax[1].imshow(np.flipud(vis_frontier_proximity_grid_data), extent=extent)

        plt.show()


def test_calc_local_costmap(map_name, lethal_cost_threshold=65, skip_plot=False):
    test_costmap = costmap_utils.load_costmap_file(map_name)

    robot_pose, _ = test_utils.get_random_robot_pose(test_costmap, lethal_cost_threshold)
    print("robot_pose : " + str(robot_pose))

    local_map_width = 20.0
    local_map_height = 20.0

    robot_pose_yaw = tf.transformations.euler_from_quaternion((robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w))[2]
    # set local map y-coordinate direction as robot heading direction
    local_map_yaw = robot_pose_yaw + math.pi/2.0
    local_costmap = costmap_utils.calc_local_costmap(test_costmap, robot_pose.position.x, robot_pose.position.y, local_map_yaw, local_map_width, local_map_height)
    print("finish calculate local costmap, local_costmap.costmap_info.occupancy_grid_info = " + str(local_costmap.costmap_info.occupancy_grid_info))

    if not skip_plot:
        vis_grid_data = np.array(test_costmap.occupancy_grid_data).reshape(test_costmap.height, test_costmap.width).astype(float)
        vis_grid_data = np.where(vis_grid_data==-1, 0.5, vis_grid_data/100)
        vis_grid_data = ((1.0-vis_grid_data) * 255).astype(np.uint8)
        vis_grid_data = cv2.cvtColor(vis_grid_data, cv2.COLOR_GRAY2RGB)

        vis_local_grid_data = np.array(local_costmap.occupancy_grid_data).reshape(local_costmap.height, local_costmap.width).astype(float)
        vis_local_grid_data = np.where(vis_local_grid_data==-1, 0.5, vis_local_grid_data/100)
        vis_local_grid_data = ((1.0-vis_local_grid_data) * 255).astype(np.uint8)
        vis_local_grid_data = cv2.cvtColor(vis_local_grid_data, cv2.COLOR_GRAY2RGB)

        fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(10,10))

        extent = test_costmap.origin.position.x, test_costmap.origin.position.x + test_costmap.width*test_costmap.resolution, \
            test_costmap.origin.position.y, test_costmap.origin.position.y + test_costmap.height*test_costmap.resolution
        ax[0].imshow(np.flipud(vis_grid_data), extent=extent)
        ax[0].add_patch(matplotlib.patches.Circle(xy=(robot_pose.position.x, robot_pose.position.y), radius=0.3, fill=False, color='b'))
        ax[0].plot([robot_pose.position.x, robot_pose.position.x+0.3*math.cos(robot_pose_yaw)], [robot_pose.position.y, robot_pose.position.y+0.3*math.sin(robot_pose_yaw)], color='b')

        extent = 0.0, local_costmap.width*local_costmap.resolution, 0.0, local_costmap.height*local_costmap.resolution
        local_robot_x = (local_costmap.width*local_costmap.resolution)/2.0
        local_robot_y = (local_costmap.height*local_costmap.resolution)/2.0
        ax[1].imshow(np.flipud(vis_local_grid_data), extent=extent)
        ax[1].add_patch(matplotlib.patches.Circle(xy=(local_robot_x, local_robot_y), radius=0.3, fill=False, color='b'))
        ax[1].plot([local_robot_x, local_robot_x+0.3*math.cos(math.pi/2.0)], [local_robot_y, local_robot_y+0.3*math.sin(-math.pi/2.0)], color='b')

        plt.show()


def test_is_point_in_map(map_name, global_point_list=None, local_point_list=None, lethal_cost_threshold=65, skip_plot=False):
    test_costmap = costmap_utils.load_costmap_file(map_name)

    robot_pose, _ = test_utils.get_random_robot_pose(test_costmap, lethal_cost_threshold)
    print("robot_pose : " + str(robot_pose))

    local_map_width = 20.0
    local_map_height = 20.0

    robot_pose_yaw = tf.transformations.euler_from_quaternion((robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w))[2]
    # set local map y-coordinate direction as robot heading direction
    local_map_yaw = robot_pose_yaw + math.pi/2.0
    local_costmap = costmap_utils.calc_local_costmap(test_costmap, robot_pose.position.x, robot_pose.position.y, local_map_yaw, local_map_width, local_map_height)
    print("finish calculate local costmap, local_costmap.costmap_info.occupancy_grid_info = " + str(local_costmap.costmap_info.occupancy_grid_info))

    if global_point_list is None:
        map_global_point_list = np.random.uniform(-100, max(test_costmap.width, test_costmap.height) + 100, (100, 2))
        global_point_list = costmap_utils.map_to_world_np_array(test_costmap.costmap_info, map_global_point_list)

    if local_point_list is None:
        map_local_point_list = np.random.uniform(-100, max(local_costmap.width, local_costmap.height) + 100, (100, 2))
        local_point_list = costmap_utils.map_to_world_np_array(local_costmap.costmap_info, map_local_point_list)

    is_global_point_in_map = []
    for point in global_point_list:
        is_global_point_in_map.append(costmap_utils.is_point_in_map(test_costmap.costmap_info, point[0], point[1]))

    is_local_point_in_map = []
    for point in local_point_list:
        is_local_point_in_map.append(costmap_utils.is_point_in_map(local_costmap.costmap_info, point[0], point[1]))
    print("finish is_point_in_map")

    if not skip_plot:
        in_global_point_list = global_point_list[is_global_point_in_map,:]
        out_global_point_list = global_point_list[np.logical_not(is_global_point_in_map),:]

        norm_local_point_list = costmap_utils.world_to_map_np_array(local_costmap.costmap_info, local_point_list) * local_costmap.resolution
        in_norm_local_point_list = norm_local_point_list[is_local_point_in_map,:]
        out_norm_local_point_list = norm_local_point_list[np.logical_not(is_local_point_in_map),:]

        vis_grid_data = np.array(test_costmap.occupancy_grid_data).reshape(test_costmap.height, test_costmap.width).astype(float)
        vis_grid_data = np.where(vis_grid_data==-1, 0.5, vis_grid_data/100)
        vis_grid_data = ((1.0-vis_grid_data) * 255).astype(np.uint8)
        vis_grid_data = cv2.cvtColor(vis_grid_data, cv2.COLOR_GRAY2RGB)

        vis_local_grid_data = np.array(local_costmap.occupancy_grid_data).reshape(local_costmap.height, local_costmap.width).astype(float)
        vis_local_grid_data = np.where(vis_local_grid_data==-1, 0.5, vis_local_grid_data/100)
        vis_local_grid_data = ((1.0-vis_local_grid_data) * 255).astype(np.uint8)
        vis_local_grid_data = cv2.cvtColor(vis_local_grid_data, cv2.COLOR_GRAY2RGB)

        fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(10,10))

        extent = test_costmap.origin.position.x, test_costmap.origin.position.x + test_costmap.width*test_costmap.resolution, \
            test_costmap.origin.position.y, test_costmap.origin.position.y + test_costmap.height*test_costmap.resolution
        ax[0].imshow(np.flipud(vis_grid_data), extent=extent)
        ax[0].add_patch(matplotlib.patches.Circle(xy=(robot_pose.position.x, robot_pose.position.y), radius=0.3, fill=False, color='b'))
        ax[0].plot([robot_pose.position.x, robot_pose.position.x+0.3*math.cos(robot_pose_yaw)], [robot_pose.position.y, robot_pose.position.y+0.3*math.sin(robot_pose_yaw)], color='b')
        ax[0].scatter(in_global_point_list[:, 0], in_global_point_list[:, 1], s=5, marker='x', color='g')
        ax[0].scatter(out_global_point_list[:, 0], out_global_point_list[:, 1], s=5, marker='x', color='r')

        extent = 0.0, local_costmap.width*local_costmap.resolution, 0.0, local_costmap.height*local_costmap.resolution
        local_robot_x = (local_costmap.width*local_costmap.resolution)/2.0
        local_robot_y = (local_costmap.height*local_costmap.resolution)/2.0
        ax[1].imshow(np.flipud(vis_local_grid_data), extent=extent)
        ax[1].add_patch(matplotlib.patches.Circle(xy=(local_robot_x, local_robot_y), radius=0.3, fill=False, color='b'))
        ax[1].plot([local_robot_x, local_robot_x+0.3*math.cos(math.pi/2.0)], [local_robot_y, local_robot_y+0.3*math.sin(-math.pi/2.0)], color='b')
        ax[1].scatter(in_norm_local_point_list[:, 0], in_norm_local_point_list[:, 1], s=5, marker='x', color='g')
        ax[1].scatter(out_norm_local_point_list[:, 0], out_norm_local_point_list[:, 1], s=5, marker='x', color='r')

        plt.show()


def main():
    parser = argparse.ArgumentParser(description='Test route_utils.py')
    parser.add_argument('-s', '--skip-plot', action='store_true')
    args = parser.parse_args()
    print("skip plot : " + str(args.skip_plot))

    test_map_to_world("./test/testmap", skip_plot=args.skip_plot)
    test_world_to_map("./test/testmap", skip_plot=args.skip_plot)
    test_map_to_world_np_array("./test/testmap", skip_plot=args.skip_plot)
    test_world_to_map_np_array("./test/testmap", skip_plot=args.skip_plot)
    test_calc_reachable_map("./test/testmap_route_ii", skip_plot=args.skip_plot)
    test_inflate_costmap("./test/testmap_route_x", skip_plot=args.skip_plot)
    test_calc_frontier_proximity_map("./test/testmap_route_ii", skip_plot=args.skip_plot)
    test_calc_reachable_frontier_proximity_map("./test/testmap_route_ii", skip_plot=args.skip_plot)
    test_calc_local_costmap("./test/testmap_route_x", skip_plot=args.skip_plot)
    test_is_point_in_map("./test/testmap_route_x", skip_plot=args.skip_plot)


if __name__=="__main__":
    main()
