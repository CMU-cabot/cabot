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
from scipy.spatial import ConvexHull

from route_explore_utils import costmap_utils, geometry_utils, test_utils


def test_is_line_segments_intersect(line_points=None, skip_plot=False):
    if line_points is None:
        line_points = np.random.uniform(-10, 10, (4, 2))
    print("line_points : " + str(line_points))

    is_intersect = geometry_utils.is_line_segments_intersect(line_points[0], line_points[1], line_points[2], line_points[3])
    print("finish test_is_line_segments_intersect, is_intersect = " + str(is_intersect))

    if not skip_plot:
        color = "green"
        if is_intersect:
            color = "red"
        collection = matplotlib.collections.LineCollection([[line_points[0], line_points[1]], [line_points[2], line_points[3]]], color=[color, color])
        fig,axes=plt.subplots(1,1)
        axes.add_collection(collection)
        axes.autoscale()
        plt.show()


def test_is_line_segment_convex_hull_intersect(hull_points=None, line_points=None, skip_plot=False):
    if hull_points is None:
        hull_points = np.random.uniform(-5, 5, (100, 2))
    print("hull_points : " + str(hull_points))
    convex_hell = ConvexHull(hull_points)

    if line_points is None:
        line_points = np.random.uniform(-10, 10, (2, 2))
    print("line_points : " + str(line_points))

    is_intersect = geometry_utils.is_line_segment_convex_hull_intersect(line_points[0], line_points[1], convex_hell)
    print("finish test_is_line_segment_convex_hull_intersect, is_intersect = " + str(is_intersect))

    if not skip_plot:
        lines = []
        line_colors = []
        for simplex in convex_hell.simplices:
            assert(len(simplex)==2)

            simplex_point1 = [convex_hell.points[simplex[0], 0], convex_hell.points[simplex[0], 1]]
            simplex_point2 = [convex_hell.points[simplex[1], 0], convex_hell.points[simplex[1], 1]]
            lines.append([simplex_point1, simplex_point2])
            line_colors.append("blue")

        lines.append([line_points[0], line_points[1]])
        if is_intersect:
            line_colors.append("red")
        else:
            line_colors.append("green")

        collection = matplotlib.collections.LineCollection(lines, color=line_colors)
        fig,axes=plt.subplots(1,1)
        axes.add_collection(collection)
        axes.autoscale()
        plt.show()


def test_check_is_point_free(map_name, lethal_cost_threshold=65, point_list=None, skip_plot=False):
    test_costmap = costmap_utils.load_costmap_file(map_name)

    if point_list is None:
        point_list = np.random.uniform(-3, 3, (100, 2))
    print("point_list : " + str(point_list))

    is_point_free = []
    for point in point_list:
        is_point_free.append(geometry_utils.check_is_point_free(test_costmap, lethal_cost_threshold, Point(point[0], point[1], 0.0)))
    print("finish test_check_is_point_free, is_point_free = " + str(is_point_free))

    if not skip_plot:
        in_point_list = point_list[is_point_free,:]
        out_point_list = point_list[np.logical_not(is_point_free),:]

        map_resolution = test_costmap.resolution
        map_width = test_costmap.width
        map_height = test_costmap.height
        map_origin_x = test_costmap.origin.position.x
        map_origin_y = test_costmap.origin.position.y

        vis_grid_data = np.array(test_costmap.occupancy_grid_data).reshape(test_costmap.height, test_costmap.width).astype(float)
        vis_grid_data = np.where(vis_grid_data==-1, 0.5, vis_grid_data/100)
        vis_grid_data = ((1.0-vis_grid_data) * 255).astype(np.uint8)
        vis_grid_data = cv2.cvtColor(vis_grid_data, cv2.COLOR_GRAY2RGB)

        extent = map_origin_x, map_origin_x + map_width*map_resolution, map_origin_y, map_origin_y + map_height*map_resolution

        fig,axes=plt.subplots(1,1)
        axes.imshow(np.flipud(vis_grid_data), extent=extent)
        axes.scatter(in_point_list[:, 0], in_point_list[:, 1], s=5, marker='x', color='r')
        axes.scatter(out_point_list[:, 0], out_point_list[:, 1], s=5, marker='x', color='g')
        plt.show()


def test_check_is_in_line_of_sight(map_name, lethal_cost_threshold=65, point_list=None, skip_plot=False):
    test_costmap = costmap_utils.load_costmap_file(map_name)

    if point_list is None:
        point_list = []
        while len(point_list)<10:
            point = np.random.uniform(-3, 3, (2))
            if geometry_utils.check_is_point_free(test_costmap, lethal_cost_threshold, Point(point[0], point[1], 0.0)):
                point_list.append(point)
        point_list = np.array(point_list)
    print("point_list : " + str(point_list))

    grid_data_image = np.array(test_costmap.occupancy_grid_data, dtype=np.int8).reshape(test_costmap.height, test_costmap.width)
    binary_non_free_image = costmap_utils.calc_binary_non_free_image(grid_data_image, lethal_cost_threshold)

    is_in_line_of_sight = {}
    for point_idx1 in range(len(point_list))[:-1]:
        point1 = Point(point_list[point_idx1][0], point_list[point_idx1][1], 0.0)
        for point_idx2 in range(len(point_list))[point_idx1+1:]:
            point2 = Point(point_list[point_idx2][0], point_list[point_idx2][1], 0.0)
            is_in_line_of_sight[(point_idx1, point_idx2)] = geometry_utils.check_is_in_line_of_sight(test_costmap.costmap_info, binary_non_free_image, point1, point2)
    print("finish test_check_is_in_line_of_sight")

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

        extent = map_origin_x, map_origin_x + map_width*map_resolution, map_origin_y, map_origin_y + map_height*map_resolution

        fig,axes=plt.subplots(1,1)
        axes.imshow(np.flipud(vis_grid_data), extent=extent)
        axes.scatter(point_list[:, 0], point_list[:, 1], s=5, marker='x', color='g')
        lines = []
        line_colors = []
        for point_idx1 in range(len(point_list))[:-1]:
            for point_idx2 in range(len(point_list))[point_idx1+1:]:
                lines.append([point_list[point_idx1], point_list[point_idx2]])
                if is_in_line_of_sight[(point_idx1, point_idx2)]:
                    line_colors.append("blue")
                else:
                    line_colors.append("red")
        collection = matplotlib.collections.LineCollection(lines, color=line_colors)
        axes.add_collection(collection)
        plt.show()


def test_calc_coverage_polyhedron(map_name, lethal_cost_threshold=65, skip_plot=False):
    test_costmap = costmap_utils.load_costmap_file(map_name)

    robot_pose, robot_yaw = test_utils.get_random_robot_pose(test_costmap, lethal_cost_threshold)
    print("robot_pose : " + str(robot_pose))

    polyhedron_sensor_range = 3.0
    num_sample_coverage_polyhedron_point = 36
    angles_sample_coverage_polyhedron = [i*(math.pi*2.0)/float(num_sample_coverage_polyhedron_point) for i in range(num_sample_coverage_polyhedron_point)]
    angles_sample_coverage_polyhedron = [angle if angle<=math.pi else -(math.pi*2.0-angle) for angle in angles_sample_coverage_polyhedron]
    polyhedron_buffer_length = 0.2
    polyhedron_buffer_samples = 12

    coverage_polyhedron = geometry_utils.calc_coverage_polyhedron(test_costmap, lethal_cost_threshold, robot_pose, polyhedron_sensor_range, angles_sample_coverage_polyhedron,
                                                                polyhedron_buffer_length=polyhedron_buffer_length, polyhedron_buffer_samples=polyhedron_buffer_samples)
    print("finish calc_coverage_polyhedron, coverage_polyhedron = " + str(coverage_polyhedron))

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

        extent = map_origin_x, map_origin_x + map_width*map_resolution, map_origin_y, map_origin_y + map_height*map_resolution

        x, y = np.mgrid[slice(map_origin_x, map_origin_x + map_width*map_resolution, map_resolution),
                        slice(map_origin_y, map_origin_y + map_height*map_resolution, map_resolution)]
        fig,axes=plt.subplots(1,1)
        axes.imshow(np.flipud(vis_grid_data), extent=extent)

        axes.scatter([point[0] for point in coverage_polyhedron.points[coverage_polyhedron.vertices]], [point[1] for point in coverage_polyhedron.points[coverage_polyhedron.vertices]], s=5, marker='x', color='b')
        lines = []
        line_colors = []
        if coverage_polyhedron is not None:
            for simplex in coverage_polyhedron.simplices:
                assert(len(simplex)==2)
                simplex_point1 = [coverage_polyhedron.points[simplex[0], 0], coverage_polyhedron.points[simplex[0], 1]]
                simplex_point2 = [coverage_polyhedron.points[simplex[1], 0], coverage_polyhedron.points[simplex[1], 1]]
                lines.append([simplex_point1, simplex_point2])
                line_colors.append("blue")
            collection = matplotlib.collections.LineCollection(lines, color=line_colors)
            axes.add_collection(collection)
        axes.add_patch(matplotlib.patches.Circle(xy=(robot_pose.position.x, robot_pose.position.y), radius=0.1, fill=False, color='b'))
        axes.plot([robot_pose.position.x, robot_pose.position.x+0.1*math.cos(robot_yaw)], [robot_pose.position.y, robot_pose.position.y+0.1*math.sin(robot_yaw)], color='b')
        plt.show()


def test_is_point_in_area(map_name, lethal_cost_threshold=65, point_list=None, skip_plot=False):
    test_costmap = costmap_utils.load_costmap_file(map_name)

    if point_list is None:
        point_list = np.random.uniform(-3, 3, (100, 2))
    print("point_list : " + str(point_list))

    robot_pose, robot_yaw = test_utils.get_random_robot_pose(test_costmap, lethal_cost_threshold)
    print("robot_pose : " + str(robot_pose))

    polyhedron_sensor_range = 3.0
    num_sample_coverage_polyhedron_point = 36
    angles_sample_coverage_polyhedron = [i*(math.pi*2.0)/float(num_sample_coverage_polyhedron_point) for i in range(num_sample_coverage_polyhedron_point)]
    angles_sample_coverage_polyhedron = [angle if angle<=math.pi else -(math.pi*2.0-angle) for angle in angles_sample_coverage_polyhedron]
    polyhedron_buffer_length = 0.2
    polyhedron_buffer_samples = 12

    coverage_polyhedron = geometry_utils.calc_coverage_polyhedron(test_costmap, lethal_cost_threshold, robot_pose, polyhedron_sensor_range, angles_sample_coverage_polyhedron,
                                                                polyhedron_buffer_length=polyhedron_buffer_length, polyhedron_buffer_samples=polyhedron_buffer_samples)
    binary_coverage_polyhedron_image = costmap_utils.calc_binary_polyhedron_image(test_costmap.costmap_info, coverage_polyhedron)
    binary_coverage_polyhedron_map = costmap_utils.Costmap(test_costmap.costmap_info, tuple(binary_coverage_polyhedron_image.flatten().astype(np.uint8)))

    is_in_area = []
    for point in point_list:
        is_in_area.append(geometry_utils.is_point_in_area(binary_coverage_polyhedron_map, Point(point[0], point[1], 0.0)))
    print("finish test_is_point_in_area, is_in_area = " + str(is_in_area))

    if not skip_plot:
        in_point_list = point_list[is_in_area,:]
        out_point_list = point_list[np.logical_not(is_in_area),:]
    
        map_resolution = test_costmap.resolution
        map_width = test_costmap.width
        map_height = test_costmap.height
        map_origin_x = test_costmap.origin.position.x
        map_origin_y = test_costmap.origin.position.y

        vis_grid_data = np.array(test_costmap.occupancy_grid_data).reshape(test_costmap.height, test_costmap.width).astype(float)
        vis_grid_data = np.where(vis_grid_data==-1, 0.5, vis_grid_data/100)
        vis_grid_data = ((1.0-vis_grid_data) * 255).astype(np.uint8)
        vis_grid_data = cv2.cvtColor(vis_grid_data, cv2.COLOR_GRAY2RGB)

        extent = map_origin_x, map_origin_x + map_width*map_resolution, map_origin_y, map_origin_y + map_height*map_resolution

        x, y = np.mgrid[slice(map_origin_x, map_origin_x + map_width*map_resolution, map_resolution),
                        slice(map_origin_y, map_origin_y + map_height*map_resolution, map_resolution)]
        fig,axes=plt.subplots(1,1)
        axes.imshow(np.flipud(vis_grid_data), extent=extent)

        axes.scatter([point[0] for point in coverage_polyhedron.points[coverage_polyhedron.vertices]], [point[1] for point in coverage_polyhedron.points[coverage_polyhedron.vertices]], s=5, marker='x', color='b')
        lines = []
        line_colors = []
        if coverage_polyhedron is not None:
            for simplex in coverage_polyhedron.simplices:
                assert(len(simplex)==2)
                simplex_point1 = [coverage_polyhedron.points[simplex[0], 0], coverage_polyhedron.points[simplex[0], 1]]
                simplex_point2 = [coverage_polyhedron.points[simplex[1], 0], coverage_polyhedron.points[simplex[1], 1]]
                lines.append([simplex_point1, simplex_point2])
                line_colors.append("blue")
            collection = matplotlib.collections.LineCollection(lines, color=line_colors)
            axes.add_collection(collection)
        axes.add_patch(matplotlib.patches.Circle(xy=(robot_pose.position.x, robot_pose.position.y), radius=0.1, fill=False, color='b'))
        axes.scatter(in_point_list[:, 0], in_point_list[:, 1], s=5, marker='x', color='r')
        axes.scatter(out_point_list[:, 0], out_point_list[:, 1], s=5, marker='x', color='g')
        axes.plot([robot_pose.position.x, robot_pose.position.x+0.1*math.cos(robot_yaw)], [robot_pose.position.y, robot_pose.position.y+0.1*math.sin(robot_yaw)], color='b')
        plt.show()


def test_calc_visible_area_contour(map_name, lethal_cost_threshold=65, skip_plot=False):
    test_costmap = costmap_utils.load_costmap_file(map_name)

    robot_pose, robot_yaw = test_utils.get_random_robot_pose(test_costmap, lethal_cost_threshold)
    print("robot_pose : " + str(robot_pose))

    sample_range = 5.0
    num_sample_visible_contour_point = 72
    angles_sample_visible_contour = [i*(math.pi*2.0)/float(num_sample_visible_contour_point) for i in range(num_sample_visible_contour_point)]
    angles_sample_visible_contour = [angle if angle<=math.pi else -(math.pi*2.0-angle) for angle in angles_sample_visible_contour]

    visible_area_contour = geometry_utils.calc_visible_area_contour(test_costmap, lethal_cost_threshold, robot_pose, sample_range, angles_sample_visible_contour)
    print("finish calc_visible_area_contour, visible_area_contour = " + str(visible_area_contour))

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

        map_visible_area_contour = costmap_utils.world_to_map_np_array(test_costmap.costmap_info, visible_area_contour)
        cv2.drawContours(vis_grid_data, [map_visible_area_contour], -1, color=[255,0,0], thickness=-1)
        cv2.drawContours(vis_grid_data, [map_visible_area_contour], -1, color=[255,0,0], thickness=1)

        extent = map_origin_x, map_origin_x + map_width*map_resolution, map_origin_y, map_origin_y + map_height*map_resolution

        x, y = np.mgrid[slice(map_origin_x, map_origin_x + map_width*map_resolution, map_resolution),
                        slice(map_origin_y, map_origin_y + map_height*map_resolution, map_resolution)]
        fig,axes=plt.subplots(1,1)
        axes.imshow(np.flipud(vis_grid_data), extent=extent)

        axes.scatter([point[0] for point in visible_area_contour], [point[1] for point in visible_area_contour], s=5, marker='x', color='b')
        axes.add_patch(matplotlib.patches.Circle(xy=(robot_pose.position.x, robot_pose.position.y), radius=0.1, fill=False, color='b'))
        axes.plot([robot_pose.position.x, robot_pose.position.x+0.1*math.cos(robot_yaw)], [robot_pose.position.y, robot_pose.position.y+0.1*math.sin(robot_yaw)], color='b')
        plt.show()


def test_calc_contours_outside_coverage_polyhedron(map_name, lethal_cost_threshold=65, skip_plot=False):
    test_costmap = costmap_utils.load_costmap_file(map_name)

    robot_pose, robot_yaw = test_utils.get_random_robot_pose(test_costmap, lethal_cost_threshold)
    print("robot_pose : " + str(robot_pose))

    polyhedron_sensor_range = 3.0
    num_sample_coverage_polyhedron_point = 36
    ignore_small_area_square_meters = 1.0
    angles_sample_coverage_polyhedron = [i*(math.pi*2.0)/float(num_sample_coverage_polyhedron_point) for i in range(num_sample_coverage_polyhedron_point)]
    angles_sample_coverage_polyhedron = [angle if angle<=math.pi else -(math.pi*2.0-angle) for angle in angles_sample_coverage_polyhedron]
    polyhedron_buffer_length = 0.2
    polyhedron_buffer_samples = 12

    grid_data_image = np.array(test_costmap.occupancy_grid_data, dtype=np.int8).reshape(test_costmap.height, test_costmap.width)
    binary_free_image = costmap_utils.calc_binary_free_image(grid_data_image, lethal_cost_threshold)

    coverage_polyhedron = geometry_utils.calc_coverage_polyhedron(test_costmap, lethal_cost_threshold, robot_pose, polyhedron_sensor_range, angles_sample_coverage_polyhedron,
                                                                polyhedron_buffer_length=polyhedron_buffer_length, polyhedron_buffer_samples=polyhedron_buffer_samples)
    binary_coverage_polyhedron_image = costmap_utils.calc_binary_polyhedron_image(test_costmap.costmap_info, coverage_polyhedron)
    binary_coverage_polyhedron_map = costmap_utils.Costmap(test_costmap.costmap_info, tuple(binary_coverage_polyhedron_image.flatten().astype(np.uint8)))

    free_contours_outside_polyhedron, free_contours_outside_polyhedron_id_map = geometry_utils.calc_contours_outside_coverage_polyhedron(binary_coverage_polyhedron_map, binary_free_image,
                                                                                                                                        ignore_small_area_square_meters=ignore_small_area_square_meters)
    print("finish calc_contours_outside_coverage_polyhedron, free_contours_outside_polyhedron = " + str(free_contours_outside_polyhedron))

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

        free_contours_outside_polyhedron_image = np.array(free_contours_outside_polyhedron_id_map.occupancy_grid_data, dtype=np.int8).reshape(free_contours_outside_polyhedron_id_map.height, free_contours_outside_polyhedron_id_map.width)
        vis_free_contours_outside_polyhedron_image = np.zeros(free_contours_outside_polyhedron_image.shape + (3,), dtype=np.uint8)
        free_contours_outside_polyhedron_ids = np.unique(free_contours_outside_polyhedron_image)
        for free_contours_outside_polyhedron_id in free_contours_outside_polyhedron_ids:
            if free_contours_outside_polyhedron_id>0:
                vis_free_contours_outside_polyhedron_image[free_contours_outside_polyhedron_image==free_contours_outside_polyhedron_id] = [np.random.randint(0, 256), np.random.randint(0, 256), np.random.randint(0, 256)]

        extent = map_origin_x, map_origin_x + map_width*map_resolution, map_origin_y, map_origin_y + map_height*map_resolution

        x, y = np.mgrid[slice(map_origin_x, map_origin_x + map_width*map_resolution, map_resolution),
                        slice(map_origin_y, map_origin_y + map_height*map_resolution, map_resolution)]
        fig, axes=plt.subplots(1,2)
        axes[0].imshow(np.flipud(vis_grid_data), extent=extent)

        axes[0].scatter([point[0] for point in coverage_polyhedron.points[coverage_polyhedron.vertices]], [point[1] for point in coverage_polyhedron.points[coverage_polyhedron.vertices]], s=5, marker='x', color='b')
        lines = []
        line_colors = []
        if coverage_polyhedron is not None:
            for simplex in coverage_polyhedron.simplices:
                assert(len(simplex)==2)
                simplex_point1 = [coverage_polyhedron.points[simplex[0], 0], coverage_polyhedron.points[simplex[0], 1]]
                simplex_point2 = [coverage_polyhedron.points[simplex[1], 0], coverage_polyhedron.points[simplex[1], 1]]
                lines.append([simplex_point1, simplex_point2])
                line_colors.append("blue")
            collection = matplotlib.collections.LineCollection(lines, color=line_colors)
            axes[0].add_collection(collection)
        axes[0].add_patch(matplotlib.patches.Circle(xy=(robot_pose.position.x, robot_pose.position.y), radius=0.1, fill=False, color='b'))
        axes[0].plot([robot_pose.position.x, robot_pose.position.x+0.1*math.cos(robot_yaw)], [robot_pose.position.y, robot_pose.position.y+0.1*math.sin(robot_yaw)], color='b')

        extent = map_origin_x, map_origin_x + map_width*map_resolution, map_origin_y, map_origin_y + map_height*map_resolution
        axes[1].imshow(np.flipud(vis_free_contours_outside_polyhedron_image), extent=extent)

        plt.show()


def test_calc_contours_centroids(map_name, lethal_cost_threshold=65, skip_plot=False):
    test_costmap = costmap_utils.load_costmap_file(map_name)

    robot_pose, robot_yaw = test_utils.get_random_robot_pose(test_costmap, lethal_cost_threshold)
    print("robot_pose : " + str(robot_pose))

    polyhedron_sensor_range = 3.0
    num_sample_coverage_polyhedron_point = 36
    ignore_small_area_square_meters = 1.0
    angles_sample_coverage_polyhedron = [i*(math.pi*2.0)/float(num_sample_coverage_polyhedron_point) for i in range(num_sample_coverage_polyhedron_point)]
    angles_sample_coverage_polyhedron = [angle if angle<=math.pi else -(math.pi*2.0-angle) for angle in angles_sample_coverage_polyhedron]
    polyhedron_buffer_length = 0.2
    polyhedron_buffer_samples = 12

    grid_data_image = np.array(test_costmap.occupancy_grid_data, dtype=np.int8).reshape(test_costmap.height, test_costmap.width)
    binary_free_image = costmap_utils.calc_binary_free_image(grid_data_image, lethal_cost_threshold)

    coverage_polyhedron = geometry_utils.calc_coverage_polyhedron(test_costmap, lethal_cost_threshold, robot_pose, polyhedron_sensor_range, angles_sample_coverage_polyhedron,
                                                                polyhedron_buffer_length=polyhedron_buffer_length, polyhedron_buffer_samples=polyhedron_buffer_samples)
    binary_coverage_polyhedron_image = costmap_utils.calc_binary_polyhedron_image(test_costmap.costmap_info, coverage_polyhedron)
    binary_coverage_polyhedron_map = costmap_utils.Costmap(test_costmap.costmap_info, tuple(binary_coverage_polyhedron_image.flatten().astype(np.uint8)))

    free_contours_outside_polyhedron, free_contours_outside_polyhedron_id_map = geometry_utils.calc_contours_outside_coverage_polyhedron(binary_coverage_polyhedron_map, binary_free_image,
                                                                                                                                        ignore_small_area_square_meters=ignore_small_area_square_meters)
    free_contours_outside_polyhedron_centroids = geometry_utils.calc_contours_centroids(test_costmap.costmap_info, free_contours_outside_polyhedron)
    print("finish calc_contours_centroids, free_contours_outside_polyhedron_centroids = " + str(free_contours_outside_polyhedron_centroids))

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

        free_contours_outside_polyhedron_image = np.array(free_contours_outside_polyhedron_id_map.occupancy_grid_data, dtype=np.int8).reshape(free_contours_outside_polyhedron_id_map.height, free_contours_outside_polyhedron_id_map.width)
        vis_free_contours_outside_polyhedron_image = np.zeros(free_contours_outside_polyhedron_image.shape + (3,), dtype=np.uint8)
        free_contours_outside_polyhedron_ids = np.unique(free_contours_outside_polyhedron_image)
        for free_contours_outside_polyhedron_id in free_contours_outside_polyhedron_ids:
            if free_contours_outside_polyhedron_id>0:
                vis_free_contours_outside_polyhedron_image[free_contours_outside_polyhedron_image==free_contours_outside_polyhedron_id] = [np.random.randint(0, 256), np.random.randint(0, 256), np.random.randint(0, 256)]

                free_contours_outside_polyhedron_index = free_contours_outside_polyhedron_id - 1
                free_contours_outside_polyhedron_centroid = free_contours_outside_polyhedron_centroids[free_contours_outside_polyhedron_index]
                centroid_map_x, centroid_map_y = costmap_utils.world_to_map(test_costmap.costmap_info, free_contours_outside_polyhedron_centroid.x, free_contours_outside_polyhedron_centroid.y)
                cv2.circle(vis_free_contours_outside_polyhedron_image, (centroid_map_x, centroid_map_y), radius=2, color=(255, 0, 0), thickness=cv2.FILLED)

        extent = map_origin_x, map_origin_x + map_width*map_resolution, map_origin_y, map_origin_y + map_height*map_resolution

        x, y = np.mgrid[slice(map_origin_x, map_origin_x + map_width*map_resolution, map_resolution),
                        slice(map_origin_y, map_origin_y + map_height*map_resolution, map_resolution)]
        fig, axes=plt.subplots(1,2)
        axes[0].imshow(np.flipud(vis_grid_data), extent=extent)

        axes[0].scatter([point[0] for point in coverage_polyhedron.points[coverage_polyhedron.vertices]], [point[1] for point in coverage_polyhedron.points[coverage_polyhedron.vertices]], s=5, marker='x', color='b')
        lines = []
        line_colors = []
        if coverage_polyhedron is not None:
            for simplex in coverage_polyhedron.simplices:
                assert(len(simplex)==2)
                simplex_point1 = [coverage_polyhedron.points[simplex[0], 0], coverage_polyhedron.points[simplex[0], 1]]
                simplex_point2 = [coverage_polyhedron.points[simplex[1], 0], coverage_polyhedron.points[simplex[1], 1]]
                lines.append([simplex_point1, simplex_point2])
                line_colors.append("blue")
            collection = matplotlib.collections.LineCollection(lines, color=line_colors)
            axes[0].add_collection(collection)
        axes[0].add_patch(matplotlib.patches.Circle(xy=(robot_pose.position.x, robot_pose.position.y), radius=0.1, fill=False, color='b'))
        axes[0].plot([robot_pose.position.x, robot_pose.position.x+0.1*math.cos(robot_yaw)], [robot_pose.position.y, robot_pose.position.y+0.1*math.sin(robot_yaw)], color='b')

        extent = map_origin_x, map_origin_x + map_width*map_resolution, map_origin_y, map_origin_y + map_height*map_resolution
        axes[1].imshow(np.flipud(vis_free_contours_outside_polyhedron_image), extent=extent)

        plt.show()


def test_calc_contours_attributes(map_name, lethal_cost_threshold=65, skip_plot=False):
    test_costmap = costmap_utils.load_costmap_file(map_name)

    robot_pose, robot_yaw = test_utils.get_random_robot_pose(test_costmap, lethal_cost_threshold)
    print("robot_pose : " + str(robot_pose))

    polyhedron_sensor_range = 3.0
    num_sample_coverage_polyhedron_point = 36
    ignore_small_area_square_meters = 1.0
    angles_sample_coverage_polyhedron = [i*(math.pi*2.0)/float(num_sample_coverage_polyhedron_point) for i in range(num_sample_coverage_polyhedron_point)]
    angles_sample_coverage_polyhedron = [angle if angle<=math.pi else -(math.pi*2.0-angle) for angle in angles_sample_coverage_polyhedron]
    polyhedron_buffer_length = 0.2
    polyhedron_buffer_samples = 12

    grid_data_image = np.array(test_costmap.occupancy_grid_data, dtype=np.int8).reshape(test_costmap.height, test_costmap.width)
    binary_free_image = costmap_utils.calc_binary_free_image(grid_data_image, lethal_cost_threshold)

    coverage_polyhedron = geometry_utils.calc_coverage_polyhedron(test_costmap, lethal_cost_threshold, robot_pose, polyhedron_sensor_range, angles_sample_coverage_polyhedron,
                                                                polyhedron_buffer_length=polyhedron_buffer_length, polyhedron_buffer_samples=polyhedron_buffer_samples)
    binary_coverage_polyhedron_image = costmap_utils.calc_binary_polyhedron_image(test_costmap.costmap_info, coverage_polyhedron)
    binary_coverage_polyhedron_map = costmap_utils.Costmap(test_costmap.costmap_info, tuple(binary_coverage_polyhedron_image.flatten().astype(np.uint8)))

    free_contours_outside_polyhedron, free_contours_outside_polyhedron_id_map = geometry_utils.calc_contours_outside_coverage_polyhedron(binary_coverage_polyhedron_map, binary_free_image,
                                                                                                                                        ignore_small_area_square_meters=ignore_small_area_square_meters)
    free_contours_outside_polyhedron_attributes = geometry_utils.calc_contours_attributes(test_costmap.costmap_info, free_contours_outside_polyhedron)
    print("finish calc_contours_attributes, free_contours_outside_polyhedron_attributes = " + str(free_contours_outside_polyhedron_attributes))

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

        free_contours_outside_polyhedron_image = np.array(free_contours_outside_polyhedron_id_map.occupancy_grid_data, dtype=np.int8).reshape(free_contours_outside_polyhedron_id_map.height, free_contours_outside_polyhedron_id_map.width)
        vis_free_contours_outside_polyhedron_image = np.zeros(free_contours_outside_polyhedron_image.shape + (3,), dtype=np.uint8)
        free_contours_outside_polyhedron_ids = np.unique(free_contours_outside_polyhedron_image)
        for free_contours_outside_polyhedron_id in free_contours_outside_polyhedron_ids:
            if free_contours_outside_polyhedron_id>0:
                vis_free_contours_outside_polyhedron_image[free_contours_outside_polyhedron_image==free_contours_outside_polyhedron_id] = [np.random.randint(0, 256), np.random.randint(0, 256), np.random.randint(0, 256)]

                free_contours_outside_polyhedron_index = free_contours_outside_polyhedron_id - 1
                mean = free_contours_outside_polyhedron_attributes[free_contours_outside_polyhedron_index]["mean"]
                mean_map_x, mean_map_y = costmap_utils.world_to_map(test_costmap.costmap_info, mean[0], mean[1])
                cv2.circle(vis_free_contours_outside_polyhedron_image, (mean_map_x, mean_map_y), radius=2, color=(255, 0, 0), thickness=cv2.FILLED)

                length_angle = free_contours_outside_polyhedron_attributes[free_contours_outside_polyhedron_index]["length_angle"]
                length = free_contours_outside_polyhedron_attributes[free_contours_outside_polyhedron_index]["length"]
                length_pixels = length / test_costmap.resolution
                cv2.line(vis_free_contours_outside_polyhedron_image, (mean_map_x-int(length_pixels*math.cos(length_angle)/2.0), mean_map_y-int(length_pixels*math.sin(length_angle)/2.0)), 
                    (mean_map_x+int(length_pixels*math.cos(length_angle)/2.0), mean_map_y+int(length_pixels*math.sin(length_angle)/2.0)), color=(0, 0, 255), thickness=1)

                width_angle = free_contours_outside_polyhedron_attributes[free_contours_outside_polyhedron_index]["width_angle"]
                width = free_contours_outside_polyhedron_attributes[free_contours_outside_polyhedron_index]["width"]
                width_pixels = width / test_costmap.resolution
                cv2.line(vis_free_contours_outside_polyhedron_image, (mean_map_x-int(width_pixels*math.cos(width_angle)/2.0), mean_map_y-int(width_pixels*math.sin(width_angle)/2.0)), 
                    (mean_map_x+int(width_pixels*math.cos(width_angle)/2.0), mean_map_y+int(width_pixels*math.sin(width_angle)/2.0)), color=(0, 0, 255), thickness=1)

        extent = map_origin_x, map_origin_x + map_width*map_resolution, map_origin_y, map_origin_y + map_height*map_resolution

        x, y = np.mgrid[slice(map_origin_x, map_origin_x + map_width*map_resolution, map_resolution),
                        slice(map_origin_y, map_origin_y + map_height*map_resolution, map_resolution)]
        fig, axes=plt.subplots(1,2)
        axes[0].imshow(np.flipud(vis_grid_data), extent=extent)

        axes[0].scatter([point[0] for point in coverage_polyhedron.points[coverage_polyhedron.vertices]], [point[1] for point in coverage_polyhedron.points[coverage_polyhedron.vertices]], s=5, marker='x', color='b')
        lines = []
        line_colors = []
        if coverage_polyhedron is not None:
            for simplex in coverage_polyhedron.simplices:
                assert(len(simplex)==2)
                simplex_point1 = [coverage_polyhedron.points[simplex[0], 0], coverage_polyhedron.points[simplex[0], 1]]
                simplex_point2 = [coverage_polyhedron.points[simplex[1], 0], coverage_polyhedron.points[simplex[1], 1]]
                lines.append([simplex_point1, simplex_point2])
                line_colors.append("blue")
            collection = matplotlib.collections.LineCollection(lines, color=line_colors)
            axes[0].add_collection(collection)
        axes[0].add_patch(matplotlib.patches.Circle(xy=(robot_pose.position.x, robot_pose.position.y), radius=0.1, fill=False, color='b'))
        axes[0].plot([robot_pose.position.x, robot_pose.position.x+0.1*math.cos(robot_yaw)], [robot_pose.position.y, robot_pose.position.y+0.1*math.sin(robot_yaw)], color='b')

        extent = map_origin_x, map_origin_x + map_width*map_resolution, map_origin_y, map_origin_y + map_height*map_resolution
        axes[1].imshow(np.flipud(vis_free_contours_outside_polyhedron_image), extent=extent)

        plt.show()


def test_sample_visible_free_points_by_angles(map_name, lethal_cost_threshold=65, skip_plot=False):
    test_costmap = costmap_utils.load_costmap_file(map_name)

    robot_pose, robot_yaw = test_utils.get_random_robot_pose(test_costmap, lethal_cost_threshold)
    print("robot_pose : " + str(robot_pose))

    sample_range = 5.0
    num_sample_visible_contour_point = 72
    angles_sample_visible_contour = [i*(math.pi*2.0)/float(num_sample_visible_contour_point) for i in range(num_sample_visible_contour_point)]
    angles_sample_visible_contour = [angle if angle<=math.pi else -(math.pi*2.0-angle) for angle in angles_sample_visible_contour]
    num_sample_free_point = 36
    angles_sample_free_point = [i*(math.pi*2.0)/float(num_sample_free_point) for i in range(num_sample_free_point)]
    angles_sample_free_point = [angle if angle<=math.pi else -(math.pi*2.0-angle) for angle in angles_sample_free_point]
    min_distance = 0.35
    unknown_area_margin = 0.05
    lethal_area_margin = 0.35

    visible_area_contour = geometry_utils.calc_visible_area_contour(test_costmap, lethal_cost_threshold, robot_pose, sample_range, angles_sample_visible_contour)
    binary_visible_area_contour_image = costmap_utils.calc_binary_contours_image(test_costmap.costmap_info, [visible_area_contour])
    binary_visible_area_map = costmap_utils.Costmap(test_costmap.costmap_info, tuple(binary_visible_area_contour_image.flatten().astype(np.uint8)))
    free_points_list = geometry_utils.sample_visible_free_points_by_angles(test_costmap, lethal_cost_threshold, binary_visible_area_map, robot_pose, sample_range, angles_sample_free_point,
                                                                        min_distance=min_distance, unknown_area_margin=unknown_area_margin, lethal_area_margin=lethal_area_margin)
    print("finish sample_visible_free_points_by_angles, free_points_list = " + str(free_points_list))

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

        extent = map_origin_x, map_origin_x + map_width*map_resolution, map_origin_y, map_origin_y + map_height*map_resolution

        x, y = np.mgrid[slice(map_origin_x, map_origin_x + map_width*map_resolution, map_resolution),
                        slice(map_origin_y, map_origin_y + map_height*map_resolution, map_resolution)]
        fig,axes=plt.subplots(1,1)
        axes.imshow(np.flipud(vis_grid_data), extent=extent)
        axes.scatter([point.x for point in free_points_list], [point.y for point in free_points_list], s=5, marker='x', color='r')
        axes.add_patch(matplotlib.patches.Circle(xy=(robot_pose.position.x, robot_pose.position.y), radius=0.1, fill=False, color='b'))
        axes.plot([robot_pose.position.x, robot_pose.position.x+0.1*math.cos(robot_yaw)], [robot_pose.position.y, robot_pose.position.y+0.1*math.sin(robot_yaw)], color='b')
        plt.show()


def test_select_forward_backward_farthest_point_idx(map_name, lethal_cost_threshold=65, point_list=None, skip_plot=False):
    test_costmap = costmap_utils.load_costmap_file(map_name)

    if point_list is None:
        point_list = []
        np_point_list = np.random.uniform(-3, 3, (100, 2))
        for np_point in np_point_list:
            point = Point(np_point[0], np_point[1], 0.0)
            if geometry_utils.check_is_point_free(test_costmap, lethal_cost_threshold, point):
                point_list.append(point)
    print("point_list : " + str(point_list))

    robot_pose, robot_yaw = test_utils.get_random_robot_pose(test_costmap, lethal_cost_threshold)
    print("robot_pose : " + str(robot_pose))

    max_abs_yaw_forward_point=math.pi/6.0
    min_abs_yaw_backward_point=math.pi*5.0/6.0

    forward_point_idx, backward_point_idx, _ = geometry_utils.select_forward_backward_farthest_point_idx(robot_pose, point_list,
                                                                                                        max_abs_yaw_forward_point=max_abs_yaw_forward_point,
                                                                                                        min_abs_yaw_backward_point=min_abs_yaw_backward_point)
    print("forward_point_idx : " + str(forward_point_idx) + ", backward_point_idx : " + str(backward_point_idx))
    forward_point = None
    if forward_point_idx is not None:
        forward_point = point_list[forward_point_idx]
    backward_point = None
    if backward_point_idx is not None:
        backward_point = point_list[backward_point_idx]
    print("forward_point : " + str(forward_point) + ", backward_point : " + str(backward_point))

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

        extent = map_origin_x, map_origin_x + map_width*map_resolution, map_origin_y, map_origin_y + map_height*map_resolution

        x, y = np.mgrid[slice(map_origin_x, map_origin_x + map_width*map_resolution, map_resolution),
                        slice(map_origin_y, map_origin_y + map_height*map_resolution, map_resolution)]
        fig,axes=plt.subplots(1,1)
        axes.imshow(np.flipud(vis_grid_data), extent=extent)
        axes.scatter([point.x for point in point_list], [point.y for point in point_list], s=5, marker='x', color='gray')
        if forward_point is not None:
            axes.scatter([forward_point.x], [forward_point.y], s=5, marker='x', color='green')
        if backward_point is not None:
            axes.scatter([backward_point.x], [backward_point.y], s=5, marker='x', color='orange')
        axes.add_patch(matplotlib.patches.Circle(xy=(robot_pose.position.x, robot_pose.position.y), radius=0.1, fill=False, color='b'))
        axes.plot([robot_pose.position.x, robot_pose.position.x+0.1*math.cos(robot_yaw)], [robot_pose.position.y, robot_pose.position.y+0.1*math.sin(robot_yaw)], color='b')
        plt.show()


def test_select_farthest_point_idx_with_forward_priority(map_name, lethal_cost_threshold=65, point_list=None, skip_plot=False):
    test_costmap = costmap_utils.load_costmap_file(map_name)

    if point_list is None:
        point_list = []
        np_point_list = np.random.uniform(-3, 3, (100, 2))
        for np_point in np_point_list:
            point = Point(np_point[0], np_point[1], 0.0)
            if geometry_utils.check_is_point_free(test_costmap, lethal_cost_threshold, point):
                point_list.append(point)
    print("point_list : " + str(point_list))

    robot_pose, robot_yaw = test_utils.get_random_robot_pose(test_costmap, lethal_cost_threshold)
    print("robot_pose : " + str(robot_pose))

    max_abs_yaw_forward_point=math.pi/4.0

    farthest_point_idx = geometry_utils.select_farthest_point_idx_with_forward_priority(robot_pose, point_list, max_abs_yaw_forward_point=max_abs_yaw_forward_point)
    print("farthest_point_idx : " + str(farthest_point_idx))
    farthest_point = None
    if farthest_point_idx is not None:
        farthest_point = point_list[farthest_point_idx]

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

        extent = map_origin_x, map_origin_x + map_width*map_resolution, map_origin_y, map_origin_y + map_height*map_resolution

        x, y = np.mgrid[slice(map_origin_x, map_origin_x + map_width*map_resolution, map_resolution),
                        slice(map_origin_y, map_origin_y + map_height*map_resolution, map_resolution)]
        fig,axes=plt.subplots(1,1)
        axes.imshow(np.flipud(vis_grid_data), extent=extent)
        axes.scatter([point.x for point in point_list], [point.y for point in point_list], s=5, marker='x', color='gray')
        if farthest_point is not None:
            axes.scatter([farthest_point.x], [farthest_point.y], s=5, marker='x', color='red')
        axes.add_patch(matplotlib.patches.Circle(xy=(robot_pose.position.x, robot_pose.position.y), radius=0.1, fill=False, color='b'))
        axes.plot([robot_pose.position.x, robot_pose.position.x+0.1*math.cos(robot_yaw)], [robot_pose.position.y, robot_pose.position.y+0.1*math.sin(robot_yaw)], color='b')
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='Test geomtry_utils.py')
    parser.add_argument('-s', '--skip-plot', action='store_true')
    args = parser.parse_args()
    print("skip plot : " + str(args.skip_plot))

    test_is_line_segments_intersect(skip_plot=args.skip_plot)
    test_is_line_segment_convex_hull_intersect(skip_plot=args.skip_plot)
    test_check_is_point_free("./test/testmap", skip_plot=args.skip_plot)
    test_check_is_in_line_of_sight("./test/testmap", skip_plot=args.skip_plot)
    test_calc_coverage_polyhedron("./test/testmap", skip_plot=args.skip_plot)
    test_is_point_in_area("./test/testmap", skip_plot=args.skip_plot)
    test_calc_visible_area_contour("./test/testmap", skip_plot=args.skip_plot)
    test_calc_contours_outside_coverage_polyhedron("./test/testmap", skip_plot=args.skip_plot)
    test_calc_contours_centroids("./test/testmap", skip_plot=args.skip_plot)
    test_calc_contours_attributes("./test/testmap", skip_plot=args.skip_plot)
    test_sample_visible_free_points_by_angles("./test/testmap", skip_plot=args.skip_plot)
    test_select_forward_backward_farthest_point_idx("./test/testmap", skip_plot=args.skip_plot)
    test_select_farthest_point_idx_with_forward_priority("./test/testmap", skip_plot=args.skip_plot)


if __name__=="__main__":
    main()
