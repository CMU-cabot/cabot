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

import itertools
import math
import sys

import cv2
from geometry_msgs.msg import Point, Pose
import numpy as np
import rospy
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from skimage import draw
import tf

from . import costmap_utils


def pose2transform(pose):
    """
    Convert Pose to transformation matrix

    Parameters
    ----------
    pose : geometry_msgs.msg.Pose
        The input pose

    Returns
    -------
    numpy.ndarray
        The output ransformation matrix
    """
    M = np.identity(4)
    M[:3,:3] = Rotation.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]).as_dcm()
    M[:3,3] = [pose.position.x, pose.position.y, pose.position.z]
    return M


def transform2pose(M):
    """
    Convert transformation matrix to Pose

    Parameters
    ----------
    M : numpy.ndarray
        The input transformation matrix

    Returns
    -------
    geometry_msgs.msg.Pose
        The output pose
    """
    pose = Pose()
    pose.position.x = M[0,3]
    pose.position.y = M[1,3]
    pose.position.z = M[2,3]
    quat = Rotation.from_dcm(M[:3,:3]).as_quat()
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose


def calc_point_distance(point1, point2):
    """
    Calculate distance betweeen two points.

    Parameters
    ----------
    point1 : geometry_msgs.msg.Point
        The first point.
    point2 : geometry_msgs.msg.Point
        The second point.

    Returns
    -------
    float
        Distance between input two points.
    """
    return math.sqrt(pow(point1.x-point2.x, 2.0) + pow(point1.y-point2.y, 2.0))


def calc_point_direction_yaw(point1, point2):
    """
    Calculate direction from point1 to point2.

    Parameters
    ----------
    point1 : geometry_msgs.msg.Point
        The first point.
    point2 : geometry_msgs.msg.Point
        The second point.

    Returns
    -------
    float
        Yaw direction from point1 to point2.
    """
    return math.atan2(point2.y-point1.y, point2.x-point1.x)


def calc_point_direction_quaternion(point1, point2):
    """
    Calculate direction from point1 to point2.

    Parameters
    ----------
    point1 : geometry_msgs.msg.Point
        The first point.
    point2 : geometry_msgs.msg.Point
        The second point.

    Returns
    -------
    geometry_msgs.Quaternion
        Direction from point1 to point2.
    """
    return tf.transformations.quaternion_from_euler(0.0, 0.0, calc_point_direction_yaw(point1, point2))


def calc_relative_orientation_quaternion(quat1, quat2):
    """
    Calculate relative orientation from quat1 to quat2.

    Parameters
    ----------
    quat1 : array-like
        The first pose orientation.
    quat2 : array-like
        The second pose orientation.

    Returns
    -------
    array-like
        Relative orientation from quat1 to quat2.
    """
    assert len(quat1)==4 and len(quat2)==4

    quat1_inv = (quat1[0], quat1[1], quat1[2], -quat1[3])
    return tf.transformations.quaternion_multiply(quat2, quat1_inv)


def get_perpendicular_point_to_line(point, line_start, line_end):
    """
    find the perpendicular point to line segment from point

    Parameters
    ----------
    point: numpy.ndarray
        The input point
    line_start: numpy.ndarray
        The start point of input line segment
    line_end: numpy.ndarray
        The end point of input line segment

    Returns
    -------
    numpy.ndarray
        found perpendicular point to line segment
    """
    vec_start_to_point = point - line_start
    vec_start_to_end = line_end - line_start
    dot_product = np.dot(vec_start_to_point, vec_start_to_end)
    norm_vec_start_to_end = np.linalg.norm(vec_start_to_end)
    projection = dot_product/norm_vec_start_to_end
    start_cross_vector = vec_start_to_end * (projection/norm_vec_start_to_end)
    return np.array([line_start[0]+start_cross_vector[0], line_start[1]+start_cross_vector[1]])


def project_point_on_robot_moving_path(robot_pose, point):
    """
    calculate the position when projecting input point on the robot moving path

    Parameters
    ----------
    robot_pose : geometry_msgs.msg.Pose
        The robot pose
    point : geometry_msgs.msg.Point
        The input point.

    Returns
    -------
    geometry_msgs.msg.Point
        The point which is projected on the robot moving path
    """
    robot_pose_position = robot_pose.position
    robot_pose_quat = (robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w)

    np_point = np.array([point.x, point.y])
    robot_pose_yaw = tf.transformations.euler_from_quaternion(robot_pose_quat)[2]
    np_robot_position = np.array([robot_pose_position.x, robot_pose_position.y])
    np_robot_forward_position = np.array([np_robot_position[0] + math.cos(robot_pose_yaw), np_robot_position[1] + math.sin(robot_pose_yaw)])
    np_project_point = get_perpendicular_point_to_line(np_point, np_robot_position, np_robot_forward_position)
    return Point(np_project_point[0], np_project_point[1], 0.0)


def check_is_point_free(costmap, lethal_cost_threshold, point):
    """
    Check if input point is in free space.

    Parameters
    ----------
    costmap : costmap_utils.Costmap
        The cost map
    lethal_cost_threshold : int
        If the cost map value is equal or larther than this value, that point is considered as lethal area
    point : geometry_msgs.msg.Point
        The input point.

    Returns
    -------
    bool
        Return True if input point is in free space, and False otherwise.
    """
    costmap_x, costmap_y = costmap_utils.world_to_map(costmap.costmap_info, point.x, point.y)

    try:
        cost = costmap.get_cost(costmap_x, costmap_y)
        if cost>=0 and cost<lethal_cost_threshold:
            return True
    except IndexError as e:
        rospy.logerr("skip check point, index error, costmap_y = " + str(costmap_y) + ", costmap_x = " + str(costmap_x))

    return False


def is_line_segments_intersect(point1, point2, point3, point4):
    """
    Check if input two line segments intersects
    Reference :
        https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line_segment
        Antonio, Franklin (1992). "Chapter IV.6: Faster Line Segment Intersection". In Kirk, David (ed.). Graphics Gems III. Academic Press, Inc. pp. 199–202.

    Parameters
    ----------
    point1 : array-like
        The first point of the first line.
    point2 : array-like
        The second point of the first line.
    point3 : array-like
        The first point of the second line.
    point4 : array-like
        The second point of the second line.

    Returns
    -------
    bool
        True if input two line segments intersects, otherwise False.
    """
    def is_solution_in_range(numerator, denominator):
        if denominator>0:
            if numerator<0 or numerator>denominator:
                return False
            else:
                return True
        elif denominator<0:
            if numerator>0 or numerator<denominator:
                return False
            else:
                return True
        else:
            return False

    Ax = point2[0] - point1[0]
    Ay = point2[1] - point1[1]
    Bx = point3[0] - point4[0]
    By = point3[1] - point4[1]
    Cx = point1[0] - point3[0]
    Cy = point1[1] - point3[1]

    alpha_numerator = By*Cx - Bx*Cy
    alpha_denominator = Ay*Bx - Ax*By

    # check if alpha will be in the range [0,1] without devision
    if is_solution_in_range(alpha_numerator, alpha_denominator):
        beta_numerator = Ax*Cy - Ay*Cx
        beta_denominator = Ay*Bx - Ax*By
        # check if beta will be in the range [0,1] without devision
        if is_solution_in_range(beta_numerator, beta_denominator):
            return True
        else:
            return False
    else:
        return False


def is_line_segment_convex_hull_intersect(point1, point2, convex_hell):
    """
    Check if input line segment intersects with convex hull

    Parameters
    ----------
    point1 : array-like
        The first point of the input line.
    point2 : array-like
        The second point of the input line.
    convex_hell : scipy.spatial.ConvexHull
        The input convex hull.

    Returns
    -------
    bool
        True if input line segment intersects with convex hull, otherwise False.
    """
    for simplex_idx, simplex in enumerate(convex_hell.simplices):
        assert(len(simplex)==2)

        simplex_point1 = [convex_hell.points[simplex[0], 0], convex_hell.points[simplex[0], 1]]
        simplex_point2 = [convex_hell.points[simplex[1], 0], convex_hell.points[simplex[1], 1]]
        if is_line_segments_intersect(point1, point2, simplex_point1, simplex_point2):
            return True
    return False


def check_is_in_line_of_sight(costmap_info, binary_non_free_image, point1, point2):
    """
    Check if two input points are in line of sight each other.

    Parameters
    ----------
    costmap_info : costmap_utils.CostmapInfo
        The cost map info
    binary_non_free_image : numpy.ndarray
        The binary non free area image for the input cost map
    point1 : geometry_msgs.msg.Point
        The first point.
    point2 : geometry_msgs.msg.Point
        The second point.

    Returns
    -------
    bool
        Return True if two input points are visible to each other, and False otherwise.
    """
    costmap_x1, costmap_y1 = costmap_utils.world_to_map(costmap_info, point1.x, point1.y)
    costmap_x2, costmap_y2 = costmap_utils.world_to_map(costmap_info, point2.x, point2.y)

    binary_line_of_sight_image = np.zeros(binary_non_free_image.shape, np.uint8)
    cv2.line(binary_line_of_sight_image, (costmap_x1, costmap_y1), (costmap_x2, costmap_y2), color=1, thickness=1)

    ## debug visualization
    # cv2.imshow("binary line of sight image", binary_line_of_sight_image*255)
    # cv2.waitKey(1)

    binary_line_of_sight_image_lethal_image = cv2.bitwise_and(binary_line_of_sight_image, binary_non_free_image)

    ## debug visualization
    # cv2.imshow("binary line of sight lethal image", binary_line_of_sight_image_lethal_image*255)
    # cv2.waitKey(1)

    if np.any(binary_line_of_sight_image_lethal_image):
        return False
    else:
        return True


def is_point_in_area(binary_area_map, point):
    """
    Check if input point is in input area

    Parameters
    ----------
    binary_area_map : costmap_utils.Costmap
        The binary area map
    point : geometry_msgs.msg.Point
        The input point.

    Returns
    -------
    bool
        Return True if input point is in input area, and False otherwise.
    """
    point_map_x, point_map_y = costmap_utils.world_to_map(binary_area_map.costmap_info, point.x, point.y)
    if binary_area_map.get_cost(point_map_x, point_map_y)>0:
        return True
    else:
        return False


def select_points_in_area(binary_area_map, points):
    """
    Select points which are in input area

    Parameters
    ----------
    binary_area_map : costmap_utils.Costmap
        The binary area map
    points : array-like
        The input points

    Returns
    -------
    array-like
        The list of indices of points which are in input area
    """
    if len(points)==0:
        return []

    selected_points_indices = []
    for index, point in enumerate(points):
        point_map_x, point_map_y = costmap_utils.world_to_map(binary_area_map.costmap_info, point.x, point.y)
        if binary_area_map.get_cost(point_map_x, point_map_y)>0:
            selected_points_indices.append(index)
    return selected_points_indices


def _is_free_in_angle(costmap, lethal_cost_threshold, robot_position, sensor_range, check_angle, return_non_free_point=False):
    """
    Check if input angle is free from the input robot pose.

    Parameters
    ----------
    costmap : costmap_utils.Costmap
        The cost map
    lethal_cost_threshold : int
        If the cost map value is equal or larther than this value, that point is considered as lethal area
    robot_position : geometry_msgs.msg.Point
        The robot position
    sensor_range : float
        The valid range of sensor
    check_angle : float
        The angle to check if the direction from robot is free
    return_non_free_point : bool
        If True, return the closest non free point for input angle. Otherwise, return the farthest free point for input angle.

    Returns
    -------
    bool
        Return True if input angle is free from robot by using a sensor, and False otherwise.
    geometry_msgs.msg.Point
        Return the farthest free point or the closest non free point for input angle.
    float
        Cost value of obstacle or unknown area position if input angle is not free, and farthest point cost value otherwise.
    """
    sample_pose_x = robot_position.x + sensor_range*math.cos(check_angle)
    sample_pose_y = robot_position.y + sensor_range*math.sin(check_angle)

    costmap_x1, costmap_y1 = costmap_utils.world_to_map(costmap.costmap_info, robot_position.x, robot_position.y)
    costmap_x2, costmap_y2 = costmap_utils.world_to_map(costmap.costmap_info, sample_pose_x, sample_pose_y)

    rows, cols = draw.line(costmap_y1, costmap_x1, costmap_y2, costmap_x2)
    assert len(rows)==len(cols)

    cost = None
    indices = range(len(rows))[1:]
    for idx in indices:
        row = rows[idx]
        col = cols[idx]
        try:
            cost = costmap.get_cost(col, row)
            if cost<0 or cost>=lethal_cost_threshold:
                if return_non_free_point:
                    non_free_world_x, non_free_world_y = costmap_utils.map_to_world(costmap.costmap_info, cols[idx], rows[idx])
                    return False, Point(non_free_world_x, non_free_world_y, 0.0), cost
                else:
                    free_world_x, free_world_y = costmap_utils.map_to_world(costmap.costmap_info, cols[idx-1], rows[idx-1])
                    return False, Point(free_world_x, free_world_y, 0.0), cost
        except IndexError as e:
            rospy.logerr("skip check point, index error, row = " + str(row) + ", col = " + str(col))

    if return_non_free_point:
        return True, None, cost
    else:
        free_world_x, free_world_y = costmap_utils.map_to_world(costmap.costmap_info, cols[-1], rows[-1])
        return True, Point(free_world_x, free_world_y, 0.0), cost


def sample_visible_free_points_by_angles(costmap, lethal_cost_threshold, binary_visible_area_map, robot_pose, sample_range, angles_sample_point, min_distance=1.0, unknown_area_margin=1.0, lethal_area_margin=1.0):
    """
    Sample free points which are visible from robot by input angles

    Parameters
    ----------
    costmap : costmap_utils.Costmap
        The cost map
    lethal_cost_threshold : int
        If the cost map value is equal or larther than this value, that point is considered as lethal area
    binary_visible_area_map : costmap_utils.Costmap
        The binary visible area map
    robot_pose : geometry_msgs.msg.Pose
        The robot pose
    sample_range : float
        The valid range to sample points
    angles_sample_point : array-like
        The list of angles to sample points
    min_distance: float
        The minimum distance betweeen a sample point and robot position
    unknown_area_margin: float
        The margin between a sample point and unknown area
    lethal_area_margin: float
        The margin between a sample point and lethal area

    Returns
    -------
    array-like
        The list of sample points which are visible from robot
    """
    assert sample_range>min_distance+unknown_area_margin
    assert min_distance>=unknown_area_margin
    assert sample_range>min_distance+lethal_area_margin
    assert min_distance>=lethal_area_margin
    assert len(angles_sample_point)>0

    robot_pose_position = robot_pose.position
    robot_pose_quat = (robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w)
    robot_pose_yaw = tf.transformations.euler_from_quaternion(robot_pose_quat)[2]

    # sample movable points by direction
    sample_points = []
    for sample_angle in angles_sample_point:
        angle_sample_yaw = robot_pose_yaw + sample_angle
        is_free, free_point, none_free_cost = _is_free_in_angle(costmap, lethal_cost_threshold, robot_pose_position, sample_range, angle_sample_yaw, return_non_free_point=False)
        if calc_point_distance(robot_pose.position, free_point)>min_distance:
            if is_free:
                sample_point = Point(free_point.x, free_point.y, 0)
            else:
                # return point with margin if point is not free
                if none_free_cost<0:
                    sample_point = Point(free_point.x-unknown_area_margin*math.cos(angle_sample_yaw), free_point.y-unknown_area_margin*math.sin(angle_sample_yaw), 0)
                else:
                    sample_point = Point(free_point.x-lethal_area_margin*math.cos(angle_sample_yaw), free_point.y-lethal_area_margin*math.sin(angle_sample_yaw), 0)

            sample_point_map_x, sample_point_map_y = costmap_utils.world_to_map(costmap.costmap_info, sample_point.x, sample_point.y)
            if binary_visible_area_map.get_cost(sample_point_map_x, sample_point_map_y)>0:
                sample_points.append(sample_point)
    return sample_points


def calc_visible_area_contour(costmap, lethal_cost_threshold, robot_pose, sample_range, angles_sample_coverage_polyhedron):
    """
    Calculate contour points of visible area by samping points

    Parameters
    ----------
    costmap : costmap_utils.Costmap
        The cost map
    lethal_cost_threshold : int
        If the cost map value is equal or larther than this value, that point is considered as lethal area
    robot_pose : geometry_msgs.msg.Pose
        The robot pose
    sample_range : float
        The valid range to sample points
    angles_sample_coverage_polyhedron : array-like
        The list of sample angles to check if obstacle exists from robot

    Returns
    -------
    numpy.ndarray
        contour points of visible area
    """
    assert len(angles_sample_coverage_polyhedron)>0

    robot_pose_position = robot_pose.position
    robot_pose_quat = (robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w)
    robot_pose_yaw = tf.transformations.euler_from_quaternion(robot_pose_quat)[2]

    sample_points = []
    for sample_angle in angles_sample_coverage_polyhedron:
        angle_sample_yaw = robot_pose_yaw + sample_angle
        _, free_point, _ = _is_free_in_angle(costmap, lethal_cost_threshold, robot_pose_position, sample_range, angle_sample_yaw, return_non_free_point=False)
        sample_points.append([free_point.x, free_point.y])

    return np.array(sample_points)


def calc_coverage_polyhedron(costmap, lethal_cost_threshold, robot_pose, polyhedron_sensor_range, angles_sample_coverage_polyhedron, polyhedron_buffer_length=0.0, polyhedron_buffer_samples=12):
    """
    Calculate coverage polyhedron to cluster local frontier points
    (See Fan et.al., ICRA 2021, https://arxiv.org/abs/2103.16829)

    Parameters
    ----------
    costmap : costmap_utils.Costmap
        The cost map
    lethal_cost_threshold : int
        If the cost map value is equal or larther than this value, that point is considered as lethal area
    robot_pose : geometry_msgs.msg.Pose
        The robot pose
    polyhedron_sensor_range : float
        The valid range of sensor to calculate polyhedron
    angles_sample_coverage_polyhedron : array-like
        The list of sample angles to check if obstacle exists from robot
    polyhedron_buffer_length : float
        The length to expand coverage polyhedron
    polyhedron_buffer_samples : float
        The number of samples to expand coverage polyhedron

    Returns
    -------
    scipy.spatial.ConvexHull
        The coverage polyhedron for input robot pose if enough obstacle points are found, and None otherwise.
    """
    assert len(angles_sample_coverage_polyhedron)>0

    robot_pose_position = robot_pose.position
    robot_pose_quat = (robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w)
    robot_pose_yaw = tf.transformations.euler_from_quaternion(robot_pose_quat)[2]

    convex_points = []
    for sample_angle in angles_sample_coverage_polyhedron:
        angle_sample_yaw = robot_pose_yaw + sample_angle
        is_free, non_free_point, none_free_cost = _is_free_in_angle(costmap, lethal_cost_threshold, robot_pose_position, polyhedron_sensor_range, angle_sample_yaw, return_non_free_point=True)
        if not is_free and none_free_cost>=lethal_cost_threshold:
            convex_points.append([non_free_point.x, non_free_point.y])

    if len(convex_points)>=3:
        try:
            convex_points = np.array(convex_points)
            hull_polyhedron = ConvexHull(convex_points)
            if polyhedron_buffer_length==0:
                return hull_polyhedron
            else:
                angles_sample_buffered_points = [i*(math.pi*2.0)/float(polyhedron_buffer_samples) for i in range(polyhedron_buffer_samples)]

                buffered_convex_points = []
                vertices_points = convex_points[hull_polyhedron.vertices]
                for point in vertices_points:
                    buffered_convex_points.extend([[point[0]+polyhedron_buffer_length*math.cos(angle), point[1]+polyhedron_buffer_length*math.sin(angle)] for angle in angles_sample_buffered_points])
                buffered_convex_points = np.array(buffered_convex_points)
                return ConvexHull(buffered_convex_points)
        except Exception as e:
            print("Cannot create convex hull")
            return None
    else:
        return None


def calc_dt_coverage_polyhedron_range(route_dt_map, route_node_point_kd_tree, route_node_points, robot_pose, closest_route_node_range, dt_polyhrdron_range_eps):
    """
    Calculate coverage polyhedron range dynamically by using route distance transform map and route node points.
    First, select the closest node from robot, and get distance transform value for the node,
    and find close route nodes withing the distance transform value from the closest route node.
    Then, calculate coverarge polyhedron range dynamically by adding distance from robot to found nodes, distance transform value, and constant epsilon value,
    and return the largest coverarge polyhedron.

    Parameters
    ----------
    route_dt_map : costmap_utils.Costmap
        The route distance transform map
    route_node_point_kd_tree : scipy.spatial.cKDTree
        KDTree created by route node points
    route_node_points : numpy.ndarray
        route node points which is used to create route_node_point_kd_tree
    robot_pose : geometry_msgs.msg.Pose
        The robot pose
    closest_route_node_range : float
        The range to select the closest route node
    dt_polyhrdron_range_eps : float
        The constant value to add when calculating distance transform coverage polyhedron

    Returns
    -------
    float
        The coverage polyhedron range which is dynamically calculated by route distance transform map and route node points
    """
    # find the closest route node and get distance transform value for the node
    _, closest_route_node_point_index = route_node_point_kd_tree.query(np.array([robot_pose.position.x, robot_pose.position.y]), k=1)
    closest_route_node_point = Point(route_node_points[closest_route_node_point_index][0], route_node_points[closest_route_node_point_index][1], 0.0)
    closest_route_node_point_distance = calc_point_distance(robot_pose.position, closest_route_node_point)
    if closest_route_node_point_distance>closest_route_node_range:
        return None

    closest_route_node_point_map_x, closest_route_node_point_map_y = costmap_utils.world_to_map(route_dt_map.costmap_info, closest_route_node_point.x, closest_route_node_point.y)
    closest_route_node_point_dt = route_dt_map.get_cost(closest_route_node_point_map_x, closest_route_node_point_map_y)
    closest_route_node_point_dt_meters = closest_route_node_point_dt * route_dt_map.resolution

    # find close route nodes withing the distance transform value for the closest route nodes,
    # then find the largest distance closest route node and get distance transform value for that node
    dt_polyhedron_sensor_range = None
    close_route_node_point_indices = route_node_point_kd_tree.query_ball_point(route_node_points[closest_route_node_point_index], closest_route_node_point_dt_meters)
    if len(close_route_node_point_indices)==0:
        dt_polyhedron_sensor_range = closest_route_node_point_distance + closest_route_node_point_dt_meters + dt_polyhrdron_range_eps
    else:
        largest_dt_polyhedron_sensor_range = sys.float_info.min

        for close_route_node_point_index in close_route_node_point_indices:
            close_route_node_point = Point(route_node_points[close_route_node_point_index][0], route_node_points[close_route_node_point_index][1], 0.0)
            close_route_node_map_x, close_route_node_map_y = costmap_utils.world_to_map(route_dt_map.costmap_info, close_route_node_point.x, close_route_node_point.y)
            try:
                close_route_node_point_distance = calc_point_distance(robot_pose.position, close_route_node_point)

                close_route_node_point_dt = route_dt_map.get_cost(close_route_node_map_x, close_route_node_map_y)
                close_route_node_point_dt_meters = close_route_node_point_dt * route_dt_map.resolution

                dt_polyhedron_sensor_range = close_route_node_point_distance + close_route_node_point_dt_meters + dt_polyhrdron_range_eps
                if dt_polyhedron_sensor_range>largest_dt_polyhedron_sensor_range:
                    largest_dt_polyhedron_sensor_range = dt_polyhedron_sensor_range
            except IndexError as e:
                rospy.logerr("cannot get route closeness, close_route_node_map_x = " + str(close_route_node_map_x) + ", close_route_node_map_y = " + str(close_route_node_map_y))

        if largest_dt_polyhedron_sensor_range is not None:
            dt_polyhedron_sensor_range = largest_dt_polyhedron_sensor_range
    return dt_polyhedron_sensor_range


def cluster_point_by_coverage_polyhedron(costmap, lethal_cost_threshold, point_list, hull_polyhedron):
    """
    Cluster input points by considering visibility for each pair of point and coverage polyhedron
    (See Fan et.al., ICRA 2021, https://arxiv.org/abs/2103.16829)

    Parameters
    ----------
    costmap : costmap_utils.Costmap
        The cost map
    lethal_cost_threshold : int
        If the cost map value is equal or larther than this value, that point is considered as lethal area
    robot_pose : geometry_msgs.msg.Pose
        The robot pose
    point_list : array-like
        The list of points to cluster
    hull_polyhedron : scipy.spatial.ConvexHull
        The coverage polyhedron for robot pose.

    Returns
    -------
    clustered_point_list : array-like
        The list of clustered points
    """
    # create binary image for lethal area
    grid_data_image = np.array(costmap.occupancy_grid_data, dtype=np.int8).reshape(costmap.height, costmap.width)
    binary_non_free_image = costmap_utils.calc_binary_non_free_image(grid_data_image, lethal_cost_threshold)

    # start cluster points
    clustered_point_indices_list = []
    processed_point_indices_set = set()
    check_is_in_line_of_sight_indices_dict = {}
    if len(point_list)>1:
        for point_idx1 in range(len(point_list)):
            if point_idx1 not in processed_point_indices_set:
                point1_clustered_idx_set = set()
                point1_clustered_idx_set.add(point_idx1)

                clustered_point_indices = [point_idx1]
                processed_point_indices_set.add(point_idx1)

                if point_idx1<len(point_list)-1:
                    # if point1 is clustered with any point, recursively check all other unprocessed points if they will be clustered with point1 cluster
                    while True:
                        point1_cluster_updated = False

                        for point_idx2 in range(len(point_list))[point_idx1+1:]:
                            if point_idx2 not in processed_point_indices_set:
                                point2 = point_list[point_idx2]

                                for point1_clustered_idx in point1_clustered_idx_set:
                                    point1_clustered_point = point_list[point1_clustered_idx]

                                    # check if two points are in line of sight with cache
                                    if point1_clustered_idx not in check_is_in_line_of_sight_indices_dict:
                                        check_is_in_line_of_sight_indices_dict[point1_clustered_idx] = {}
                                    if point_idx2 not in check_is_in_line_of_sight_indices_dict[point1_clustered_idx]:
                                        check_is_in_line_of_sight_indices_dict[point1_clustered_idx][point_idx2] = check_is_in_line_of_sight(costmap.costmap_info, binary_non_free_image,
                                                                                                                                             point1_clustered_point, point2)
                                    is_in_line_of_sight = check_is_in_line_of_sight_indices_dict[point1_clustered_idx][point_idx2]

                                    if is_in_line_of_sight:
                                        line_point1 = [point1_clustered_point.x, point1_clustered_point.y]
                                        line_point2 = [point2.x, point2.y]

                                        if (hull_polyhedron is None) or (not is_line_segment_convex_hull_intersect(line_point1, line_point2, hull_polyhedron)):
                                            clustered_point_indices.append(point_idx2)
                                            processed_point_indices_set.add(point_idx2)

                                            point1_clustered_idx_set.add(point_idx2)
                                            point1_cluster_updated = True
                                            break

                        if not point1_cluster_updated:
                            break

                clustered_point_indices_list.append(clustered_point_indices)
    elif len(point_list)==1:
        clustered_point_indices_list.append([0])

    clustered_point_list = []
    for clustered_point_indices in clustered_point_indices_list:
        clustered_point = []
        for point_idx in clustered_point_indices:
            clustered_point.append(point_list[point_idx])
        clustered_point_list.append(clustered_point)
    return clustered_point_list


def calc_contours_outside_coverage_polyhedron(binary_polyhedron_map, binary_area_image, ignore_small_area_square_meters=1.0):
    """
    Calculate contours of input binary area outside coverage polyhedron

    Parameters
    ----------
    binary_polyhedron_map : costmap_utils.Costmap
        The binary coverage polyhedron map
    binary_area_image : numpy.ndarray
        The binary area image
    ignore_small_area_square_meters : float
        The value of contour area to ignore small areas

    Returns
    -------
    outside_polyhedron_contours : numpy.ndarray
        The contours of input binary area outside the coverage polyhedron
    outside_polyhedron_contours_id_map : costmap_utils.Costmap
        The contour ID map of input binary area outside the coverage polyhedron.
        The contours of input binary area outside the coverage polyhedron will have IDs of positive value. Other area will have ID of 0.
    """
    binary_coverage_polyhedron_image = np.array(binary_polyhedron_map.occupancy_grid_data, dtype=np.uint8).reshape(binary_polyhedron_map.height, binary_polyhedron_map.width)
    binary_outside_coverage_polyhedron_image = 1 - binary_coverage_polyhedron_image

    ## fill coverage polyhedron area as obstacles
    binary_area_outside_coverage_polyhedron_image = cv2.bitwise_and(binary_area_image, binary_outside_coverage_polyhedron_image)

    ## debug visualization
    # cv2.imshow("binary area image outside coverage polyhedron", binary_area_outside_coverage_polyhedron_image*255)
    # cv2.waitKey(1)

    ## find binary area contours outside coverage polyhedron
    outside_polyhedron_contours, _ = cv2.findContours(binary_area_outside_coverage_polyhedron_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # ignore small binary area contours outside coverage polyhedron
    if ignore_small_area_square_meters>0:
        ignore_small_area_square_pixels = int((math.sqrt(ignore_small_area_square_meters)/binary_polyhedron_map.resolution)**2)
        outside_polyhedron_contours = list(filter(lambda x: cv2.contourArea(x) > ignore_small_area_square_pixels, outside_polyhedron_contours))

    outside_polyhedron_contours_id_image = np.zeros(binary_area_image.shape, np.float32)
    for contour_idx, contour in enumerate(outside_polyhedron_contours):
        contour_id = contour_idx + 1
        cv2.drawContours(outside_polyhedron_contours_id_image, [contour], -1, color=contour_id, thickness=-1)
        cv2.drawContours(outside_polyhedron_contours_id_image, [contour], -1, color=contour_id, thickness=1)

    ## debug visualization
    # vis_outside_polyhedron_contours_id_image = np.zeros(binary_area_image.shape + (3,), dtype=np.uint8)
    # contour_ids = np.unique(outside_polyhedron_contours_id_image)
    # for contour_id in contour_ids:
    #     if contour_id>0:
    #         vis_outside_polyhedron_contours_id_image[outside_polyhedron_contours_id_image==contour_id] = [np.random.randint(0, 256), np.random.randint(0, 256), np.random.randint(0, 256)]
    # cv2.imshow("visualize contours outside coverage polyhedron image", vis_outside_polyhedron_contours_id_image*255)
    # cv2.waitKey(1)

    return [costmap_utils.map_to_world_np_array(binary_polyhedron_map.costmap_info, contour.squeeze(1)) for contour in outside_polyhedron_contours], \
        costmap_utils.Costmap(binary_polyhedron_map.costmap_info, tuple(outside_polyhedron_contours_id_image.flatten()))


def calc_contours_centroids(costmap_info, contours):
    """
    Calculate controids of contours

    Parameters
    ----------
    costmap_info : costmap_utils.CostmapInfo
        The cost map info
    contours : numpy.ndarray
        The input contours

    Returns
    -------
    centroid_point_list : array-like
        The list of centroid points for input contours
    """
    if len(contours)==0:
        return []

    map_centroid_point_list = []
    for contour in contours:
        binary_contour_image = costmap_utils.calc_binary_contours_image(costmap_info, [contour])

        if np.any(binary_contour_image):
            # fast version
            # dist_map = cv2.distanceTransform(binary_contour_image, distanceType=cv2.DIST_L1, maskSize=cv2.DIST_MASK_3)
            # slow version
            dist_map = cv2.distanceTransform(binary_contour_image, distanceType=cv2.DIST_L2, maskSize=cv2.DIST_MASK_5)
            _, radius, _, center = cv2.minMaxLoc(dist_map)
            map_centroid_point_list.append(center)

            ## debug visualization
            # vis_binary_contour_image = (binary_contour_image * 255).astype(np.uint8)
            # vis_binary_contour_image = cv2.cvtColor(vis_binary_contour_image, cv2.COLOR_GRAY2BGR)
            # cv2.circle(vis_binary_contour_image, center, radius=int(radius), color=(255, 0, 0), thickness=1, lineType=cv2.LINE_8)
            # cv2.circle(vis_binary_contour_image, center, radius=1, color=(0, 0, 255), thickness=cv2.FILLED, lineType=cv2.LINE_8)
            # cv2.imshow("binary contour image", vis_binary_contour_image)
            # cv2.waitKey(1)

    if len(map_centroid_point_list)>0:
        world_centroid_point_list = costmap_utils.map_to_world_np_array(costmap_info, np.array(map_centroid_point_list))
        return [Point(point[0], point[1], 0.0) for point in world_centroid_point_list]
    else:
        return []


def calc_contours_attributes(costmap_info, contours):
    """
    Calculate attributes of contours

    Parameters
    ----------
    costmap_info : costmap_utils.CostmapInfo
        The cost map info
    contours : numpy.ndarray
        The input contours

    Returns
    -------
    attributes_list : array-like
        The list of attributes for input contours
    """
    if len(contours)==0:
        return []

    attributes_list = []
    for contour in contours:
        if len(contour)>3:
            contour_map = costmap_utils.world_to_map_np_array(costmap_info, contour)

            area_pixels = cv2.contourArea(contour_map)
            area_meters = (math.sqrt(area_pixels)*costmap_info.resolution)**2

            contour_map_float = contour_map.astype(float)
            mean = np.empty((0))
            mean, eigen_vectors, eigen_values = cv2.PCACompute2(contour_map_float, mean)

            mean_meters = costmap_utils.map_to_world_np_array(costmap_info, mean)

            contour_map_project = (contour_map_float - mean) @ eigen_vectors.transpose()
            contour_length_meters = (np.max(contour_map_project[:,0]) - np.min(contour_map_project[:,0])) * costmap_info.resolution
            contour_width_meters = (np.max(contour_map_project[:,1]) - np.min(contour_map_project[:,1])) * costmap_info.resolution

            contour_stddev_length_meters = math.sqrt(eigen_values[0]) * costmap_info.resolution * 2.0
            contour_stddev_width_meters = math.sqrt(eigen_values[1]) * costmap_info.resolution * 2.0

            length_angle = math.atan2(eigen_vectors[0,1], eigen_vectors[0,0])
            width_angle = math.atan2(eigen_vectors[1,1], eigen_vectors[1,0])

            attributes_list.append({"area": area_meters, "mean": mean_meters[0].tolist(), "length": contour_length_meters, "width": contour_width_meters, \
                "stddev_length": contour_stddev_length_meters, "stddev_width": contour_stddev_width_meters, "length_angle": length_angle, "width_angle": width_angle})
        else:
            rospy.logerr("Invalid contour is found")
            attributes_list.append({"area": 0.0, "mean": contour[0].tolist(), "length": 0.0, "width": 0.0, \
                "stddev_length": 0.0, "stddev_width": 0.0, "length_angle": 0.0, "width_angle": 0.0})
    return attributes_list


def calc_heuristic_route_contour_confidence(attributes_list, min_area=1.0, max_area=5.0, min_length=1.0, max_length=5.0, min_width=0.5, max_width=1.5, min_conf=0.1):
    """
    Calculate heuristic confidence value if contours are route from attributes of contours for test

    Parameters
    ----------
    attributes_list : array-like
        The list of attributes for contours

    Returns
    -------
    confidence_list : array-like
        The list of confidence value if contours are route
    """
    if len(attributes_list)==0:
        return []

    confidence_list = []
    for attributes in attributes_list:
        if ("area" in attributes) and ("length" in attributes) and ("width" in attributes) and (attributes["area"]>0) and (attributes["length"]>0) and (attributes["width"]>0):
            conf = 1.0
            conf = max(conf * (min(max_area, attributes["area"]) - min_area)/(max_area - min_area), min_conf)
            conf = max(conf * (min(max_length, attributes["length"]) - min_length)/(max_length - min_length), min_conf)
            conf = max(conf * (min(max_width, attributes["width"]) - min_width)/(max_width - min_width), min_conf)
        else:
            conf = min_conf
        confidence_list.append(conf)
    return confidence_list


def select_forward_contours(contours, robot_pose):
    """
    Select contours which have at least one contour point in forward from robot

    Parameters
    ----------
    outside_polyhedron_contours : numpy.ndarray
        The input contours
    robot_pose : geometry_msgs.msg.Pose
        The robot pose

    Returns
    -------
    array-like
        The list of indices of contours which have at least one contour point in forward from robot
    """
    if len(contours)==0:
        return []

    forward_contours_indices = []
    robot_pose_quat = (robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w)
    for contour_idx, contour in enumerate(contours):
        is_forward_contour = False
        for contour_point in contour:
            point_quat = calc_point_direction_quaternion(robot_pose.position, Point(contour_point[0], contour_point[1], 0))
            robot_point_quat_diff = calc_relative_orientation_quaternion(robot_pose_quat, point_quat)
            robot_point_yaw_diff = tf.transformations.euler_from_quaternion(robot_point_quat_diff)[2]
            if math.cos(robot_point_yaw_diff)>0:
                is_forward_contour = True
                break
        if is_forward_contour:
            forward_contours_indices.append(contour_idx)

    return forward_contours_indices


def cluster_point_by_contour_id_map(point_list, contours_id_map):
    """
    Cluster input points by using contour ID map

    Parameters
    ----------
    point_list : array-like
        The list of points to cluster
    contours_id_map : costmap_utils.Costmap
        The contour ID map. Contour areas will have IDs of positive value. Other area will have ID of 0.

    Returns
    -------
    clustered_point_list : array-like
        The list of clustered points
    clustered_point_contour_id_set : set
        The set of contour ID of clustered points
    """
    # cluster points by contour IDs
    clustered_point_indices_list = []
    clustered_point_contour_id_set = set()
    contour_id_cluster_index_dict = {}
    for point_idx, point in enumerate(point_list):
        point_map_x, point_map_y = costmap_utils.world_to_map(contours_id_map.costmap_info, point.x, point.y)
        contour_id = contours_id_map.get_cost(point_map_x, point_map_y)
        if contour_id>0:
            if contour_id in contour_id_cluster_index_dict:
                clustered_point_indices_list[contour_id_cluster_index_dict[contour_id]].append(point_idx)
            else:
                contour_id_cluster_index_dict[contour_id] = len(clustered_point_indices_list)
                clustered_point_indices_list.append([point_idx])
                clustered_point_contour_id_set.add(contour_id)

    clustered_point_list = []
    for clustered_point_indices in clustered_point_indices_list:
        clustered_point = []
        for point_idx in clustered_point_indices:
            clustered_point.append(point_list[point_idx])
        clustered_point_list.append(clustered_point)
    return clustered_point_list, clustered_point_contour_id_set


def sort_point_list_clockwise(robot_pose, point_list, max_abs_yaw_forward_point=math.pi/6.0, min_abs_yaw_backward_point=math.pi*5.0/6.0, filter_forward_points=True):
    """
    Sort list of points by direction from robot pose in clockwise order
    If point in forward direciton exists, the first point is forward direction.

    Parameters
    ----------
    robot_pose : geometry_msgs.msg.Pose
        The robot pose
    point_list : array-like
        The list of points to cluster
    max_abs_yaw_forward_point : float
        The maximum yaw differenct to point which is considered as forward point
    min_abs_yaw_backward_point : float
        The minimum yaw differenct to point which is considered as backward point
    filter_forward_points : bool
        If True, select only one point for forward direction in sorted points list.
        Otherwise, return all points in sorted points list.

    Returns
    -------
    sorted_point_list : array-like
        The list of sorted points
    has_forward_point : bool
        If sorted points has point in forward, return True. Otherwise, return False
    """
    # find forward/backward farthest point, and store direction of all points
    forward_point_idx, _, robot_point_yaw_diff_list  = select_forward_backward_farthest_point_idx(robot_pose, point_list,
                                                                                                max_abs_yaw_forward_point=max_abs_yaw_forward_point,
                                                                                                min_abs_yaw_backward_point=min_abs_yaw_backward_point)


    # convert yaw difference to range [0, 2.0*math.pi] to sort point in closkwise order
    robot_point_yaw_diff_list = [robot_point_yaw_diff if robot_point_yaw_diff>=0 else math.pi*2.0+robot_point_yaw_diff for robot_point_yaw_diff in robot_point_yaw_diff_list]

    # sort point lists in clockwise
    sorted_point_list_indices = list(np.argsort(-np.array(robot_point_yaw_diff_list)))
    sorted_point_list = [point_list[index] for index in sorted_point_list_indices]

    if forward_point_idx is not None:
        has_forward_point = True

        # if forward point is close to 2.0*math.pi and not the first element, move forward point to the first element
        sorted_forward_point_idx = sorted_point_list_indices.index(forward_point_idx)
        if (sorted_forward_point_idx is not None) and (sorted_forward_point_idx!=0):
            sorted_forward_point = sorted_point_list.pop(sorted_forward_point_idx)
            sorted_point_list.insert(0, sorted_forward_point)
    else:
        has_forward_point = False

    if has_forward_point and filter_forward_points:
        # remove points which are not first elements but in forward direction
        robot_pose_quat = (robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w)
        sorted_point_selector = [True] * len(sorted_point_list)
        for point_idx in range(len(sorted_point_list))[1:]:
            point = sorted_point_list[point_idx]

            point_quat = calc_point_direction_quaternion(robot_pose.position, point)
            robot_point_quat_diff = calc_relative_orientation_quaternion(robot_pose_quat, point_quat)
            robot_point_yaw_diff = tf.transformations.euler_from_quaternion(robot_point_quat_diff)[2]
            abs_robot_point_yaw_diff = abs(robot_point_yaw_diff)

            if (abs_robot_point_yaw_diff<max_abs_yaw_forward_point):
                sorted_point_selector[point_idx] = False
        sorted_point_list = list(itertools.compress(sorted_point_list, sorted_point_selector))

    return sorted_point_list, has_forward_point


def select_forward_backward_farthest_point_idx(robot_pose, point_list, max_abs_yaw_forward_point=math.pi/4.0, min_abs_yaw_backward_point=math.pi*3.0/4.0, eps_yaw_forward_backward_point=math.pi/90.0):
    """
    Select the points that are in forward or backward direction from robot and farthest ones
    Select the point which has largest or smallest cosine value from robot at first,
    then find the points which has close cosine value to the largest or smallest cosine value and farthest cosine distance.

    Parameters
    ----------
    robot_pose : geometry_msgs.msg.Pose
        The robot pose
    point_list : array-like
        The list of points to cluster
    max_abs_yaw_forward_point : float
        The maximum yaw differenct to point which is considered as forward point
    min_abs_yaw_backward_point : float
        The minimum yaw differenct to point which is considered as backward point
    eps_yaw_forward_backward_point : float
        The threshold for finding forward or backwards points which are close to the points with largest or smallest cosine value from robot

    Returns
    -------
    forward_point_idx : int
        The index of point whose direction from robot pose is close to forward direction and farthest
    backward_point_idx : int
        The index of point whose direction from robot pose is close to backward direction and farthest
    robot_point_yaw_list : array-like
        The list of yaw from robot pose to input points
    """
    robot_pose_quat = (robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w)

    largest_cos_point_idx = None
    largest_cos_point_cos = sys.float_info.min
    largest_cos_point_yaw = None
    largest_cos_point_cos_distance = None
    forward_point_index_list = []
    forward_point_index_point_distance_dict = {}
    forward_point_index_point_yaw_dict = {}

    smallest_cos_point_idx = None
    smallest_cos_point_cos = sys.float_info.max
    smallest_cos_point_yaw = None
    smallest_cos_point_cos_distance = None
    backward_point_index_list = []
    backward_point_index_point_distance_dict = {}
    backward_point_index_point_yaw_dict = {}

    robot_point_yaw_list = []

    for point_idx, point in enumerate(point_list):
        distance = calc_point_distance(robot_pose.position, point)
        point_quat = calc_point_direction_quaternion(robot_pose.position, point)
        robot_point_quat = calc_relative_orientation_quaternion(robot_pose_quat, point_quat)
        robot_point_yaw = tf.transformations.euler_from_quaternion(robot_point_quat)[2]
        abs_robot_point_yaw = abs(robot_point_yaw)
        cos_robot_point_yaw = math.cos(robot_point_yaw)

        if abs_robot_point_yaw<max_abs_yaw_forward_point:
            forward_point_index_list.append(point_idx)
            forward_point_index_point_distance_dict[point_idx] = distance
            forward_point_index_point_yaw_dict[point_idx] = robot_point_yaw

            if (abs_robot_point_yaw<max_abs_yaw_forward_point) and (cos_robot_point_yaw>largest_cos_point_cos):
                largest_cos_point_idx = point_idx
                largest_cos_point_cos = cos_robot_point_yaw
                largest_cos_point_yaw = robot_point_yaw
                largest_cos_point_cos_distance = distance * cos_robot_point_yaw
        elif abs_robot_point_yaw>min_abs_yaw_backward_point:
            backward_point_index_list.append(point_idx)
            backward_point_index_point_distance_dict[point_idx] = distance
            backward_point_index_point_yaw_dict[point_idx] = robot_point_yaw

            if (abs_robot_point_yaw>min_abs_yaw_backward_point) and (cos_robot_point_yaw<smallest_cos_point_cos):
                smallest_cos_point_idx = point_idx
                smallest_cos_point_cos = cos_robot_point_yaw
                smallest_cos_point_yaw = robot_point_yaw
                smallest_cos_point_cos_distance = distance * cos_robot_point_yaw

        robot_point_yaw_list.append(robot_point_yaw)

    if largest_cos_point_idx is not None:
        # if forward point with largest cosine value is found, check if farther point exist within the yaw difference of eps_yaw_forward_point
        for forward_point_idx in forward_point_index_list:
            forward_point_distance = forward_point_index_point_distance_dict[forward_point_idx]
            forward_point_yaw = forward_point_index_point_yaw_dict[forward_point_idx]
            if (forward_point_idx!=largest_cos_point_idx) and (abs(forward_point_yaw-largest_cos_point_yaw)<eps_yaw_forward_backward_point):
                point_cos_distance = forward_point_distance * math.cos(forward_point_yaw)
                if point_cos_distance>largest_cos_point_cos_distance:
                    largest_cos_point_idx = forward_point_idx
                    largest_cos_point_cos_distance =point_cos_distance

    if smallest_cos_point_idx is not None:
        # if backward point with smallest cosine value is found, check if farther point exist within the yaw difference of eps_yaw_forward_point
        for backward_point_idx in backward_point_index_list:
            backward_point_distance = backward_point_index_point_distance_dict[backward_point_idx]
            backward_point_yaw = backward_point_index_point_yaw_dict[backward_point_idx]
            if (backward_point_idx!=smallest_cos_point_idx) and (abs(backward_point_yaw-smallest_cos_point_yaw)<eps_yaw_forward_backward_point):
                point_cos_distance = backward_point_distance * math.cos(backward_point_yaw)
                if point_cos_distance<smallest_cos_point_cos_distance:
                    smallest_cos_point_idx = backward_point_idx
                    smallest_cos_point_cos_distance =point_cos_distance

    return largest_cos_point_idx, smallest_cos_point_idx, robot_point_yaw_list


def select_farthest_point_idx_with_forward_priority(robot_pose, point_list, max_abs_yaw_forward_point=math.pi/4.0, priority_max_abs_yaw_forward_point=math.pi/18.0):
    """
    Select the points that is farthest from robot by prioritizing forward direction
    If any points exist in forward direction, check if any forward points have yaw smaller than priority yaw threshold value at first,
    then select the forward farthest point which has largest closest direction to robot pose
    Otherwise, select the point which is farthest from robot.

    Parameters
    ----------
    robot_pose : geometry_msgs.msg.Pose
        The robot pose
    point_list : array-like
        The list of points
    max_abs_yaw_forward_point : float
        The maximum yaw differenct to point which is considered as forward point
    forward_distance_percentile : int
        The threshold to select forward points

    Returns
    -------
    int
        The index of point that is farthest from robot with prioritizing forward direction
    """
    robot_pose_position = robot_pose.position
    robot_pose_quat = (robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w)

    farthest_point_idx = None
    farthest_point_distance = sys.float_info.min
    forward_point_index_list = []
    forward_point_index_abs_robot_point_yaw_dict = {}
    forward_point_index_cos_distance_dict = {}
    for point_idx, point in enumerate(point_list):
        distance = calc_point_distance(robot_pose.position, point)
        if distance>farthest_point_distance:
            farthest_point_idx = point_idx
            farthest_point_distance = distance

        point_quat = calc_point_direction_quaternion(robot_pose_position, point)
        robot_point_quat = calc_relative_orientation_quaternion(robot_pose_quat, point_quat)
        robot_point_yaw = tf.transformations.euler_from_quaternion(robot_point_quat)[2]
        abs_robot_point_yaw = abs(robot_point_yaw)
        if abs_robot_point_yaw<max_abs_yaw_forward_point:
            forward_point_index_list.append(point_idx)
            forward_point_index_abs_robot_point_yaw_dict[point_idx] = abs_robot_point_yaw
            forward_point_index_cos_distance_dict[point_idx] = distance * math.cos(robot_point_yaw)

    if len(forward_point_index_list)>0:
        # if forward points exist, select the points which has yaw smaller than priority yaw threshold value at first
        priority_forward_point_dict_indices = np.argwhere(np.array(list(forward_point_index_abs_robot_point_yaw_dict.values()))<priority_max_abs_yaw_forward_point).flatten().tolist()
        priority_forward_point_indices = np.array(list(forward_point_index_abs_robot_point_yaw_dict.keys()))[priority_forward_point_dict_indices].tolist()
        if len(priority_forward_point_dict_indices)>0:
            select_forward_point_index_list = priority_forward_point_indices
        else:
            select_forward_point_index_list = forward_point_index_list

        # select forward points from list of forward points smaller than priority yaw threshold or original forward points
        largest_forward_point_cos_distance_point_idx = None
        largest_forward_point_cos_distance = sys.float_info.min
        for forward_point_idx in select_forward_point_index_list:
            forward_point_cos_distance = forward_point_index_cos_distance_dict[forward_point_idx]
            if forward_point_cos_distance>largest_forward_point_cos_distance:
                largest_forward_point_cos_distance_point_idx = forward_point_idx
                largest_forward_point_cos_distance = forward_point_cos_distance
        return largest_forward_point_cos_distance_point_idx
    else:
        return farthest_point_idx