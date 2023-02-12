# Copyright (c) 2021  IBM Corporation
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

import numpy as np


def is_intersect_perpendicular_point_to_line(point, line_start, line_end):
    """
    check if perpendicular line from point to line segment intersects
    Args:
        point: point in numpy.array
        line_start: line segment start point in numpy.array
        line_end: line segment end point in numpy.array
    Returns:
        result: Bool value if perpendicular line intersects or not
    """
    vec_start_to_point = point - line_start
    vec_start_to_end = line_end - line_start
    dot_product = np.dot(vec_start_to_point, vec_start_to_end)
    if dot_product > 0:
        norm_vec_start_to_end = np.linalg.norm(vec_start_to_end)
        projection = dot_product/norm_vec_start_to_end
        if projection < norm_vec_start_to_end:
            return True
        else:
            return False
    else:
        return False


def get_perpendicular_point_to_line(point, line_start, line_end):
    """
    find the perpendicular point to line segment from point
    Args:
        point: point in numpy.array
        line_start: line segment start point in numpy.array
        line_end: line segment end point in numpy.array
    Returns:
        result: found perpendicular point to line segment in numpy.array
    """
    vec_start_to_point = point - line_start
    vec_start_to_end = line_end - line_start
    dot_product = np.dot(vec_start_to_point, vec_start_to_end)
    norm_vec_start_to_end = np.linalg.norm(vec_start_to_end)
    projection = dot_product/norm_vec_start_to_end
    start_cross_vector = vec_start_to_end * (projection/norm_vec_start_to_end)
    return np.array([line_start[0]+start_cross_vector[0], line_start[1]+start_cross_vector[1]])


def get_closest_point_to_line(point, line_start, line_end):
    """
    find the closest point on line segments from point
    reference : https://gihyo.jp/dev/serial/01/as3/0053
    Args:
        point: point in numpy.array
        line_start: line segment start point in numpy.array
        line_end: line segment end point in numpy.array
    Returns:
        result: found closest point on line segment in numpy.array
    """
    vec_start_to_point = point - line_start
    vec_start_to_end = line_end - line_start
    dot_product = np.dot(vec_start_to_point, vec_start_to_end)
    if dot_product > 0:
        norm_vec_start_to_end = np.linalg.norm(vec_start_to_end)
        projection = dot_product/norm_vec_start_to_end
        if projection < norm_vec_start_to_end:
            start_cross_vector = vec_start_to_end * (projection/norm_vec_start_to_end)
            return np.array([line_start[0]+start_cross_vector[0], line_start[1]+start_cross_vector[1]])
        else:
            return np.copy(line_end)
    else:
        return np.copy(line_start)


def get_distance_to_queue_head(queue_expected_path_segment_idx, queue_expected_path_point, queue_expected_path_line_segments, queue_expected_path_line_segments_length):
    """
    calculate distance from a point along queue expected path to head point of queue expected path
    Args:
        queue_expected_path_segment_idx: segment of queue expected path which is closest to input point
        queue_expected_path_point: input point on queue expected path
        queue_expected_path_line_segments: line segments of queue expected path
        queue_expected_path_line_segments_length: line segments length of queue expected path
    Returns:
        result: distance from input point to head point of queue expected path
    """
    dist = 0.0
    for segment_idx in range(queue_expected_path_segment_idx, len(queue_expected_path_line_segments)):
        if segment_idx == queue_expected_path_segment_idx:
            dist += np.linalg.norm(queue_expected_path_line_segments[segment_idx][1]-queue_expected_path_point)
        else:
            dist += queue_expected_path_line_segments_length[segment_idx]
    return dist
