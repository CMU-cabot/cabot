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

import math
import os
import yaml

import cv2
from nav_msgs.msg import MapMetaData
import numpy as np
import rospy
import tf


class CostmapInfo:

    def __init__(self, occupancy_grid_info):
        self._occupancy_grid_info = occupancy_grid_info
        self._origin_quaternion = (self._occupancy_grid_info.origin.orientation.x, self._occupancy_grid_info.origin.orientation.y, 
                                   self._occupancy_grid_info.origin.orientation.z, self._occupancy_grid_info.origin.orientation.w)
        self._origin_yaw = tf.transformations.euler_from_quaternion(self._origin_quaternion)[2]


    @property
    def occupancy_grid_info(self):
        return self._occupancy_grid_info


    @property
    def resolution(self):
        return self._occupancy_grid_info.resolution


    @property
    def width(self):
        return self._occupancy_grid_info.width


    @property
    def height(self):
        return self._occupancy_grid_info.height


    @property
    def origin(self):
        return self._occupancy_grid_info.origin


    @property
    def origin_quaternion(self):
        return self._origin_quaternion


    @property
    def origin_yaw(self):
        return self._origin_yaw


class Costmap:

    def __init__(self, costmap_info, occupancy_grid_data):
        rospy.loginfo("Start initializing Costmap...")
        self._costmap_info = costmap_info
        if type(occupancy_grid_data) is tuple:
            self._occupancy_grid_data = occupancy_grid_data
        else:
            self._occupancy_grid_data = tuple(occupancy_grid_data)
        rospy.loginfo("Finish initializing Costmap.")


    @property
    def costmap_info(self):
        return self._costmap_info


    @property
    def occupancy_grid_data(self):
        return self._occupancy_grid_data


    @property
    def resolution(self):
        return self._costmap_info.resolution


    @property
    def width(self):
        return self._costmap_info.width


    @property
    def height(self):
        return self._costmap_info.height


    @property
    def origin(self):
        return self._costmap_info.origin


    @property
    def origin_quaternion(self):
        return self._costmap_info.origin_quaternion


    @property
    def origin_yaw(self):
        return self._costmap_info.origin_yaw


    def get_cost(self, costmap_x, costmap_y):
        return self._occupancy_grid_data[costmap_y*self._costmap_info.width + costmap_x]


def load_costmap_file(map_path):
    """
    Load costmap from input costmap file base path
    Note that ROS does not support rotated map, this function support rotated costmap

    Parameters
    ----------
    map_path : str
        The input costmap path

    Returns
    -------
    costmap_utils.Costmap
        The loaded costmap class
    """
    map_yaml = map_path + ".yaml"
    map_pgm = map_path + ".pgm"
    assert os.path.exists(map_yaml)
    assert os.path.exists(map_pgm)

    with open(map_yaml, 'r') as yml:
        map_config = yaml.load(yml, Loader=yaml.SafeLoader)
    print("map_config = " + str(map_config))

    # support only negate 0
    assert map_config['negate']==0

    map_data = cv2.imread(map_pgm, cv2.IMREAD_GRAYSCALE).astype(int)
    print("map_data.shape = " + str(map_data.shape))

    if np.all((map_data==0) | (map_data==205) | (map_data==254)):
        # assume map is created by map_saver
        # https://github.com/ros-planning/navigation/blob/noetic-devel/map_server/src/map_saver.cpp
        grid_data = np.empty_like(map_data, dtype=np.int8)
        grid_data[map_data==0] = 100  # occupied
        grid_data[map_data==205] = -1 # unknown
        grid_data[map_data==254] = 0  # free
        assert np.all((grid_data==100) | (grid_data==-1) | (grid_data==0))
    else:
        # assume map is created by cartographer_ros
        grid_data = np.where(map_data==128, -1, 100*(255-map_data)/255).astype(np.int8)

    origin_quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, map_config['origin'][2])

    grid_info = MapMetaData()
    grid_info.resolution = map_config['resolution']
    grid_info.width = grid_data.shape[1]
    grid_info.height = grid_data.shape[0]
    grid_info.origin.position.x = map_config['origin'][0]
    grid_info.origin.position.y = map_config['origin'][1]
    grid_info.origin.position.z = 0.0
    grid_info.origin.orientation.x = origin_quaternion[0]
    grid_info.origin.orientation.y = origin_quaternion[1]
    grid_info.origin.orientation.z = origin_quaternion[2]
    grid_info.origin.orientation.w = origin_quaternion[3]

    return Costmap(CostmapInfo(grid_info), tuple(grid_data.flatten()))


def create_map_pgm(costmap):
    """
    Create a map pgm image from Costmap which will be saved as ROS map file

    Parameters
    ----------
    costmap_utils.Costmap
        The input costmap class

    Returns
    -------
    numpy.ndarray
        The numpy array which will be saved as a map pgm file
    """
    grid_data = np.array(costmap.occupancy_grid_data).reshape(costmap.height, costmap.width)

    if np.all((grid_data==0) | (grid_data==-1) | (grid_data==100)):
        # assume map is created by map_saver
        # https://github.com/ros-planning/navigation/blob/noetic-devel/map_server/src/map_saver.cpp
        map_data = np.empty_like(grid_data, dtype=np.uint8)
        map_data[grid_data==100] = 0  # occupied
        map_data[grid_data==-1] = 205 # unknown
        map_data[grid_data==0] = 254  # free
        assert np.all((map_data==0) | (map_data==205) | (map_data==254))
    else:
        # assume map is created by cartographer_ros
        map_data = np.where(grid_data==-1, 128, 255*(100-grid_data)/100).astype(np.uint8)

    return map_data


def is_point_in_map(costmap_info, world_x, world_y):
    """
    Check if input point exists in input map area

    Parameters
    ----------
    costmap_info : costmap_utils.CostmapInfo
        The cost map info
    world_x : int
        The x-coordinate of input point in world coordinate
    world_y : int
        The y-coordinate of input point in world coordinate

    Returns
    -------
    bool
        If input point exists in input map, return True. Otherwise, return False
    """
    map_origin_yaw = costmap_info.origin_yaw
    map_origin_position = costmap_info.origin.position
    map_resolution = costmap_info.resolution

    if map_origin_yaw==0.0:
        costmap_x = int((world_x - map_origin_position.x) / map_resolution)
        costmap_y = int((world_y - map_origin_position.y) / map_resolution)

        if (costmap_x>=0) and (costmap_x<costmap_info.width) and (costmap_y>=0) and (costmap_y<costmap_info.height):
            return True
        else:
            return False
    else:
        diff_world_x = world_x - map_origin_position.x
        diff_world_y = world_y - map_origin_position.y

        costmap_x = int((diff_world_x*math.cos(-map_origin_yaw) - diff_world_y*math.sin(-map_origin_yaw)) / map_resolution)
        costmap_y = int((diff_world_x*math.sin(-map_origin_yaw) + diff_world_y*math.cos(-map_origin_yaw)) / map_resolution)

        if (costmap_x>=0) and (costmap_x<costmap_info.width) and (costmap_y>=0) and (costmap_y<costmap_info.height):
            return True
        else:
            return False


def map_to_world(costmap_info, costmap_x, costmap_y):
    """
    Convert x-y coordinates of input point from map coordinate to world coordinate

    Parameters
    ----------
    costmap_info : costmap_utils.CostmapInfo
        The cost map info
    costmap_x : int
        The x-coordinate of input point in map coordinate
    costmap_y : int
        The y-coordinate of input point in map coordinate

    Returns
    -------
    world_x : float
        The x-coordinate of input point in world coordinate
    world_y : float
        The y-coordinate of input point in world coordinate
    """
    map_origin_yaw = costmap_info.origin_yaw
    map_origin_position = costmap_info.origin.position
    map_resolution = costmap_info.resolution

    if map_origin_yaw==0.0:
        world_x = map_origin_position.x + costmap_x * map_resolution
        world_y = map_origin_position.y + costmap_y * map_resolution
        return world_x, world_y
    else:
        rotate_costmap_x = costmap_x*math.cos(map_origin_yaw) - costmap_y*math.sin(map_origin_yaw)
        rotate_costmap_y = costmap_x*math.sin(map_origin_yaw) + costmap_y*math.cos(map_origin_yaw)

        world_x = map_origin_position.x + rotate_costmap_x * map_resolution
        world_y = map_origin_position.y + rotate_costmap_y * map_resolution
        return world_x, world_y


def world_to_map(costmap_info, world_x, world_y):
    """
    Convert x-y coordinates of input point from world coordinate to map coordinate

    Parameters
    ----------
    costmap_info : costmap_utils.CostmapInfo
        The cost map info
    world_x : float
        The x-coordinate of input point in world coordinate
    world_y : float
        The y-coordinate of input point in world coordinate

    Returns
    -------
    costmap_x : int
        The x-coordinate of input point in map coordinate
    costmap_y : int
        The y-coordinate of input point in map coordinate
    """
    map_origin_yaw = costmap_info.origin_yaw
    map_origin_position = costmap_info.origin.position
    map_resolution = costmap_info.resolution

    if map_origin_yaw==0.0:
        costmap_x = int((world_x - map_origin_position.x) / map_resolution)
        costmap_y = int((world_y - map_origin_position.y) / map_resolution)
    else:
        diff_world_x = world_x - map_origin_position.x
        diff_world_y = world_y - map_origin_position.y

        costmap_x = int((diff_world_x*math.cos(-map_origin_yaw) - diff_world_y*math.sin(-map_origin_yaw)) / map_resolution)
        costmap_y = int((diff_world_x*math.sin(-map_origin_yaw) + diff_world_y*math.cos(-map_origin_yaw)) / map_resolution)

    if costmap_x<0:
        # rospy.loginfo("fix to 0 from costmap x " + str(costmap_x))
        costmap_x = 0
    elif costmap_x>=costmap_info.width:
        # rospy.loginfo("fix to width-1 from costmap x " + str(costmap_x))
        costmap_x = costmap_info.width-1
    if costmap_y<0:
        # rospy.loginfo("fix to 0 from costmap y " + str(costmap_y))
        costmap_y = 0
    elif costmap_y>=costmap_info.height:
        # rospy.loginfo("fix to height-1 from costmap y " + str(costmap_y))
        costmap_y = costmap_info.height-1

    return costmap_x, costmap_y


def map_to_world_np_array(costmap_info, costmap_xy_np_array):
    """
    Convert x-y coordinates of input points from map coordinate to world coordinate

    Parameters
    ----------
    costmap_info : costmap_utils.CostmapInfo
        The cost map info
    costmap_xy_np_array : numpy.ndarray
        The x-y coordinates of points in map coordinate

    Returns
    -------
    world_xy_np_array : numpy.ndarray
        The x-y coordinates of points in world coordinate
    """
    map_origin_yaw = costmap_info.origin_yaw
    map_origin_position = costmap_info.origin.position
    map_resolution = costmap_info.resolution

    if map_origin_yaw==0.0:
        origin_xy_np_array = np.array([map_origin_position.x, map_origin_position.y])
        world_xy_np_array = origin_xy_np_array + costmap_xy_np_array * map_resolution
        return world_xy_np_array
    else:
        costmap_x_np_array = costmap_xy_np_array[..., 0].reshape(-1, 1)
        costmap_y_np_array = costmap_xy_np_array[..., 1].reshape(-1, 1)

        rotate_costmap_x_np_array = costmap_x_np_array*math.cos(map_origin_yaw) - costmap_y_np_array*math.sin(map_origin_yaw)
        rotate_costmap_y_np_array = costmap_x_np_array*math.sin(map_origin_yaw) + costmap_y_np_array*math.cos(map_origin_yaw)

        rotate_costmap_xy_np_array = np.hstack((rotate_costmap_x_np_array, rotate_costmap_y_np_array))
        if rotate_costmap_xy_np_array.shape!=costmap_xy_np_array.shape:
            rotate_costmap_xy_np_array = rotate_costmap_xy_np_array.reshape(costmap_xy_np_array.shape)

        origin_xy_np_array = np.array([map_origin_position.x, map_origin_position.y])
        world_xy_np_array = origin_xy_np_array + rotate_costmap_xy_np_array * map_resolution
        return world_xy_np_array


def world_to_map_np_array(costmap_info, world_xy_np_array, fix_boundary=False):
    """
    Convert x-y coordinates of input points from world coordinate to map coordinate

    Parameters
    ----------
    costmap_info : costmap_utils.CostmapInfo
        The cost map info
    world_xy_np_array : numpy.ndarray
        The x-y coordinates of points in world coordinate
    fix_boundary : bool
        If true, fix x-y coordinates of points that are out of bounds. Otherwise, do not fix.

    Returns
    -------
    costmap_xy_np_array : numpy.ndarray
        The x-y coordinates of points in map coordinate
    """
    map_origin_yaw = costmap_info.origin_yaw
    map_origin_position = costmap_info.origin.position
    map_resolution = costmap_info.resolution

    if map_origin_yaw==0.0:
        origin_xy_np_array = np.array([map_origin_position.x, map_origin_position.y])
        costmap_xy_np_array = ((world_xy_np_array - origin_xy_np_array) / map_resolution).astype(int)
    else:
        world_x_np_array = world_xy_np_array[:,0].reshape(-1, 1)
        world_y_np_array = world_xy_np_array[:,1].reshape(-1, 1)

        diff_world_x_np_array = world_x_np_array - map_origin_position.x
        diff_world_y_np_array = world_y_np_array - map_origin_position.y

        costmap_x_np_array = ((diff_world_x_np_array*math.cos(-map_origin_yaw) - diff_world_y_np_array*math.sin(-map_origin_yaw)) / map_resolution).astype(int)
        costmap_y_np_array = ((diff_world_x_np_array*math.sin(-map_origin_yaw) + diff_world_y_np_array*math.cos(-map_origin_yaw)) / map_resolution).astype(int)

        costmap_xy_np_array = np.hstack((costmap_x_np_array, costmap_y_np_array)).reshape(-1, 2)

    if fix_boundary:
        costmap_xy_np_array[:,0][costmap_xy_np_array[:,0]<0] = 0
        costmap_xy_np_array[:,0][costmap_xy_np_array[:,0]>=costmap_info.width] = costmap_info.width-1
        costmap_xy_np_array[:,1][costmap_xy_np_array[:,1]<0] = 0
        costmap_xy_np_array[:,1][costmap_xy_np_array[:,1]>=costmap_info.height] = costmap_info.height-1

    return costmap_xy_np_array


def calc_local_costmap(costmap, center_world_x, center_world_y, local_map_yaw, local_map_width, local_map_height):
    """
    Calculate local costmap for input costmap at input pose

    Parameters
    ----------
    costmap : costmap_utils.Costmap
        The cost map
    center_world_x : float
        The x-coordinate of center point in world coordinate
    center_world_y : float
        The y-coordinate of center point in world coordinate
    local_map_yaw : float
        The heading direction to calculate local costmap
    local_map_width : float
        The width of local lcostmap in meters
    local_map_height : float
        The height of local lcostmap in meters

    Returns
    -------
    costmap_utils.Costmap
        The output local costmap
    """
    # calculate map origin by rotating local map top-left position around local map center
    local_map_top_left_x = -local_map_width/2.0
    local_map_top_left_y = -local_map_height/2.0

    rotate_local_map_top_left_x = local_map_top_left_x*math.cos(local_map_yaw) - local_map_top_left_y*math.sin(local_map_yaw)
    rotate_local_map_top_left_y = local_map_top_left_x*math.sin(local_map_yaw) + local_map_top_left_y*math.cos(local_map_yaw)

    rotate_map_origin_x = center_world_x + rotate_local_map_top_left_x
    rotate_map_origin_y = center_world_y + rotate_local_map_top_left_y

    # create local map info
    local_map_width_pixels = int(local_map_width/costmap.resolution)
    local_map_height_pixels = int(local_map_height/costmap.resolution)
    local_map_yaw_quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, local_map_yaw)

    local_costmap_info = MapMetaData()
    local_costmap_info.resolution = costmap.resolution
    local_costmap_info.width = local_map_width_pixels
    local_costmap_info.height = local_map_height_pixels
    local_costmap_info.origin.position.x = rotate_map_origin_x
    local_costmap_info.origin.position.y = rotate_map_origin_y
    local_costmap_info.origin.position.z = 0.0
    local_costmap_info.origin.orientation.x = local_map_yaw_quaternion[0]
    local_costmap_info.origin.orientation.y = local_map_yaw_quaternion[1]
    local_costmap_info.origin.orientation.z = local_map_yaw_quaternion[2]
    local_costmap_info.origin.orientation.w = local_map_yaw_quaternion[3]

    # create local map image (slow version which support any data)
    '''
    local_costmap_image = np.full((local_map_height_pixels, local_map_width_pixels), -1, dtype=np.int8)

    costmap_image = np.array(costmap.occupancy_grid_data).reshape(costmap.height, costmap.width)
    center_map_x, center_map_y = world_to_map(costmap.costmap_info, center_world_x, center_world_y)
    for local_map_y in range(local_map_height_pixels):
        for local_map_x in range(local_map_width_pixels):
            map_diff_x = local_map_x - local_map_width_pixels/2.0
            map_diff_y = local_map_y - local_map_height_pixels/2.0

            rotate_map_diff_x = map_diff_x*math.cos(local_map_yaw) - map_diff_y*math.sin(local_map_yaw)
            rotate_map_diff_y = map_diff_x*math.sin(local_map_yaw) + map_diff_y*math.cos(local_map_yaw)

            map_x = center_map_x + int(rotate_map_diff_x)
            map_y = center_map_y + int(rotate_map_diff_y)

            if (map_y>=0) and (map_y<costmap.height) and (map_x>=0) and (map_x<costmap.width):
                local_costmap_image[local_map_y][local_map_x] = costmap_image[map_y][map_x]
    '''
    # create local map image (fast version which support only cost map data from -1 to 100)
    costmap_image = np.array(costmap.occupancy_grid_data).reshape(costmap.height, costmap.width)
    center_map_x, center_map_y = world_to_map(costmap.costmap_info, center_world_x, center_world_y)
    center = np.array([center_map_x, center_map_y])

    src_pts = np.array([[-local_map_width_pixels // 2, local_map_height_pixels // 2 - 1],
                        [-local_map_width_pixels // 2, -local_map_height_pixels // 2],
                        [local_map_width_pixels // 2 - 1, -local_map_height_pixels // 2],
                        [local_map_width_pixels // 2 - 1, local_map_height_pixels // 2 - 1]])

    R = np.array([[np.cos(local_map_yaw), -np.sin(local_map_yaw)],
                  [np.sin(local_map_yaw), np.cos(local_map_yaw)]])
    src_pts = np.matmul(R, src_pts.T).T + center
    src_pts = src_pts.astype(np.float32)

    dst_pts = np.array([[0, local_map_height_pixels - 1],
                        [0, 0],
                        [local_map_width_pixels - 1, 0],
                        [local_map_width_pixels - 1, local_map_height_pixels - 1]],
                        dtype=np.float32)

    M = cv2.getPerspectiveTransform(src_pts, dst_pts)

    costmap_image_uint8 = np.copy(costmap_image)
    costmap_image_uint8[np.where(costmap_image_uint8==-1)] = 254
    costmap_image_uint8 = costmap_image_uint8.astype(np.uint8)
    local_costmap_image_uint8 = cv2.warpPerspective(costmap_image_uint8, M, (local_map_width_pixels, local_map_height_pixels), flags=cv2.INTER_NEAREST, borderValue=254)
    local_costmap_image = local_costmap_image_uint8.astype(np.int64)
    local_costmap_image[np.where(local_costmap_image==254)] = -1

    return Costmap(CostmapInfo(local_costmap_info), tuple(local_costmap_image.flatten()))


def calc_binary_free_image(grid_data_image, lethal_cost_threshold):
    """
    Calculate binary free area image

    Parameters
    ----------
    grid_data_image : numpy.ndarray
        The cost map image
    lethal_cost_threshold : int
        If the cost map value is equal or larther than this value, that point is considered as lethal area

    Returns
    -------
    binary_free_image : numpy.ndarray
        The binary free area image
    """
    ## create binary image for free area
    binary_free_image = np.zeros(grid_data_image.shape, np.uint8)
    binary_free_image[np.where((grid_data_image>=0) & (grid_data_image<lethal_cost_threshold))] = 1

    ## debug visualization
    # cv2.imshow("binary free image", binary_free_image*255)
    # cv2.waitKey(1)

    return binary_free_image


def calc_binary_non_free_image(grid_data_image, lethal_cost_threshold):
    """
    Calculate binary non free area image

    Parameters
    ----------
    grid_data_image : numpy.ndarray
        The cost map image
    lethal_cost_threshold : int
        If the cost map value is equal or larther than this value, that point is considered as lethal area

    Returns
    -------
    binary_non_free_image : numpy.ndarray
        The binary non free area image
    """
    binary_free_image = calc_binary_free_image(grid_data_image, lethal_cost_threshold)
    return 1 - binary_free_image


def calc_binary_lethal_image(grid_data_image, lethal_cost_threshold):
    """
    Calculate binary lethal area image

    Parameters
    ----------
    grid_data_image : numpy.ndarray
        The cost map image
    lethal_cost_threshold : int
        If the cost map value is equal or larther than this value, that point is considered as lethal area

    Returns
    -------
    binary_lethal_image : numpy.ndarray
        The binary lethal area image
    """
    ## create binary image for lethal area
    binary_lethal_image = np.zeros(grid_data_image.shape, np.uint8)
    binary_lethal_image[grid_data_image>=lethal_cost_threshold] = 1

    ## debug visualization
    # cv2.imshow("binary lethal image", binary_lethal_image*255)
    # cv2.waitKey(1)

    return binary_lethal_image


def calc_binary_unknown_image(grid_data_image):
    """
    Calculate binary unknown area image

    Parameters
    ----------
    grid_data_image : numpy.ndarray
        The cost map image

    Returns
    -------
    binary_unknown_image : numpy.ndarray
        The binary unknown area image
    """
    ## create binary image for lethal area
    binary_unknown_image = np.zeros(grid_data_image.shape, np.uint8)
    binary_unknown_image[grid_data_image<0] = 1

    ## debug visualization
    # cv2.imshow("binary unknown image", binary_unknown_image*255)
    # cv2.waitKey(1)

    return binary_unknown_image


def calc_binary_frontier_image(grid_data_image, lethal_cost_threshold, min_frontier_size_pixels=None, binary_free_image=None, binary_lethal_image=None, return_free_frontier=False):
    """
    Calculate binary frontier area image

    Parameters
    ----------
    grid_data_image : numpy.ndarray
        The cost map image
    lethal_cost_threshold : int
        If the cost map value is equal or larther than this value, that point is considered as lethal area
    min_frontier_size_pixels : int
        The minimum size of frontier area in pixels
    binary_free_image : numpy.ndarray
        The binary free area image
    binary_lethal_image : numpy.ndarray
        The binary lethal area image
    return_free_frontier : bool
        If True, return free frontier area image. Otherwise, return unknown frontier area image.

    Returns
    -------
    binary_frontier_image : numpy.ndarray
        The binary frontier area image
    """
    ## create binary image for unknown area
    binary_unknown_image = np.zeros(grid_data_image.shape, np.uint8)
    binary_unknown_image[grid_data_image<0] = 1

    ## debug visualization
    # cv2.imshow("binary unknown image", binary_unknown_image*255)
    # cv2.waitKey(1)

    if binary_lethal_image is None:
        ## create binary image for lethal area if it is not given
        binary_lethal_image = calc_binary_lethal_image(grid_data_image, lethal_cost_threshold)

    ## debug visualization
    # cv2.imshow("binary lethal image", binary_lethal_image*255)
    # cv2.waitKey(1)

    ## do not set 8-neighbor lethal area as unknown to fix digitization error
    dilate_8neighbor_binary_lethal_image = cv2.dilate(binary_lethal_image, np.ones((3,3), np.uint8), iterations=1)
    binary_unknown_image[np.where((binary_unknown_image==1) & (dilate_8neighbor_binary_lethal_image==1))] = 0

    ## debug visualization
    # cv2.imshow("binary unknown image fixed digitization error", binary_unknown_image*255)
    # cv2.waitKey(1)

    if binary_free_image is None:
        ## create binary image for free area if it is not given
        binary_free_image = calc_binary_free_image(grid_data_image, lethal_cost_threshold)

    ## create frontier image
    if return_free_frontier:
        ## calculate 8-neighbor dilate binary unknown image to calculate free frontier image
        dilate_8neighbor_binary_unknown_image = cv2.dilate(binary_unknown_image, np.ones((3,3), np.uint8), iterations=1)
        binary_frontier_image = cv2.bitwise_and(binary_free_image, dilate_8neighbor_binary_unknown_image)
    else:
        ## calculate 8-neighbor dilate binary free image to calculate unknown frontier image
        dilate_8neighbor_binary_free_image = cv2.dilate(binary_free_image, np.ones((3,3), np.uint8), iterations=1)
        binary_frontier_image = cv2.bitwise_and(binary_unknown_image, dilate_8neighbor_binary_free_image)

    ## debug visualization
    # cv2.imshow("binary frontier image", binary_frontier_image*255)
    # cv2.waitKey(1)

    if (min_frontier_size_pixels is not None) and (min_frontier_size_pixels>0):
        frontier_contours, _ = cv2.findContours(binary_frontier_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for frontier_contour in frontier_contours:
            if cv2.arcLength(frontier_contour, False)<min_frontier_size_pixels:
                cv2.drawContours(binary_frontier_image, [frontier_contour], -1, color=0, thickness=-1)
                cv2.drawContours(binary_frontier_image, [frontier_contour], -1, color=0, thickness=1)

            ## debug visualization
            # vis_frontier_contour_image = np.ones(grid_data_image.shape + (3,), dtype=np.uint8)
            # cv2.drawContours(vis_frontier_contour_image, [frontier_contour], -1, color=0, thickness=-1)
            # cv2.drawContours(vis_frontier_contour_image, [frontier_contour], -1, color=0, thickness=1)
            # cv2.imshow("frontier contour", vis_frontier_contour_image*255)
            # cv2.waitKey(1)

        ## debug visualization
        # cv2.imshow("binary frontier image after removing small frontier", binary_frontier_image*255)
        # cv2.waitKey(1)

    return binary_frontier_image


def _calc_costmap_inflation_dilation_kernel(costmap_info, inflation_radius):
    """
    Calculate dilation kernel for costmap inflation

    Parameters
    ----------
    costmap_info : costmap_utils.CostmapInfo
        The cost map info
    inflation_radius : float
        The inflation radius in meters

    Returns
    -------
    kernel : numpy.ndarray
        The dilation kernel
    """
    ## calculate dilation kernel for costmap inflation
    inflation_radius_pixels = int(inflation_radius / costmap_info.resolution)
    inflation_diameter_pixels = inflation_radius_pixels*2 + 1

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (inflation_diameter_pixels, inflation_diameter_pixels))
    ## debug visualization
    # vis_kernel = cv2.resize(kernel, dsize=None, fx=10.0, fy=10.0)
    # cv2.imshow("inflate costmap kernel", vis_kernel*255)
    # cv2.waitKey(1)

    return kernel


def calc_reachable_map(costmap_info, costmap_image, lethal_cost_threshold, robot_pose):
    """
    Calculate map of reachable area

    Parameters
    ----------
    costmap_info : costmap_utils.CostmapInfo
        The cost map info
    costmap_image : numpy.ndarray
        The cost map image
    lethal_cost_threshold : int
        If the cost map value is equal or larther than this value, that point is considered as lethal area
    robot_pose : geometry_msgs.msg.Pose
        The robot pose

    Returns
    -------
    reachable_area_image : numpy.ndarray
        The cost map image of reachable area.
    reachable_area_contours : numpy.ndarray
        The contour of reachable area.
    reachable_area_hierarchy : array-like
        The hierarchy of reachable area contours.
    """
    ## create gray image from cost map
    if costmap_image.dtype==np.int8:
        reachable_area_image = costmap_image.copy()
    else:
        reachable_area_image = costmap_image.astype(np.int8)

    ## debug visualization
    # cv2.imshow("cost map gray image", reachable_area_image)
    # cv2.waitKey(1)

    ## create binary image for free area
    binary_free_image = calc_binary_free_image(reachable_area_image, lethal_cost_threshold)

    ## debug visualization
    # cv2.imshow("cost map binary free image", binary_free_image*255)
    # cv2.waitKey(1)

    ## find only external contours for free areas at first
    free_contours, _ = cv2.findContours(binary_free_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    free_contours_image = np.zeros(reachable_area_image.shape, np.float32)
    for free_contour_idx, free_contour in enumerate(free_contours):
        free_contour_id = free_contour_idx + 1
        cv2.drawContours(free_contours_image, [free_contour], -1, color=free_contour_id, thickness=-1)
        cv2.drawContours(free_contours_image, [free_contour], -1, color=free_contour_id, thickness=1)

    ## debug visualization
    # vis_free_contours_image = np.zeros(reachable_area_image.shape + (3,), dtype=np.uint8)
    # free_contour_ids = np.unique(free_contours_image)
    # for free_contour_id in free_contour_ids:
    #     if free_contour_id>0:
    #         vis_free_contours_image[free_contours_image==free_contour_id] = [np.random.randint(0, 256), np.random.randint(0, 256), np.random.randint(0, 256)]
    # cv2.imshow("visualize free contour image", vis_free_contours_image*255)
    # cv2.waitKey(1)

    ## find free area ID where robot exists
    robot_pose_map_x, robot_pose_map_y = world_to_map(costmap_info, robot_pose.position.x, robot_pose.position.y)
    reachable_free_area_id = free_contours_image[robot_pose_map_y][robot_pose_map_x]
    if reachable_free_area_id>0:
        ## fill all free area where robot does not exist as unknown
        for free_contour_idx, free_contour in enumerate(free_contours):
            if (free_contour_idx+1)!=reachable_free_area_id:
                cv2.drawContours(reachable_area_image, [free_contour], -1, color=-1, thickness=-1)
                cv2.drawContours(reachable_area_image, [free_contour], -1, color=-1, thickness=1)

    ## find free areas with hierarchy
    reachable_area_contours_map, reachable_area_hierarchy = cv2.findContours(binary_free_image, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    reachable_area_contours = [map_to_world_np_array(costmap_info, reachable_area_contour_map).tolist() for reachable_area_contour_map in reachable_area_contours_map]

    ## debug visualization
    # cv2.imshow("reachable cost map gray image", reachable_area_image)
    # cv2.waitKey(1)

    ## debug visualization
    # binary_reachable_area_image = calc_binary_hierarchy_contours_image(costmap_info, reachable_area_contours, reachable_area_hierarchy)
    # cv2.imshow("reachable cost map binary image", binary_reachable_area_image*255)
    # cv2.waitKey(1)

    return reachable_area_image, reachable_area_contours, reachable_area_hierarchy


def inflate_costmap(costmap_info, costmap_image, lethal_cost_threshold, inflation_radius):
    """
    Inflate lethal area and frontier area in occupancy grid map

    Parameters
    ----------
    costmap_info : costmap_utils.CostmapInfo
        The cost map info
    costmap_image : numpy.ndarray
        The cost map image
    lethal_cost_threshold : int
        If the cost map value is equal or larther than this value, that point is considered as lethal area
    inflation_radius : float
        The radiuls to inflate lethal area and frontier area

    Returns
    -------
    inflate_costmap_image : numpy.ndarray
        The occupancy grid data
    """
    ## calculate dilation kernel for costmap inflation
    kernel = _calc_costmap_inflation_dilation_kernel(costmap_info, inflation_radius)

    ## create binary image for lethal area
    binary_lethal_image = calc_binary_lethal_image(costmap_image, lethal_cost_threshold)

    ## inflate lethal area
    dilate_binary_lethal_image = cv2.dilate(binary_lethal_image, kernel, iterations=1)

    ## debug visualization
    # cv2.imshow("dilate binary lethal image", dilate_binary_lethal_image*255)
    # cv2.waitKey(1)

    ## create gray image for inflate costmap
    grid_data_image = np.copy(costmap_image)
    grid_data_image[dilate_binary_lethal_image==1] = 100

    ## debug visualization
    # cv2.imshow("inflate gray image", grid_data_image)
    # cv2.waitKey(1)

    return grid_data_image


def inflate_binary_image(costmap_info, binary_image, inflation_radius):
    """
    Inflate binary image created by cost map

    Parameters
    ----------
    costmap_info : costmap_utils.CostmapInfo
        The cost map info which is used to create binary image
    binary_image : numpy.ndarray
        The binary image
    inflation_radius : float
        The radiuls to inflate binary image

    Returns
    -------
    inflate_binary_image : numpy.ndarray
        The inflated binary image
    """
    ## calculate dilation kernel for costmap inflation
    kernel = _calc_costmap_inflation_dilation_kernel(costmap_info, inflation_radius)

    ## inflate lethal area
    inflate_binary_image = cv2.dilate(binary_image, kernel, iterations=1)

    ## debug visualization
    # cv2.imshow("dilate binary image", inflate_binary_image*255)
    # cv2.waitKey(1)

    return inflate_binary_image


def calc_frontier_proximity_map(costmap, lethal_cost_threshold, proximity_threshold, min_frontier_size=None, robot_pose=None):
    """
    Calculate map of frontier proximity binary map

    Parameters
    ----------
    costmap : costmap_utils.Costmap
        The cost map
    lethal_cost_threshold : int
        If the cost map value is equal or larther than this value, that point is considered as lethal area
    proximity_threshold : float
        The threshold of frontier proximity area in meters
    min_frontier_size : float
        The minimum size of frontier area in meters
    robot_pose : geometry_msgs.msg.Pose
        The robot pose
        If this value is specified, calculate frontier proximity area only from reachable area.

    Returns
    -------
    frontier_proximity_map : costmap_utils.Costmap
        The map of frontier proximity area.
    """
    ## create gray image for costmap
    grid_data_image = np.array(costmap.occupancy_grid_data, dtype=np.int8).reshape(costmap.height, costmap.width)

    ## create binary image for free area
    binary_free_image = calc_binary_free_image(grid_data_image, lethal_cost_threshold)

    ## create binary image for lethal area
    binary_lethal_image = calc_binary_lethal_image(grid_data_image, lethal_cost_threshold)

    ## create binary image for frontier area
    min_frontier_size_pixels = None
    if min_frontier_size is not None:
        min_frontier_size_pixels = int(min_frontier_size / costmap.resolution)
    binary_frontier_image = calc_binary_frontier_image(grid_data_image, lethal_cost_threshold, min_frontier_size_pixels=min_frontier_size_pixels, binary_free_image=binary_free_image,
                                                    binary_lethal_image=binary_lethal_image, return_free_frontier=True)

    ## debug visualization
    # cv2.imshow("binary frontier image", binary_frontier_image*255)
    # cv2.waitKey(1)

    ## create binary image for frontier area only from reachable area
    if robot_pose is not None:
        reachable_costmap, _, _ = calc_reachable_map(costmap.costmap_info, grid_data_image, lethal_cost_threshold, robot_pose)
        binary_reachable_free_image = calc_binary_free_image(reachable_costmap, lethal_cost_threshold)
        binary_frontier_image = cv2.bitwise_and(binary_frontier_image, binary_reachable_free_image)

        ## debug visualization
        # cv2.imshow("binary reachable frontier image", binary_frontier_image*255)
        # cv2.waitKey(1)

    ## calculate dilation kernel for costmap inflation
    kernel = _calc_costmap_inflation_dilation_kernel(costmap.costmap_info, proximity_threshold)

    ## inflate frontier area
    dilate_binary_frontier_image = cv2.dilate(binary_frontier_image, kernel, iterations=1)

    ## debug visualization
    # cv2.imshow("dilate binary frontier image", dilate_binary_frontier_image*255)
    # cv2.waitKey(1)

    ## calculate free and inflate frontier area
    free_dilate_binary_frontier_image = cv2.bitwise_and(binary_free_image, dilate_binary_frontier_image)

    ## debug visualization
    # cv2.imshow("free dilate binary frontier image", free_dilate_binary_frontier_image*255)
    # cv2.waitKey(1)

    ## find free and inflate frontier area contours
    free_dilate_binary_frontier_contours, _ = cv2.findContours(free_dilate_binary_frontier_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    free_dilate_binary_frontier_contours_image = np.zeros(grid_data_image.shape, np.float32)
    for contour_idx, contour in enumerate(free_dilate_binary_frontier_contours):
        contour_id = contour_idx + 1
        cv2.drawContours(free_dilate_binary_frontier_contours_image, [contour], -1, color=contour_id, thickness=-1)
        cv2.drawContours(free_dilate_binary_frontier_contours_image, [contour], -1, color=contour_id, thickness=1)

    ## debug visualization
    # vis_free_dilate_binary_frontier_contours_image = np.zeros(grid_data_image.shape + (3,), dtype=np.uint8)
    # free_dilate_binary_frontier_contours_ids = np.unique(free_dilate_binary_frontier_contours_image)
    # for contour_id in free_dilate_binary_frontier_contours_ids:
    #     if contour_id>0:
    #         vis_free_dilate_binary_frontier_contours_image[free_dilate_binary_frontier_contours_image==contour_id] = [np.random.randint(0, 256), np.random.randint(0, 256), np.random.randint(0, 256)]
    # cv2.imshow("visualize free dilate binary frontier contour image", vis_free_dilate_binary_frontier_contours_image*255)
    # cv2.waitKey(1)

    ## store all IDs of frontier area
    frontier_area_id_set = set(free_dilate_binary_frontier_contours_image[binary_frontier_image==1])

    ## create binary frontier proximity image
    binary_frontier_proximity_image = np.zeros(grid_data_image.shape, np.float32)
    for contour_idx, contour in enumerate(free_dilate_binary_frontier_contours):
        contour_id = contour_idx + 1
        if contour_id in frontier_area_id_set:
            cv2.drawContours(binary_frontier_proximity_image, [contour], -1, color=1, thickness=-1)
            cv2.drawContours(binary_frontier_proximity_image, [contour], -1, color=1, thickness=1)

    ## debug visualization
    # cv2.imshow("binary frontier proximity image", binary_frontier_proximity_image*255)
    # cv2.waitKey(1)

    return Costmap(costmap.costmap_info, tuple(binary_frontier_proximity_image.flatten()))


def calc_binary_polyhedron_image(costmap_info, hull_polyhedron):
    """
    Calculate binary polyhedron area image

    Parameters
    ----------
    costmap_info : costmap_utils.CostmapInfo
        The cost map info
    hull_polyhedron : scipy.spatial.ConvexHull
        The coverage polyhedron for robot pose

    Returns
    -------
    binary_polyhedron_image : numpy.ndarray
        The binary polyhedron area image
    """
    ## create binary image for polyhedron area
    binary_polyhedron_image = np.zeros((costmap_info.height, costmap_info.width), np.uint8)

    ## fill polyhedron area
    hull_polyhedron_points = world_to_map_np_array(costmap_info, hull_polyhedron.points[hull_polyhedron.vertices])
    cv2.drawContours(binary_polyhedron_image, np.array([hull_polyhedron_points]), -1, color=1, thickness=-1)
    cv2.drawContours(binary_polyhedron_image, np.array([hull_polyhedron_points]), -1, color=1, thickness=1)

    ## debug visualization
    # cv2.imshow("binary polyhedron image", binary_polyhedron_image*255)
    # cv2.waitKey(1)

    return binary_polyhedron_image


def calc_binary_contours_map_image(costmap_info, contours_map):
    """
    Calculate binary contours area image from contours in map coordinate

    Parameters
    ----------
    costmap_info : costmap_utils.CostmapInfo
        The cost map info
    contours_map : array-like
        The input contours in map coordinate

    Returns
    -------
    binary_contours_image : numpy.ndarray
        The binary contours area image
    """
    ## create binary image for contour area
    binary_contours_image = np.zeros((costmap_info.height, costmap_info.width), np.uint8)
    for contour_map in contours_map:
        if cv2.contourArea(contour_map)>0:
            cv2.drawContours(binary_contours_image, [contour_map], -1, color=1, thickness=-1)
            cv2.drawContours(binary_contours_image, [contour_map], -1, color=1, thickness=1)

    ## debug visualization
    # cv2.imshow("binary contour areas image", binary_contours_image*255)
    # cv2.waitKey(1)

    return binary_contours_image


def calc_binary_contours_image(costmap_info, contours):
    """
    Calculate binary contours area image

    Parameters
    ----------
    costmap_info : costmap_utils.CostmapInfo
        The cost map info
    contours : array-like
        The input contours

    Returns
    -------
    binary_contours_image : numpy.ndarray
        The binary contours area image
    """
    contours_map = [world_to_map_np_array(costmap_info, contour) for contour in contours]
    return calc_binary_contours_map_image(costmap_info, contours_map)


def calc_binary_hierarchy_contours_map_image(costmap_info, contours_map, hierarchy):
    """
    Calculate binary contours area image from contours in map coordinate

    Parameters
    ----------
    costmap_info : costmap_utils.CostmapInfo
        The cost map info
    contours_map : array-like
        The input contours in map coordinate

    Returns
    -------
    binary_contours_image : numpy.ndarray
        The binary contours area image
    """
    ## create binary image for contour area
    binary_contours_image = np.zeros((costmap_info.height, costmap_info.width), np.uint8)
    for contour_idx, contour_map in enumerate(contours_map):
        if cv2.contourArea(contour_map)>0:
            # assuming costmap has at most 2 levels of hierarchy
            if hierarchy[0][contour_idx][3]==-1:
                # draw external contour
                cv2.drawContours(binary_contours_image, [contour_map], -1, color=1, thickness=-1)
                cv2.drawContours(binary_contours_image, [contour_map], -1, color=1, thickness=1)
            else:
                # draw hole area
                cv2.drawContours(binary_contours_image, [contour_map], -1, color=0, thickness=-1)
                cv2.drawContours(binary_contours_image, [contour_map], -1, color=0, thickness=1)

    ## debug visualization
    # cv2.imshow("binary contour areas image", binary_contours_image*255)
    # cv2.waitKey(1)

    return binary_contours_image


def calc_binary_hierarchy_contours_image(costmap_info, contours, hierarchy):
    """
    Calculate binary contours area image

    Parameters
    ----------
    costmap_info : costmap_utils.CostmapInfo
        The cost map info
    contours : array-like
        The input contours
    hierarchy : array-like
        The hierarchy of input contours

    Returns
    -------
    binary_contours_image : numpy.ndarray
        The binary contours area image
    """
    contours_map = [world_to_map_np_array(costmap_info, contour) for contour in contours]
    return calc_binary_hierarchy_contours_map_image(costmap_info, contours_map, hierarchy)


def calc_float_contours_map_image(costmap_info, contours_map):
    """
    Calculate float contours area image whose contour areas have ID values from contours in map coordinate

    Parameters
    ----------
    costmap_info : costmap_utils.CostmapInfo
        The cost map info
    contours_map : array-like
        The input contours in map coordinate

    Returns
    -------
    float_contours_image : numpy.ndarray
        The float contours area image whose contour areas have ID values
    """
    ## create float image for contour area
    float_contours_image = np.zeros((costmap_info.height, costmap_info.width), np.float32)
    for contour_idx, contour_map in enumerate(contours_map):
        if cv2.contourArea(contour_map)>0:
            contour_id = contour_idx + 1
            cv2.drawContours(float_contours_image, [contour_map], -1, color=contour_id, thickness=-1)
            cv2.drawContours(float_contours_image, [contour_map], -1, color=contour_id, thickness=1)

    ## debug visualization
    # vis_float_contours_image = np.zeros(float_contours_image.shape + (3,), dtype=np.uint8)
    # float_contours_ids = np.unique(float_contours_image)
    # for contour_id in float_contours_ids:
    #     if contour_id>0:
    #         vis_float_contours_image[float_contours_image==contour_id] = [np.random.randint(0, 256), np.random.randint(0, 256), np.random.randint(0, 256)]
    # cv2.imshow("float contour areas image", vis_float_contours_image*255)
    # cv2.waitKey(1)

    return float_contours_image


def calc_float_contours_image(costmap_info, contours):
    """
    Calculate float contours area image whose contour areas have ID values

    Parameters
    ----------
    costmap_info : costmap_utils.CostmapInfo
        The cost map info
    contours : array-like
        The input contours

    Returns
    -------
    float_contours_image : numpy.ndarray
        The float contours area image whose contour areas have ID values
    """
    contours_map = [world_to_map_np_array(costmap_info, contour) for contour in contours]
    return calc_float_contours_map_image(costmap_info, contours_map)


def calc_binary_points_image(costmap_info, points):
    """
    Calculate binary points image

    Parameters
    ----------
    costmap_info : costmap_utils.CostmapInfo
        The cost map info
    points : numpy.ndarray
        The input points

    Returns
    -------
    binary_points_image : numpy.ndarray
        The binary points image
    """
    ## create binary image for points
    binary_points_image = np.zeros((costmap_info.height, costmap_info.width), np.uint8)
    points_map = world_to_map_np_array(costmap_info, points)
    points_map = points_map[np.where((points_map[:,0]>=0) & (points_map[:,1]>=0) & (points_map[:,0]<costmap_info.width-1) & (points_map[:,1]<costmap_info.height-1))]
    binary_points_image[points_map[:,1], points_map[:,0]] = 1

    ## debug visualization
    # cv2.imshow("binary points image", binary_points_image*255)
    # cv2.waitKey(1)

    return binary_points_image


def calc_scan_points_in_world(laser_scan, transform):
    """
    Calculate laser scan points in world coordinate

    Parameters
    ----------
    laser_scan : sensor_msgs.msg.LaserScan
        The laser scan message
    transform : geometry_msgs.msg.Transform
        The transformation from Lidar to map

    Returns
    -------
    numpy.ndarray
        The laser scan points in world coordinate
    """
    # collect valid scan points
    scan_points = []
    angle = laser_scan.angle_min
    for range in laser_scan.ranges:
        if not math.isinf(range):
            if range>laser_scan.range_max:
                r = laser_scan.range_max
            elif range<laser_scan.range_min:
                r = laser_scan.range_min
            else:
                r = range
            x = r*math.cos(angle)
            y = r*math.sin(angle)
            scan_points.append([x, y])
        angle += laser_scan.angle_increment

    if len(scan_points)>0:
        transform_trans = (transform.translation.x, transform.translation.y, transform.translation.z)
        transform_rot = (transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)
        transform_mat = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(transform_trans), tf.transformations.quaternion_matrix(transform_rot))

        np_scan_points = np.array(scan_points)
        # add 0 values for z-coordinate
        np_scan_points = np.hstack([np_scan_points, np.zeros([np_scan_points.shape[0], 1], dtype=np.float32)])
        # add homogeneous coordinates
        np_scan_points = np.hstack([np_scan_points, np.ones([np_scan_points.shape[0], 1], dtype=np.float32)])
        # transform scan poinits to map coordinate
        transform_scan_points = transform_mat.dot(np.transpose(np_scan_points))
        # add x-y values in map coordinate
        return np.transpose(transform_scan_points[0:2,:])
    else:
        return np.array([])

