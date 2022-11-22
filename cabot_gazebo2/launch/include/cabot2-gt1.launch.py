# Copyright (c) 2022  Carnegie Mellon University
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

import os
import tempfile
import traceback
import xml.dom.minidom
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, Substitution
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import LogInfo
from launch.actions import TimerAction
from launch.event_handlers import OnExecutionComplete
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions


def generate_launch_description():
    use_sim_time = True

    return LaunchDescription([

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': 'velodyne',
                'transform_tolerance': 0.01,
                'min_height': -0.30,  # origin is the sensor
                'max_height': 1.4,  # origin is the sensor
                'angle_min': -2.57,  # -M_PI/2 - 1.0 (angle clipping)
                'angle_max': 1.57,  # M_PI/2
                'angle_increment': 0.00436,  # M_PI/360/2
                'scan_time': 0.1,
                'range_min': 0.2,
                'range_max': 50.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                # Concurrency level, affects number of pointclouds queued for
                # processing and number of threads used
                # 0 : Detect number of cores
                # 1 : Single threaded
                # 2->inf : Parallelism level
                'concurrency_level': 0                
            }],
            remappings=[
                ('/cloud_in', '/velodyne_points')
            ]
        ),

        Node(
            package='pcl_ros',
            executable='filter_crop_box_node',
            name='filter_crop_box_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'min_x': -0.7,
                'min_y': -0.6,
                'min_z': 0.0,
                'max_x': 0.2,
                'max_y': 0.1,
                'max_z': 2.0,
                'keep_organized': False,
                'negative': True,
                'input_frame': "base_link",
                'output_frame': "velodyne"
            }],
            remappings=[
                ('/input',  '/velodyne_points'),
                ('/output', '/velodyne_points_cropped')
            ]
        ),

    ])
