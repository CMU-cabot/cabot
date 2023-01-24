#!/usr/bin/env python3
# Copyright (c) 2021, 2023  IBM Corporation and Carnegie Mellon University

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('mf_localization_mapping')
    scan = LaunchConfiguration('scan')
    no_rviz = LaunchConfiguration('no_rviz')
    bag_filenames = LaunchConfiguration('bag_filenames')

    return LaunchDescription([
        DeclareLaunchArgument('scan', default_value='velodyne_scan'),
        DeclareLaunchArgument('no_rviz', default_value='false'),
        DeclareLaunchArgument('bag_filenames'),

        SetParameter('use_sim_time', 'true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['cat ', pkg_dir, '/urdf/rover.urdf']),
                    value_type=str
                )
            }]
        ),

        Node(
            name="cartographer_offline_node",
            package="cartographer_ros",
            executable="cartographer_offline_node",
            arguments=[
                '-configuration_directory', [pkg_dir, '/configuration_files/cartographer/'],
                '-configuration_basenames', 'cartographer_2d_mapping.lua',
                '-urdf_filenames', [pkg_dir, '/urdf/rover.urdf'],
                '-bag_filenames', [bag_filenames],
            ],
            remappings=[
                ('/imu', 'imu/data'),
                ('/scan', scan),
                ('/points2', '/velodyne_points'),
            ]
        ),

        Node(
            name="rviz",
            package="rviz2",
            executable="rviz2",
            arguments=["-d", [pkg_dir, "/configuration_files/demo_2d.rviz"]],
            condition=UnlessCondition(no_rviz)
        ),
    ])
