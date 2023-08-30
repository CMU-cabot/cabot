#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2023  Carnegie Mellon University and IBM Corporation
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

"""
Launch file for hesai lidar
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    pkg_dir = get_package_share_directory('cabot')

    model_name = LaunchConfiguration('model')
    pandar = LaunchConfiguration('pandar')
    pandar_packets = LaunchConfiguration('pandar_packets')
    output = LaunchConfiguration('output')

    param_files = [
        ParameterFile(PathJoinSubstitution([
            pkg_dir,
            'config',
            'cabot2-common.yaml'
        ]),
            allow_substs=True,
        ),
        ParameterFile(PathJoinSubstitution([
            pkg_dir,
            'config',
            PythonExpression(['"', model_name, '.yaml"'])
        ]),
            allow_substs=True,
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=EnvironmentVariable('CABOT_MODEL'),
            description='CaBot model'
        ),
        DeclareLaunchArgument(
            'pandar',
            default_value='pandar',
            description='Published PointCloud2 topic'
        ),
        DeclareLaunchArgument(
            'pandar_packets',
            default_value='pandar_packets',
            description='Published PandarScan topic'
        ),
        DeclareLaunchArgument(
            'output',
            default_value='both',
            description='hesai_lidar node output'
        ),

        Node(
            package='hesai_lidar',
            namespace='',
            executable='hesai_lidar_node',
            name='hesai_lidar',
            output=output,
            parameters=[
                ParameterFile(PathJoinSubstitution([
                    pkg_dir, 'config', 'hesai', 'hesai_lidar.yaml'
                ]), allow_substs=True),
                *param_files
            ],
            remappings=[
                ('pandar', pandar),
                ('pandar_packets', pandar_packets)
            ]
        )
    ])
