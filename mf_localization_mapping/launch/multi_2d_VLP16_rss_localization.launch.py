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
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('mf_localization_mapping')
    mf_localization_dir = get_package_share_directory('mf_localization')

    map_config_file = LaunchConfiguration('map_config_file')
    beacon_topic = LaunchConfiguration('beacon_topic')
    show_rviz = LaunchConfiguration('show_rviz')

    # cartographer
    scan = LaunchConfiguration('scan')
    points2 = LaunchConfiguration('points2')
    imu = LaunchConfiguration('imu')

    robot = LaunchConfiguration('robot')

    # run multi_floor_manager
    multi_floor_config_filename = LaunchConfiguration('multi_floor_config_filename')

    return LaunchDescription([
        DeclareLaunchArgument('map_config_file'),
        DeclareLaunchArgument('beacon_topic', default_value='beacons'),
        DeclareLaunchArgument('show_rviz', default_value='true'),

        # cartographer
        DeclareLaunchArgument('scan', default_value='velodyne_scan'),
        DeclareLaunchArgument('points2', default_value='velodyne_points'),
        DeclareLaunchArgument('imu', default_value='imu/data'),

        DeclareLaunchArgument('robot', default_value='rover'),

        # run multi_floor_manager
        DeclareLaunchArgument('multi_floor_config_filename',
                              default_value=PathJoinSubstitution([mf_localization_dir, 'configuration_files', 'multi_floor', 'multi_floor_manager.yaml'])),

        # run multi_floor_manager
        GroupAction([
            Node(
                package='mf_localization',
                executable='multi_floor_manager.py',
                name='multi_floor_manager',
                parameters=[
                    multi_floor_config_filename,
                    {
                        'map_config_file': map_config_file,
                        'configuration_directory': PathJoinSubstitution([mf_localization_dir, 'configuration_files', 'cartographer']),
                        'configuration_file_prefix': 'cartographer_2d',
                    }
                ],
                remappings=[
                    ('scan', scan),
                    ('points2', points2),
                    ('imu',  imu),
                    ('beacons', beacon_topic),
                ],
            ),
        ]),

        # publish robot model
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['cat ', pkg_dir, '/urdf/', robot, '.urdf']),
                    value_type=str
                )
            }],
        ),

        # rviz
        Node(
            name='rviz2',
            package='rviz2',
            executable='rviz2',
            arguments=[
                '-d', PathJoinSubstitution([pkg_dir, 'configuration_files', 'rviz', 'demo_2d_floors.rviz'])
            ],
            condition=IfCondition(show_rviz),
        ),

    ])
