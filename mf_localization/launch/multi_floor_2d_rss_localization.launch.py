#!/usr/bin/env python3
# Copyright (c) 2021, 2022  IBM Corporation and Carnegie Mellon University

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

import os
from launch.logging import launch_config, _get_logging_directory
launch_config._log_dir = os.path.join(_get_logging_directory(), "mf_localization")

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
from launch_ros.actions import SetParametersFromFile

def generate_launch_description():
    pkg_dir = get_package_share_directory('mf_localization')
    robot = LaunchConfiguration('robot')
    rssi_offset = LaunchConfiguration('rssi_offset')

    map_config_file = LaunchConfiguration('map_config_file')
    beacons_topic = LaunchConfiguration('beacons_topic')
    wifi_topic = LaunchConfiguration('wifi_topic')
    with_odom_topic = LaunchConfiguration('with_odom_topic')

    points2_topic = LaunchConfiguration('points2_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    pressure_available = LaunchConfiguration('pressure_available')
    pressure_topic = LaunchConfiguration('pressure_topic')

    use_gnss = LaunchConfiguration('use_gnss')
    use_global_localizer = LaunchConfiguration('use_global_localizer')
    gnss_fix_topic = LaunchConfiguration('gnss_fix_topic')
    gnss_fix_velocity_topic = LaunchConfiguration('gnss_fix_velocity_topic')

    publish_current_rate = LaunchConfiguration('publish_current_rate')
    
    return LaunchDescription([
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),

        DeclareLaunchArgument('robot', default_value='', description='Robot type should be specified'),
        DeclareLaunchArgument('rssi_offset', default_value='0.0', description='Set RSSI offset to estimate location'),

        DeclareLaunchArgument('map_config_file', default_value='', description='Specify map config file path'),
        DeclareLaunchArgument('beacons_topic', default_value='beacons', description='Specify beacons topic name'),
        DeclareLaunchArgument('wifi_topic', default_value='wifi', description='Specify wifi topic name'),
        DeclareLaunchArgument('with_odom_topic', default_value='false', description='Weather odom topic is used to cartographer localization or not'),

        DeclareLaunchArgument('points2_topic', default_value='velodyne_points', description='Specify velodyne points topic name'),
        DeclareLaunchArgument('imu_topic', default_value='imu/data', description='Specify IMU topic name'),
        DeclareLaunchArgument('odom_topic', default_value='odom', description='Specify odometry topic name'),
        DeclareLaunchArgument('pressure_available', default_value='true', description='Weather pressure topic is available or not'),
        DeclareLaunchArgument('pressure_topic', default_value='pressure', description='Specify pressure topic name'),

        DeclareLaunchArgument('use_gnss', default_value='false', description='Weather GNSS functionality is used or not'),
        DeclareLaunchArgument('use_global_localizer', default_value='false', description='Weather use global localizer or not'),
        DeclareLaunchArgument('gnss_fix_topic', default_value='gnss_fix', description='Specify GNSS fix topic name'),
        DeclareLaunchArgument('gnss_fix_velocity_topic', default_value='gnss_fix_velocity', description='Specify GNSS fix velocity topicname'),

        DeclareLaunchArgument('publish_current_rate', default_value='0', description='Specify the rate of publishing current location'),

        Node(
            package='mf_localization',
            executable='ublox_converter.py',
            name='ublox_converter',
            parameters=[
                PathJoinSubstitution([pkg_dir, 'configuration_files', 'ublox', 'ublox_converter.yaml'])
            ],
            remappings=[
                ('navsat', 'ublox/navsat'),
                ('num_active_sv', 'ublox_converter/num_active_sv'),
                ('sv_status', 'ublox_converter/sv_status'),
            ],
        ),

        GroupAction([
            SetParametersFromFile(
                PathJoinSubstitution([pkg_dir, 'configuration_files', 'multi_floor', 'multi_floor_manager_with_odom.yaml']),
                condition=IfCondition(with_odom_topic),
            ),
            SetParametersFromFile(
                PathJoinSubstitution([pkg_dir, 'configuration_files', 'multi_floor', 'multi_floor_manager.yaml']),
                condition=UnlessCondition(with_odom_topic),
            ),
            Node(
                package='mf_localization',
                executable='multi_floor_manager.py',
                name='multi_floor_manager',
                parameters=[{
                    'map_config_file': map_config_file,
                    'configuration_directory': pkg_dir+'/configuration_files/cartographer',
                    'configuration_file_prefix': 'cartographer_2d',
                    'robot': robot,
                    'rssi_offset': rssi_offset,
                    'use_gnss': use_gnss,
                    'use_global_localizer': use_global_localizer,
                    'publish_current_rate': publish_current_rate,
                    'pressure_available': pressure_available,
                    'verbose': True,
                }],
                remappings=[
                    ('beacons', beacons_topic),
                    ('wifi', wifi_topic),
                    ('points2', points2_topic),
                    ('imu', imu_topic),
                    ('odom', odom_topic),
                    ('pressure', pressure_topic),
                    ('gnss_fix', gnss_fix_topic),
                    ('gnss_fix_velocity', gnss_fix_velocity_topic),
                ],
                # prefix='python3 -m cProfile -o multi_floor_manager.profile',
            ),
        ]),
    ])
