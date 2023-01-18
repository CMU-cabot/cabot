#!/usr/bin/env python3
# Copyright (c) 2023  Carnegie Mellon University

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

from pathlib import Path
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    frame_id = LaunchConfiguration('frame_id')
    manager = LaunchConfiguration('manager')

    calibration = LaunchConfiguration('calibration')
    max_range = LaunchConfiguration('max_range')
    min_range = LaunchConfiguration('min_range')

    laserscan_ring = LaunchConfiguration('laserscan_ring')
    laserscan_resolution = LaunchConfiguration('laserscan_resolution')
    scan = LaunchConfiguration('scan')

    return LaunchDescription([
        DeclareLaunchArgument('frame_id', default_value='velodyne'),
        DeclareLaunchArgument('manager', default_value=[frame_id, '_component_manager']),

        DeclareLaunchArgument('calibration', default_value=PathJoinSubstitution([get_package_share_directory('velodyne_pointcloud'), 'params', 'VLP16db.yaml'])),
        DeclareLaunchArgument('max_range', default_value='130.0'),
        DeclareLaunchArgument('min_range', default_value='0.1'),

        DeclareLaunchArgument('laserscan_ring', default_value='-1'),
        DeclareLaunchArgument('laserscan_resolution', default_value='0.007'),
        DeclareLaunchArgument('scan', default_value='/velodyne_scan'),


        ComposableNodeContainer(
            name=manager,
            package='rclcpp_components',
            namespace='',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='velodyne_pointcloud',
                    plugin='velodyne_pointcloud::Transform',
                    name='velodyne_transform_node',
                    parameters=[{
                        "calibration": calibration,
                        "max_range": max_range,
                        "min_range": min_range
                    }]
                ),
                ComposableNode(
                    package='velodyne_laserscan',
                    plugin='velodyne_laserscan::VelodyneLaserScan',
                    name='velodyne_laserscan_node',
                    parameters=[{
                        "ring": laserscan_ring,
                        "resolution": laserscan_resolution
                    }],
                    remappings=[
                        ('/scan', scan)
                    ]
                )
            ]
        ),
    ])


"""
<launch>

  <arg name="scan" default="/velodyne_scan" />
  <!-- declare arguments with default values -->
  <arg name="calibration" default="$(find velodyne_pointcloud)/params/VLP16db.yaml"/>
  <arg name="device_ip" default="" />
  <arg name="frame_id" default="velodyne" />
  <arg name="max_range" default="130.0" />
  <arg name="min_range" default="0.0" />
  <arg name="pcap" default="" />
  <arg name="port" default="2368" />
  <arg name="read_fast" default="false" />
  <arg name="read_once" default="false" />
  <arg name="repeat_delay" default="0.0" />
  <arg name="rpm" default="600.0" />
  <arg name="cut_angle" default="-0.01" />
  <arg name="laserscan_ring" default="-1" />
  <arg name="laserscan_resolution" default="0.007" />

  <!-- start nodelet manager and driver nodelets -->
  <include file="$(find velodyne_driver)/launch/nodelet_manager.launch">
    <arg name="device_ip" value="$(arg device_ip)"/>
    <arg name="frame_id" value="$(arg frame_id)"/>
    <arg name="manager" value="$(arg manager)" />
    <arg name="model" value="VLP16"/>
    <arg name="pcap" value="$(arg pcap)"/>
    <arg name="port" value="$(arg port)"/>
    <arg name="read_fast" value="$(arg read_fast)"/>
    <arg name="read_once" value="$(arg read_once)"/>
    <arg name="repeat_delay" value="$(arg repeat_delay)"/>
    <arg name="rpm" value="$(arg rpm)"/>
    <arg name="cut_angle" value="$(arg cut_angle)"/>
  </include>

  <!-- start cloud nodelet -->
  <include file="$(find velodyne_pointcloud)/launch/cloud_nodelet.launch">
    <arg name="calibration" value="$(arg calibration)"/>
    <arg name="manager" value="$(arg manager)" />
    <arg name="max_range" value="$(arg max_range)"/>
    <arg name="min_range" value="$(arg min_range)"/>
  </include>

  <!-- start laserscan nodelet -->
  <arg name="ring" value="$(arg laserscan_ring)"/>
  <arg name="resolution" value="$(arg laserscan_resolution)"/>
  <node pkg="nodelet" type="nodelet" name="$(arg manager)_laserscan"
        args="load velodyne_laserscan/LaserScanNodelet $(arg manager)">
    <param name="ring" value="$(arg laserscan_ring)"/>
    <param name="resolution" value="$(arg laserscan_resolution)"/>
    <remap from="/scan" to="$(arg scan)"/>
  </node>

</launch>
"""
