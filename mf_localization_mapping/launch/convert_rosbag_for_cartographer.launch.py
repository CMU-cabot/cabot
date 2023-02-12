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
from launch.actions import EmitEvent
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import Shutdown as ShutdownAction
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.utilities import normalize_to_list_of_substitutions


def generate_launch_description():
    pkg_dir = get_package_share_directory('mf_localization_mapping')

    bag_filename = LaunchConfiguration('bag_filename')
    save_filename = LaunchConfiguration('save_filename')
    scan = LaunchConfiguration('scan')
    rate = LaunchConfiguration('rate')
    convert_points = LaunchConfiguration('convert_points')
    convert_imu = LaunchConfiguration('convert_imu')
    convert_esp32 = LaunchConfiguration('convert_esp32')
    # compress = LaunchConfiguration('compress')
    start = LaunchConfiguration('start')

    def configure_ros2_bag_play_arguments(context, node):
        cmd = node.cmd.copy()
        if float(start.perform(context)) > 0.0:
            cmd.extend(['--start-offset', start])
        if convert_points.perform(context) == 'true' or convert_imu.perform(context) == 'true':
            cmd.append('--remap')
        if convert_points.perform(context) == 'true':
            cmd.append('velodyne_points:=velodyne_points_temp')
            cmd.append([scan, ":=", scan, '_temp'])
        if convert_imu.perform(context) == 'true':
            cmd.append('imu/data:=imu/data_temp')
        cmd.extend(['--', bag_filename])
        node.cmd.clear()
        # needs to be normalized
        node.cmd.extend([normalize_to_list_of_substitutions(x) for x in cmd])
        return [node]

    ros2_bag_play = ExecuteProcess(cmd=['ros2', 'bag', 'play', '--rate', rate, '--clock', '--delay', '5.0'])

    return LaunchDescription([
        DeclareLaunchArgument('bag_filename'),
        DeclareLaunchArgument('save_filename', default_value=[bag_filename, '.carto-converted']),
        DeclareLaunchArgument('scan', default_value='velodyne_scan'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('convert_points', default_value='false'),
        DeclareLaunchArgument('convert_imu', default_value='false'),
        DeclareLaunchArgument('convert_esp32', default_value='false'),
        # DeclareLaunchArgument('conpress', default_value='false'),
        DeclareLaunchArgument('start', default_value='0.0'),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(PathJoinSubstitution([pkg_dir, 'launch', 'includes', 'VLP16_points_cloud_nodelet_cartographer.launch.py'])),
            launch_arguments={
                "use_sim_time": 'true',
                "scan": scan
            }.items(),
            condition=IfCondition(convert_points)
        ),

        Node(
            package='mf_localization',
            executable='imu_frame_renamaer.py',
            name='imu_frame_renamer',
            condition=IfCondition(convert_imu),
            parameters=[{'use_sim_time': 'true'}],
            remappings=[('imu_in', 'imu/data_temp'), ('imu_out', 'imu/data')],
        ),

        Node(
            package='wireless_scanner_ros',
            executable='esp32_wifi_scan_converter.py',
            name='esp32_wifi_scan_converter',
            condition=IfCondition(convert_esp32),
            parameters=[{'use_sim_time': 'true'}],
            remappings=[('wireless/wifi', 'esp32/wifi')]
        ),

        OpaqueFunction(
            function=configure_ros2_bag_play_arguments,
            args=[ros2_bag_play]
        ),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '/imu/data', '/velodyne_scan',  '/velodyne_points',  '/beacons',  '/wireless/beacons',  '/wireless/wifi',  '/esp32/wifi', '-o', save_filename],
            on_exit=ShutdownAction(),
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=ros2_bag_play,
                on_exit=[
                    LogInfo(msg='ros2 bag play is completed'),
                    EmitEvent(event=Shutdown(reason='ros2 bag play is completed'))
                ]
            )
        ),
    ])
