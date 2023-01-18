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
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch.utilities import normalize_to_list_of_substitutions


def generate_launch_description():
    pkg_dir = get_package_share_directory('mf_localization_mapping')

    bag_filename = LaunchConfiguration('bag_filename')
    save_samples = LaunchConfiguration('save_samples')
    save_state = LaunchConfiguration('save_state')
    convert_points = LaunchConfiguration('convert_points')
    convert_imu = LaunchConfiguration('convert_imu')
    convert_esp32 = LaunchConfiguration('convert_esp32')
    robot = LaunchConfiguration('robot')
    wireless_topics = LaunchConfiguration('wireless_topics')
    rate = LaunchConfiguration('rate')
    start = LaunchConfiguration('start')
    load_state_filename = LaunchConfiguration('load_state_filename')
    delay = LaunchConfiguration('delay')

    scan = LaunchConfiguration('scan')
    imu = LaunchConfiguration('imu')
    points2 = LaunchConfiguration('points2')
    imu_temp = LaunchConfiguration('imu_temp')
    points2_temp = LaunchConfiguration('points2_temp')
    fix = LaunchConfiguration('fix')

    configuration_basename = LaunchConfiguration('configuration_basename')
    play_limited_topics = LaunchConfiguration('play_limited_topics')
    save_empty_beacon_sample = LaunchConfiguration('save_empty_beacon_sample')
    quit_when_rosbag_finish = LaunchConfiguration('quit_when_rosbag_finish')

    def configure_ros2_bag_play(context, node):
        cmd = node.cmd.copy()
        cmd.extend(['--clock', '--rate', rate])
        if delay.perform(context) == '-1':
            cmd.append('--pause')
        else:
            cmd.extend(['-d', delay])
        if float(start.perform(context)) > 0.0:
            cmd.extend(['--start-offset', start])
        if play_limited_topics.perform(context) == 'true':
            cmd.append('--topics')
            cmd.append('/velodyne_packets')
            cmd.append(['/', points2])
            cmd.append(['/', scan])
            cmd.append(['/', imu])
            cmd.append('/beacons')
            cmd.append('/wireless/beacons')
            cmd.append('/wireless/wifi')
        if convert_imu.perform(context) == 'true' or convert_points.perform(context) == 'true':
            cmd.append('--remap')
        if convert_imu.perform(context) == 'true':
            cmd.append([imu, ':=', imu_temp])
        if convert_points.perform(context) == 'true':
            cmd.append([points2, ':=', points2_temp])
        cmd.append([bag_filename])
        node.cmd.clear()
        # needs to be normalized
        node.cmd.extend([normalize_to_list_of_substitutions(x) for x in cmd])
        return [node]
    ros2_bag_play = ExecuteProcess(cmd=['ros2', 'bag', 'play'])

    return LaunchDescription([
        DeclareLaunchArgument('bag_filename'),
        DeclareLaunchArgument('save_samples', default_value='false'),
        DeclareLaunchArgument('save_state', default_value='false'),
        DeclareLaunchArgument('convert_points', default_value='false'),
        DeclareLaunchArgument('convert_imu', default_value='false'),
        DeclareLaunchArgument('convert_esp32', default_value='false'),
        DeclareLaunchArgument('robot', default_value='rover'),
        DeclareLaunchArgument('wireless_topics', default_value="['/wireless/beacons','/wireless/wifi','/beacons','/esp32/wifi']"),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('start', default_value='0'),
        DeclareLaunchArgument('load_state_filename', default_value=''),
        DeclareLaunchArgument('delay', default_value='-1'),

        DeclareLaunchArgument('scan', default_value='velodyne_scan'),
        DeclareLaunchArgument('imu', default_value='imu/data'),
        DeclareLaunchArgument('points2', default_value='velodyne_points'),
        DeclareLaunchArgument('imu_temp', default_value='imu_temp/data'),
        DeclareLaunchArgument('points2_temp', default_value='velodyne_points_temp'),
        DeclareLaunchArgument('fix', default_value='ublox/fix'),

        DeclareLaunchArgument('configuration_basename', default_value='cartographer_2d_mapping.lua'),
        DeclareLaunchArgument('start_trajectory_with_default_topics', default_value="$(eval load_state_filename=='')"),
        DeclareLaunchArgument('play_limited_topics', default_value='false'),
        DeclareLaunchArgument('save_empty_beacon_sample', default_value='true'),
        DeclareLaunchArgument('quit_when_rosbag_finish', default_value='false'),

        SetParameter('use_sim_time', 'true'),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(PathJoinSubstitution([pkg_dir, 'launch', 'cartographer_2d_VLP16.launch.py'])),
            launch_arguments={
                'robot': robot,
                'scan': scan,
                'points2': points2,
                'imu': imu,
                'fix': fix,
                'configuration_basename': configuration_basename,
                'load_state_filename': load_state_filename,
                'save_state_filename': PythonExpression(['"', bag_filename, '.pbstream" if "', save_state, '"=="true" else ""']),
                'start_trajectory_with_default_topics': PythonExpression(['"', load_state_filename, '"==""'])
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([get_package_share_directory('cartographer_ros'), 'configuration_files', 'demo_2d.rviz'])]
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(PathJoinSubstitution([pkg_dir, 'launch', 'includes', 'VLP16_points_cloud_nodelet_cartographer.launch.py'])),
            launch_arguments={
                'scan':  scan
            }.items(),
            condition=IfCondition(convert_points)
        ),

        Node(
            package='mf_localization',
            executable='imu_frame_renamer.py',
            name='imu_frame_renamer',
            condition=IfCondition(convert_imu),
            remappings=[
                ('imu_in', imu_temp),
                ('imu_out', imu)
            ]
        ),

        Node(
            package='wireless_scanner_ros',
            executable='esp32_wifi_scan_converter.py',
            name='esp32_wifi_scan_converter',
            condition=IfCondition(convert_esp32),
            remappings=[
                ('wireless/wifi', 'esp32/wifi')
            ]
        ),

        Node(
            package='mf_localization',
            executable='fix_filter.py',
            name='ublox_fix_filter',
            remappings=[
                ('fix', 'ublox/fix'),
                ('fix_filtered', 'ublox/fix_filtered')
            ]
        ),

        OpaqueFunction(
            function=configure_ros2_bag_play,
            args=[ros2_bag_play]
        ),

        Node(
            name="tf2_beacons_listener",
            package="mf_localization",
            executable="tf2_beacons_listener.py",
            parameters=[{
                'output': [bag_filename, '.loc.samples.json'],
                'topics': wireless_topics,
                'save_empty_beacon_sample': save_empty_beacon_sample
            }],
            condition=IfCondition(save_samples),
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=ros2_bag_play,
                on_exit=[
                    LogInfo(msg='ros2 bag play is completed'),
                    EmitEvent(event=Shutdown(reason='ros2 bag play is completed'))
                ]
            ),
            condition=IfCondition(quit_when_rosbag_finish)
        ),
    ])
