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
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch_ros.descriptions import ParameterValue
from launch.utilities import normalize_to_list_of_substitutions


def generate_launch_description():
    pkg_dir = get_package_share_directory('mf_localization_mapping')
    mf_localization_dir = get_package_share_directory('mf_localization')

    robot = LaunchConfiguration('robot')

    convert_points = LaunchConfiguration('convert_points')

    # multi-floor manager
    map_config_file = LaunchConfiguration('map_config_file')
    beacon_topic = LaunchConfiguration('beacon_topic')
    rssi_offset = LaunchConfiguration('rssi_offset')
    # cartographer
    scan = LaunchConfiguration('scan')
    points2 = LaunchConfiguration('points2')
    imu = LaunchConfiguration('imu')

    # rosbag
    playbag = LaunchConfiguration('playbag')
    bag_filename = LaunchConfiguration('bag_filename')
    rate = LaunchConfiguration('rate')
    start_time = LaunchConfiguration('start_time')

    record_bag = LaunchConfiguration('record_bag')
    record_file = LaunchConfiguration('record_file')

    # rviz
    # site = LaunchConfiguration('site')

    multi_floor_config_filename = LaunchConfiguration('multi_floor_config_filename')

    def configure_ros2_bag_play(context, node):
        cmd = node.cmd.copy()
        cmd.extend(['--clock', '--rate', rate])
        if float(start_time.perform(context)) > 0.0:
            cmd.extend(['--start-offset', start_time])
        cmd.extend(['--remap', 'imu/data:=imu/data'])
        cmd.extend(['--', bag_filename])
        node.cmd.clear()
        # needs to be normalized
        node.cmd.extend([normalize_to_list_of_substitutions(x) for x in cmd])
        return [node]

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='cabot2-gt1'),

        DeclareLaunchArgument('convert_points', default_value='true'),

        # multi-floor manager
        DeclareLaunchArgument('map_config_file'),
        DeclareLaunchArgument('beacon_topic', default_value='/wireless/beacons'),
        DeclareLaunchArgument('rssi_offset', default_value=""),
        DeclareLaunchArgument('multi_floor_config_filename', default_value='$(find mf_localization)/configuration_files/multi_floor/multi_floor_manager.yaml'),
        # cartographer
        DeclareLaunchArgument('scan', default_value='velodyne_scan'),
        DeclareLaunchArgument('points2', default_value='velodyne_points'),
        DeclareLaunchArgument('imu', default_value='imu/data'),

        # rosbag
        DeclareLaunchArgument('playbag', default_value='true'),
        DeclareLaunchArgument('bag_filename'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('start_time', default_value='0.0'),

        DeclareLaunchArgument('record_bag', default_value='true'),
        DeclareLaunchArgument('record_file', default_value=PythonExpression(['"', bag_filename, '"+".tested"'])),

        # rviz
        DeclareLaunchArgument('site', default_value=''),

        SetParameter('use_sim_time', True),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(PathJoinSubstitution([pkg_dir, 'launch', 'includes', 'VLP16_points_cloud_nodelet_cartographer.launch.py'])),
            launch_arguments={
                'scan': scan,
            }.items(),
            condition=IfCondition(convert_points),
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(PathJoinSubstitution([mf_localization_dir, 'launch', 'multi_floor_map_server.launch.xml'])),
            launch_arguments={
                'map_config_file': map_config_file,
            }.items(),
        ),

        # run multi_floor_manager
        GroupAction([
            SetParameter('robot', robot, condition=LaunchConfigurationNotEquals('robot', '')),
            SetParameter('rssi_offset', rssi_offset, condition=LaunchConfigurationNotEquals('rssi_offset', '')),
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
                        'verbose': True,
                    }
                ],
                remappings=[
                    ('beacons', beacon_topic),
                    ('points2', points2),
                    ('imu',  imu),
                    ('scan', scan),
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
            }]
        ),

        OpaqueFunction(
            function=configure_ros2_bag_play,
            args=[ExecuteProcess(
                cmd=['ros2', 'bag', 'play'],
                condition=IfCondition(playbag),
            )]
        ),

        ExecuteProcess(
            cmd=["ros2", "bag", "record", '-o', record_file,
                 '/wireless/beacons', '/current_floor_raw', '/current_floor', '/pressure_std', '/pressure'],
            condition=IfCondition(record_bag)
        ),

        ExecuteProcess(cmd=['ros2', 'topic', 'echo', '--csv', '/current_floor', 'std_msgs/Int64']),
        ExecuteProcess(cmd=['ros2', 'topic', 'echo', '--csv', '/current_floor_raw', 'std_msgs/Float64']),
        ExecuteProcess(cmd=['ros2', 'topic', 'echo', '--csv', '/pressure', 'sensor_msgs/FluidPressure']),
        ExecuteProcess(cmd=['ros2', 'topic', 'echo', '--csv', '/pressure_std', 'std_msgs/Float64']),
    ])
