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

import datetime

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import SetLaunchConfiguration
from launch.actions import LogInfo
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationNotEquals
from launch.conditions import UnlessCondition
from launch.substitutions import AndSubstitution
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch.utilities import normalize_to_list_of_substitutions


def generate_launch_description():
    pkg_dir = get_package_share_directory('mf_localization_mapping')
    robot = LaunchConfiguration('robot')
    run_cartographer = LaunchConfiguration('run_cartographer')
    record_bag = LaunchConfiguration('record_bag')
    prefix = LaunchConfiguration('prefix')
    scan = LaunchConfiguration('scan')
    save_samples = LaunchConfiguration('save_samples')
    wireless_topics = LaunchConfiguration('wireless_topics')
    bag_filename = LaunchConfiguration('bag_filename')
    load_state_filename = LaunchConfiguration('load_state_filename')
    save_state_filename = LaunchConfiguration('save_state_filename')
    record_required = LaunchConfiguration('record_required')
    configuration_basename = LaunchConfiguration('configuration_basename')
    # save_state_filename # todo
    start_trajectory_with_default_topics = LaunchConfiguration('start_trajectory_with_default_topics')
    record_wireless = LaunchConfiguration('record_wireless')

    use_xsens = LaunchConfiguration('use_xsens')
    use_arduino = LaunchConfiguration('use_arduino')
    use_esp32 = LaunchConfiguration('use_esp32')
    use_velodyne = LaunchConfiguration('use_velodyne')
    use_sim_time = LaunchConfiguration('use_sim_time')
    imu_topic = LaunchConfiguration('imu_topic')

    def configure_ros2_bag_arguments(context, node):
        cmd = node.cmd.copy()
        if record_required.perform(context) == 'true':
            cmd.append('-a')
        else:
            cmd.extend(['-a', '-x', "'/map|/velodyne_points|(.*)/image_raw|(.*)/image_raw/(.*)'"])
        saved_location = []
        if bag_filename.perform(context) == '':
            saved_location = [EnvironmentVariable('HOME'), '/recordings/', prefix, datetime.datetime.now().strftime("_%Y_%m_%d-%H_%M_%S")]
            cmd.extend(['-o', saved_location])
        else:
            saved_location = [EnvironmentVariable('HOME'), '/recordings/', bag_filename]
            cmd.extend(['-o', saved_location])
        node.cmd.clear()
        node.cmd.extend([normalize_to_list_of_substitutions(x) for x in cmd])
        return [node, SetLaunchConfiguration('saved_location', ['./docker/home']+saved_location[1:])]

    ros2_bag_process = ExecuteProcess(
        cmd=["ros2", "bag", "record"]
    )
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='rover'),
        DeclareLaunchArgument('run_cartographer', default_value='true'),
        DeclareLaunchArgument('record_bag', default_value='true'),
        DeclareLaunchArgument('prefix', default_value='sensor'),
        DeclareLaunchArgument('scan', default_value='velodyne_scan'),
        DeclareLaunchArgument("save_samples", default_value="false"),
        DeclareLaunchArgument("wireless_topics", default_value="['/wireless/beacons','/wireless/wifi','/esp32/wifi']"),

        DeclareLaunchArgument("bag_filename", default_value=""),
        DeclareLaunchArgument("load_state_filename", default_value=""),
        DeclareLaunchArgument("record_required", default_value="false"),

        DeclareLaunchArgument("configuration_basename", default_value="cartographer_2d_mapping.lua"),
        DeclareLaunchArgument("save_state_filename", default_value=""),
        DeclareLaunchArgument("start_trajectory_with_default_topics", default_value="true"),
        SetLaunchConfiguration(name="start_trajectory_with_default_topics", value="false", condition=LaunchConfigurationNotEquals("load_state_filename", "")),

        DeclareLaunchArgument("record_wireless", default_value="false"),

        DeclareLaunchArgument("use_xsens", default_value="true"),
        DeclareLaunchArgument("use_arduino", default_value="false"),
        DeclareLaunchArgument("use_esp32", default_value="false"),
        DeclareLaunchArgument("use_velodyne", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("imu_topic", default_value="imu/data"),

        SetParameter('use_sim_time', use_sim_time),

        LogInfo(
            msg="use_arduino and use_xsens are both true",
            condition=IfCondition(AndSubstitution(use_xsens, use_arduino)),
        ),

        GroupAction([
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(PathJoinSubstitution([pkg_dir, 'launch', 'includes', 'xsens_driver_cartographer.launch.py'])),
                condition=IfCondition(use_xsens)
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(PathJoinSubstitution([pkg_dir, 'launch', 'includes', 'arduino_cartographer.launch.xml'])),
                condition=IfCondition(use_arduino)
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(PathJoinSubstitution([pkg_dir, 'launch', 'includes', 'esp32_cartographer.launch.xml'])),
                condition=IfCondition(use_esp32)
            ),

            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(PathJoinSubstitution([pkg_dir, 'launch', 'includes', 'VLP16_points_cartographer.launch.py'])),
                launch_arguments={
                    "scan": scan
                }.items(),
                condition=IfCondition(use_velodyne)
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(PathJoinSubstitution([get_package_share_directory('wireless_scanner_ros'), 'launch', 'wifi_ble_receiver.launch'])),
                condition=IfCondition(record_wireless)
            ),

            OpaqueFunction(
                function=configure_ros2_bag_arguments,
                args=[ros2_bag_process],
                condition=IfCondition(record_bag)
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_dir, 'launch', 'cartographer_2d_VLP16.launch.py'])),
                launch_arguments={
                    "robot": robot,
                    "scan": scan,
                    "configuration_basename": configuration_basename,
                    "load_state_filename": load_state_filename,
                    "save_state_filename": save_state_filename,
                    "start_trajectory_with_default_topics": start_trajectory_with_default_topics,
                    "imu": imu_topic
                }.items(),
                condition=IfCondition(run_cartographer)
            ),

            GroupAction([
                SetParameter(name="output", value=[bag_filename, ".loc.samples.json"], condition=LaunchConfigurationNotEquals('bag_filename', '')),
                Node(
                    package="mf_localization",
                    executable="tf2_beacons_listener.py",
                    name="tf2_beacons_listener",
                    parameters=[{
                        "topics": wireless_topics
                    }],
                    condition=IfCondition(save_samples)
                ),
            ]),

            Node(
                name="rviz2",
                package="rviz2",
                executable="rviz2",
                arguments=["-d", PathJoinSubstitution([pkg_dir, 'configuration_files', 'rviz', 'demo_2d.rviz'])],
                condition=IfCondition(run_cartographer)
            ),

            RegisterEventHandler(
                OnProcessExit(
                    target_action=ros2_bag_process,
                    on_exit=[
                        LogInfo(
                            msg=['Mapping data is saved at ', LaunchConfiguration('saved_location')]
                        )
                    ]
                )
            ),
        ], condition=UnlessCondition(AndSubstitution(use_xsens, use_arduino)))
    ])
