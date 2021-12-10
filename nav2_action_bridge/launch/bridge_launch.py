# Copyright (c) 2020  Carnegie Mellon University
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.logging import launch_config

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Create our own temporary YAML files that include substitutions
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='ros1_bridge',
            executable='parameter_bridge',
            name='parameter_bridge',
            arguments=['topics', 'services_1_to_2', 'services_2_to_1'],
            output='log'
        ),
        
        Node(
            package='nav2_action_bridge',
            executable='nav2_navigate_to_pose_bridge_node',
            name='nav2_navigate_to_pose_bridge_node',
            arguments=['navigate_to_pose'],
            output='log'
        ),

        Node(
            package='nav2_action_bridge',
            executable='nav2_navigate_to_pose_bridge_node',
            arguments=['local/navigate_to_pose'],
            name='nav2_navigate_to_pose_bridge_node',
            output='log'
        ),

        Node(
            package='nav2_action_bridge',
            executable='nav2_navigate_through_poses_bridge_node',
            arguments=['navigate_through_poses'],
            name='nav2_navigate_through_poses_bridge_node',
            output='log'
        ),

        Node(
            package='nav2_action_bridge',
            executable='nav2_navigate_through_poses_bridge_node',
            arguments=['local/navigate_through_poses'],
            name='nav2_navigate_through_poses_bridge_node',
            output='log'
        ),

        Node(
            package='nav2_action_bridge',
            executable='nav2_spin_bridge_node',
            name='nav2_spin_bridge_node',
            output='log'
        ),

        ])
