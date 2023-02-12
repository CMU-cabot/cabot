# Copyright (c) 2022  Carnegie Mellon University
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition
from ament_index_python import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('cabot_ui')
    show_robot_monitor = LaunchConfiguration('show_robot_monitor')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument('show_robot_monitor', default_value='true'),
        DeclareLaunchArgument('config_file', default_value=PathJoinSubstitution([pkg_dir, 'config', 'cabot_diagnostic.yaml'])),

        Node(
            package="rqt_robot_monitor",
            executable="rqt_robot_monitor",
            name="rqt_robot_monitor",
            condition=IfCondition(show_robot_monitor)
        ),
        Node(
            package="diagnostic_aggregator",
            executable="aggregator_node",
            name="diagnostic_aggregator",
            parameters=[config_file]
        ),
    ])
