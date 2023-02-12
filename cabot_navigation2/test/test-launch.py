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
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.logging import launch_config

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('cabot_navigation2')

    # Create the launch configuration variables
    params_file = LaunchConfiguration('params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    param_substitutions = {
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        #        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    return LaunchDescription([
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),

        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(pkg_dir, 'rviz', '/home/developer/ros2_ws/src/cabot_navigation2/test/test.rviz'),
            description='Full path to the RVIZ config file to use'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'params', '/home/developer/ros2_ws/src/cabot_navigation2/test/test.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        # default navigator
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_footprint_tf',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_footprint'],
            parameters=[configured_params],
            remappings=remappings
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='log')
    ])
