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
    bagfile = LaunchConfiguration('bagfile')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    param_substitutions = {
        'bagfile': bagfile
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        #        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    return LaunchDescription([
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'test', 'test.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        DeclareLaunchArgument(
            'bagfile',
            default_value='',
            description='Full path to the ROS2 bag for debug'),

        DeclareLaunchArgument(
            'prepare',
            default_value='true',
            description='launch main if false, rviz2 and tf if true'),

        # default navigator
        Node(
            package='cabot_navigation2',
            executable='cabot_lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['cabot_planner_test']}
                        ]),
        Node(
            package='cabot_navigation2',
            executable='cabot_planner_test',
            name='cabot_planner_test',
            output='screen',
            parameters=[configured_params],
            # arguments=["--ros-args", "--log-level", "cabot_planner_test:=debug"],
            remappings=remappings)
    ])
