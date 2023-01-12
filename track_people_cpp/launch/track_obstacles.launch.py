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

from launch.logging import launch_config

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node

from cabot_common.launch import AppendLogDirPrefix


def generate_launch_description():
    return LaunchDescription([
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        # append prefix name to the log directory for convenience
        RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("track_people_cpp-track_obstacles")])),

        Node(
            package='track_people_cpp',
            executable='detect_obstacle_on_path_node',
            name='detect_obstacle_on_path',
            namespace='obstacle',
        ),

        Node(
            package='track_people_py',
            executable='track_sort_3d_people.py',
            name='track_obstacle',
            namespace='obstacle',
            parameters=[{
                'target_fps': 15.0,
                'diagnostic_name': 'ObstacleTrack'
            }]
        ),

        Node(
            package="track_people_py",
            executable="predict_kf_obstacle.py",
            name="predict_obstacle",
            namespace='obstacle',
            parameters=[{
                'stationary_detect_threshold_duration': 1.0,
                'diagnostic_name': 'ObstaclePredict'
            }],
            remappings=[('/obstacle/people', '/obstacles')],
        ),
    ])
