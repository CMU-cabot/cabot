#  Copyright (c) 2023  Carnegie Mellon University
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
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.actions import SetParameter

from cabot_common.launch import AppendLogDirPrefix


def generate_launch_description():
    target_fps = LaunchConfiguration('target_fps')
    publish_simulator_people = LaunchConfiguration('publish_simulator_people')

    # ToDo: workaround https://github.com/CMU-cabot/cabot/issues/86
    jetpack5_workaround = LaunchConfiguration('jetpack5_workaround')

    return LaunchDescription([
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),

        # append prefix name to the log directory for convenience
        RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("track_people_py-predict_kf")])),

        DeclareLaunchArgument('target_fps', default_value=EnvironmentVariable('CABOT_PEOPLE_PREDICT_FPS', default_value='15.0')),
        DeclareLaunchArgument('publish_simulator_people', default_value='false'),

        DeclareLaunchArgument('jetpack5_workaround', default_value='false'),

        # overwrite parameters
        SetParameter(name='target_fps', value=target_fps),
        SetParameter(name='publish_simulator_people', value=publish_simulator_people),

        SetEnvironmentVariable(name='LD_PRELOAD', value='/usr/local/lib/libOpen3D.so', condition=IfCondition(jetpack5_workaround)),
        Node(
            package="track_people_py",
            executable="predict_kf_people.py",
            name="predict_kf_people_py",
        ),
    ])
