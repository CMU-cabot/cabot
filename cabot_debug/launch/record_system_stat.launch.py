# Copyright (c) 2025  Carnegie Mellon University
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
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from cabot_common.launch import AppendLogDirPrefix


def generate_launch_description():
    output = {'stderr': {'log'}}
    return LaunchDescription([
        DeclareLaunchArgument('sigterm_timeout', default_value='15'),
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("record_system_stat")])),
        Node(
            package='cabot_debug',
            executable='command_logger.py',
            name='top_node',
            output=output,
            parameters=[{'topic': '/top', 'command': 'top -bcd 1'}]
        ),
        # The following nodes were commented out in the XML:
        # Node(
        #     package='cabot_debug',
        #     executable='command_logger.py',
        #     name='lscpu_node',
        #     output=output,
        #     parameters=[{'topic': '/lscpu', 'command': 'lscpu', 'frequency': 1.0}]
        # ),
        # Node(
        #     package='cabot_debug',
        #     executable='command_logger.py',
        #     name='sensors_node',
        #     output=output,
        #     parameters=[{'topic': '/sensors', 'command': 'sensors', 'frequency': 1.0}]
        # ),
        Node(
            package='cabot_debug',
            executable='command_logger.py',
            name='nvidia_smi_node',
            output=output,
            parameters=[{'topic': '/nvidia_smi_dmon', 'command': 'nvidia-smi dmon'}]
        ),
        Node(
            package='cabot_debug',
            executable='command_logger.py',
            name='tegrastats_node',
            output=output,
            parameters=[{'topic': '/tegrastats', 'command': 'tegrastats'}]
        )
    ])
