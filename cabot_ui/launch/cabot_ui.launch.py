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
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

from cabot_common.launch import NamespaceParameterFile
from cabot_common.launch import GetPackageShareDirectory
from cabot_common.launch import AppendLogDirPrefix


def generate_launch_description():
    init_speed = LaunchConfiguration('init_speed')
    anchor_file = LaunchConfiguration('anchor_file')
    language = LaunchConfiguration('language')
    site = LaunchConfiguration('site')
    global_map_name = LaunchConfiguration('global_map_name')
    plan_topic = LaunchConfiguration('plan_topic')
    show_topology = LaunchConfiguration('show_topology')

    def hoge(text):
        return text

    config_path = PathJoinSubstitution([
        GetPackageShareDirectory(site),
        'config',
        'config.yaml'
    ])

    return LaunchDescription([
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        # append prefix name to the log directory for convenience
        RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("cabot_ui")])),
        DeclareLaunchArgument(
            'init_speed', default_value='1.0',
            description='Set the robot initial maximum speed. This will be capped by the max speed.'
        ),
        DeclareLaunchArgument(
            'anchor_file', default_value='',
            description='File path that includes anchor data'
        ),
        DeclareLaunchArgument(
            'language', default_value='en',
            description='Set the system language'
        ),
        DeclareLaunchArgument(
            'site', default_value='',
            description='CaBot site package name'
        ),
        DeclareLaunchArgument(
            'global_map_name', default_value='map_global',
            description='Set the global map name'
        ),
        DeclareLaunchArgument(
            'plan_topic', default_value='/plan',
            description='Topic name for planned path'
        ),
        DeclareLaunchArgument(
            'show_topology', default_value='false',
            description='Show topology on rviz'
        ),

        Node(
            package="cabot_ui",
            executable="cabot_ui_manager.py",
            name="cabot_ui_manager",
            parameters=[{
                'init_speed': init_speed,
                'anchor_file': anchor_file,
                'language': language,
                'global_map_name': global_map_name,
                'plan_topie': plan_topic,
            }, NamespaceParameterFile('cabot_ui_manager', config_path)],
            ros_arguments=[
                '--log-level', 'cabot_ui_manager:=debug'
            ],
            # prefix='python3 -m cProfile -o cabot_ui_manager.profile',
        ),
        Node(
            package='cabot_ui',
            executable='navcog_map.py',
            namespace='cabot',
            name='navcog_map',
            parameters=[{
                'anchor_file': anchor_file,
            }, NamespaceParameterFile('cabot/navcog_map', config_path)],
            condition=IfCondition(show_topology),
        ),
        # TODO
        # Node(
        #     package="cabot_ui",
        #     executable="stop_reasons_node.py",
        #     name="stop_reasons_node",
        #     parameters=[{
        #     }],
        # ),
    ])
