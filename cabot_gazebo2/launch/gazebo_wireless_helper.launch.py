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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import LogInfo
from launch.actions import TimerAction
from launch.event_handlers import OnExecutionComplete
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    wireless_config_file = LaunchConfiguration('wireless_config_file')
    beacons_topic = LaunchConfiguration('beacons_topic')
    wifi_topic = LaunchConfiguration('wifi_topic')
    verbose = LaunchConfiguration('verbose')
    
    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace',
            description='Namespace of wireless nodes'),

        DeclareLaunchArgument(
            'wireless_config_file',
            description='Wireless config file'),

        DeclareLaunchArgument(
            'beacons_topic',
            default_value='beacons',
            description='Topic of the beacons info'),

        DeclareLaunchArgument(
            'wifi_topic',
            default_value='wifi',
            description='Topic of the wifi info'),

        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            description='True if output more'),

        Node(
            package='cabot_gazebo',
            executable='wireless_rss_simulator_node.py',
            name='wireless_rss_simulator_node',
            namespace=namespace,
            output='screen',
            parameters=[{
                'wireless_config_file': wireless_config_file,
                'verbose': verbose
            }],
            remappings=[
                ("beacons", beacons_topic)
            ]
        ),

        Node(
            package='cabot_gazebo',
            executable='wireless_sample_simulator_node.py',
            name='wireless_sample_simulator_node',
            namespace=namespace,
            output='screen',
            parameters=[{
                'wireless_config_file': wireless_config_file,
                'verbose': verbose
            }],
            remappings=[
                ("wifi", wifi_topic)
            ]
        ),

        Node(
            package='cabot_gazebo',
            executable='floor_transition_node.py',
            name='floor_transition_node',
            output='screen',
            parameters=[{
                'wireless_config_file': wireless_config_file,
                'verbose': verbose
            }]
        ),

    ])

