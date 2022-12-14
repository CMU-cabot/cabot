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

"""

"""

import os
import tempfile
import traceback
import xml.dom.minidom
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, Substitution
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import LogInfo
from launch.actions import TimerAction
from launch.event_handlers import OnExecutionComplete
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions


def generate_launch_description():
    pkg_dir = get_package_share_directory('cabot')

    config_for_cabot = PathJoinSubstitution([
        pkg_dir,
        'config',
        'velodyne',
        'vlp16.yaml'
    ])
    config_for_cabot_model = LaunchConfiguration('confing_for_cabot_model')

    vlp16_calibration_file = PathJoinSubstitution([
        get_package_share_directory('velodyne_pointcloud'),
        'params',
        'VLP16db.yaml'
    ])
    
    return LaunchDescription([

        LogInfo(msg=[config_for_cabot]),
        LogInfo(msg=[vlp16_calibration_file]),
        
        DeclareLaunchArgument(
            'config_for_cabot_model',
            default_value='',
            description='Config file path for the cabot model'
        ),

        ComposableNodeContainer(
            name='velodyne_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='velodyne_driver',
                    plugin='velodyne_driver::VelodyneDriver',
                    name='velodyne_driver_node',
                    parameters=[config_for_cabot]), #, config_for_cabot_model]),
                ComposableNode(
                    package='velodyne_pointcloud',
                    plugin='velodyne_pointcloud::Transform',
                    name='velodyne_transform_node',
                    parameters=[config_for_cabot, {'calibration':vlp16_calibration_file}]), #, config_for_cabot_model]),
                ComposableNode(
                    package='velodyne_laserscan',
                    plugin='velodyne_laserscan::VelodyneLaserScan',
                    name='velodyne_laserscan_node',
                    parameters=[config_for_cabot]), #, config_for_cabot_model]),
            ],
            output='both'
        )
    ])
