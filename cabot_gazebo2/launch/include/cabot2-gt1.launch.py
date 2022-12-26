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
import pathlib
import tempfile
import traceback
import xml.dom.minidom
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, Substitution
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import LogInfo
from launch.actions import TimerAction
from launch.event_handlers import OnExecutionComplete
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import SetParameter, SetParametersFromFile
from launch_ros.descriptions import ParameterValue
from launch_ros.descriptions import ComposableNode
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

def generate_launch_description():
    use_sim_time = True
    use_composition = LaunchConfiguration('use_composition')

    param_file = pathlib.Path(get_package_share_directory('cabot_gazebo')) / 'params' / 'cabot2-gt1.yaml'

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_composition',
            default_value='true',
            description='use composition or not'
        ),

        GroupAction(
            condition=IfCondition(use_composition),
            actions=[
                ComposableNodeContainer(
                    name='laser_transform_container',
                    namespace='',
                    package='rclcpp_components',
                    executable='component_container',
                ),
                
                LoadComposableNodes(
                    condition=IfCondition(use_composition),
                    target_container='/laser_transform_container',
                    composable_node_descriptions=[
                        ComposableNode(
                            package='pointcloud_to_laserscan',
                            plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
                            namespace='',
                            name='pointcloud_to_laserscan',
                            parameters=[{'use_sim_time': use_sim_time}, param_file],
                            remappings=[
                                ('/cloud_in', '/velodyne_points')
                            ]
                        ),
                        ComposableNode(
                            package='pcl_ros',
                            plugin='pcl_ros::CropBox',
                            namespace='',
                            name='filter_crop_box_node',
                            parameters=[{'use_sim_time': use_sim_time}, param_file],
                            remappings=[
                                ('/input',  '/velodyne_points'),
                                ('/output', '/velodyne_points_cropped')
                            ]
                        ),
                    ]
                ),
            ]
        ),

        GroupAction(
            condition=UnlessCondition(use_composition),
            actions=[
                Node(
                    package='pointcloud_to_laserscan',
                    executable='pointcloud_to_laserscan_node',
                    name='pointcloud_to_laserscan',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}, param_file],
                    remappings=[
                        ('/cloud_in', '/velodyne_points')
                    ]
                ),

                Node(
                    package='pcl_ros',
                    executable='filter_crop_box_node',
                    name='filter_crop_box_node',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}, param_file],
                    remappings=[
                        ('/input',  '/velodyne_points'),
                        ('/output', '/velodyne_points_cropped')
                    ]
                ),
            ]
        )
    ])
