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
Launch file for VLP16 and related node to filter point cloud
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ParameterFile, ComposableNode


def generate_launch_description():
    output = 'both'

    pkg_dir = get_package_share_directory('cabot')

    model_name = LaunchConfiguration('model')

    # Wrapping by ParameterFile to evaluate substitution in the yaml file
    vlp16_composable_config = ParameterFile(PathJoinSubstitution([
        pkg_dir,
        'config',
        'velodyne',
        'vlp16_composable.yaml'
    ]), allow_substs=True)

    config_cabot_common = ParameterFile(PathJoinSubstitution([
        pkg_dir,
        'config',
        'cabot2-common.yaml'
    ]), allow_substs=True)

    config_cabot_model = ParameterFile(PathJoinSubstitution([
        pkg_dir,
        'config',
        PythonExpression(['"', model_name, '.yaml', '"'])
    ]), allow_substs=True)

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='',
            description='CaBot model'
        ),

        # Kind error message
        LogInfo(
            msg=['You need to specify model parameter'],
            condition=LaunchConfigurationEquals('model', '')
        ),

        GroupAction([
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
                        parameters=[vlp16_composable_config],
                    ),
                    ComposableNode(
                        package='velodyne_pointcloud',
                        plugin='velodyne_pointcloud::Transform',
                        name='velodyne_transform_node',
                        parameters=[vlp16_composable_config],
                    ),
                    ComposableNode(
                        package='velodyne_laserscan',
                        plugin='velodyne_laserscan::VelodyneLaserScan',
                        name='velodyne_laserscan_node',
                        parameters=[vlp16_composable_config],
                        remappings=[('scan', 'scan1')],
                    ),
                    # TODO: want to use composable node
                    # does not work with galactic
                    # maybe because of remappings / lazy subscription
                    # ComposableNode(
                    #     package='pointcloud_to_laserscan',
                    #     plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
                    #     name='pointcloud_to_laserscan_node',
                    #     parameters=[vlp16_composable_config],
                    #     remappings=[('cloud_in', 'velodyne_points')],
                    # ),
                ],
                output=output
            ),

            # TODO: want to use composable node
            Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name='pointcloud_to_laserscan_node',
                output=output,
                parameters=[config_cabot_common, config_cabot_model],
                remappings=[
                    ('/cloud_in', '/velodyne_points')
                ],
            ),

            Node(
                package='pcl_ros',
                executable='filter_crop_box_node',
                name='filter_crop_box_node',
                output=output,
                parameters=[config_cabot_common, config_cabot_model],
                remappings=[
                    ('/input',  '/velodyne_points'),
                    ('/output', '/velodyne_points_cropped')
                ]
            ),
        ], condition=LaunchConfigurationNotEquals('model', '')
        ),
    ])
