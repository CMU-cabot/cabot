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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ParameterFile
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_dir = get_package_share_directory('cabot')

    target_container = LaunchConfiguration('target_container')

    # Wrapping by ParameterFile to evaluate substitution in the yaml file
    vlp16_composable_config = ParameterFile(PathJoinSubstitution([
        pkg_dir,
        'config',
        'velodyne',
        'vlp16_composable.yaml'
    ]), allow_substs=True)

    return LaunchDescription([
        DeclareLaunchArgument(
            'target_container',
            default_value='vlp16_container',
            description='The name of composable container'
        ),

        LoadComposableNodes(
            target_container=target_container,
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
            ],
        ),
    ])
