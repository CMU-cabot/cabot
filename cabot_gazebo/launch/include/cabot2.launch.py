;{p[;oi# Copyright (c) 2022  Carnegie Mellon University
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
    
    offset = LaunchConfiguration('offset')
    output = LaunchConfiguration('output')
    enable_touch = LaunchConfiguration('enable_touch')
    touch_params = LaunchConfiguration('touch_params')
    use_arduino = LaunchConfiguration('use_arduino')
    use_speedlimit = LaunchConfiguration('use_speedlimit')


    param_file = pathlib.Path(get_package_share_directory('cabot_gazebo')) / 'params' / 'cabot2-gt1.yaml'

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_composition',
            default_value='true',
            description='use composition or not'
        ),
        DeclareLaunchArgument(
            'offset',
            default_value='0.0',
            description='offset value'
        ),
        DeclareLaunchArgument(
            'output',
            default_value='both',
            description='Log output to both/screen/log'
        ),
        DeclareLaunchArgument(
            'enable_touch',
            default_value='false',
            description='Weather touch speed control is used or not'
        ),
        DeclareLaunchArgument(
            'touch_params',
            default_value='[128,48,24]',
            description='Touch parameters'
        ),
        DeclareLaunchArgument(
            'use_arduino',
            default_value='true',
            description='Whether arduino (microcontroller) is used or not'
        ),
        DeclareLaunchArgument(
            'use_speedlimit',
            default_value='true',
            description='Whether speed control is used or not (false for mapping)'
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
        ),


            # CaBot related
            Node(
                package='cabot',
                executable='cabot_handle_v2_node.py',
                namespace='/cabot',
                name='cabot_handle_v2_node',
                output=output,
                parameters=[*param_files],
            ),

            # Visualize the current speed on Rviz-
            Node(
                package='cabot',
                executable='speed_visualize_node',
                namespace='/cabot',
                name='speed_visualize_node',
                output=output,
                parameters=[*param_files],
            ),

            # Microcontroller (Arduino - gt1/gtm or ESP32 - ace)
            Node(
                package='cabot',
                executable='cabot_serial.py',
                namespace='/cabot',
                name='cabot_serial',
                output=output,
                parameters=[*param_files, {touch_params: touch_params}],
                remappings=[
                    ('/cabot/imu', '/cabot/imu/data'),
                    ('/cabot/touch_speed', '/cabot/touch_speed_raw')
                ],
            ),

            # optional wifi scanner with ESP32
            Node(
                package='cabot',
                executable='cabot_serial.py',
                namespace='/cabot',
                name='serial_esp32_wifi_scanner',
                output=output,
                parameters=[*param_files],
                remappings=[('wifi_scan_str', '/esp32/wifi_scan_str')],
                condition=IfCondition(use_standalone_wifi_scanner),
            ),

            # Costmap clearing issue hacking
            # Some obstacle points in costmap can be laid between the line of sight of lasers.
            # This requires the robot to move to clear those points. Usually this problem is
            # dealed with the rotating recovery behavior[1] in default recovery behaviors,
            # but this behavior is removed for CaBot because rotation is annoying for the user.
            # [1] https://github.com/ros-planning/navigation/tree/kinetic-devel/rotate_recovery
            # So, this node randomly rotate the laser in range of a laser scan step
            # (360/1440 degree) by changing hokuyo_link tf to remove obstacle points between
            # two laser scans.
            Node(  # TODO use component
                package='cabot',
                executable='clearing_tf_node',
                namespace='/cabot',
                name='clearing_tf_node',
                output=output,
                parameters=[*param_files],
            ),

            # The diagram of Cabot Odometry Adapter & related nodes (*components)
            # move_base's cmd_vel commands will be filtered through nodes to transform
            # command for the robot rotation center to the actual robot center.
            # Motor status will be used for calculating raw odometry of the robot
            # and will be merged by Robot Localization node to get stabilized
            # odometry. Odom adapter will convert the raw odometry to the odometry
            # of the robot rotating center which is controlled by offset.
            #
            #                                                       /cabot/cmd_vel_limited
            # +================+ /cmd_vel    +===================+              +===============+
            # |                |============>| * SpeedControl    |=============>| *             |
            # | Controller     |             +===================+              | OdomAdapter   |
            # |                |<===============================================|               |
            # +================+ /odom                                          +===============+
            #                                                                      A          |
            #                                                                      |          | /cabot/cmd_vel  # noqa: E501

            #                   /cabot/imu/data                                    |          |
            # +================+             +===================+  /cabot/odometry/filtered  |
            # |*Cabot Sensor   |============>| *                 |=================+          |
            # |================|             | RobotLocalization |                            |
            #                                |                   |<==================+        |
            #                                +===================+                   |        |
            #                                                        /cabot/odom_raw |        |
            #                                                                        |        |
            #                                                   /cabot/motorStatus   |        v
            # +================+             +==================+               +===============+
            # |                |============>|                  |==============>| *             |
            # | Motor          |   Serial    | MotorControl     |               | MotorAdapter  |
            # |                |<============|                  |<==============|               |
            # +================+             +==================+               +===============+
            #                                                   /cabot/motorTarget

            Node(
                package='cabot',
                executable='odom_adapter_node',
                namespace='/cabot',
                name='odom_adapter_node',
                output=output,
                parameters=[*param_files, {max_speed: max_speed}],
            ),
            # for local odom navigation
            Node(
                package='cabot',
                executable='odom_adapter_node',
                namespace='/cabot',
                name='odom_adapter_node2',
                output=output,
                parameters=[*param_files],
            ),
            # Cabot Lidar Speed Control
            Node(
                package='cabot',
                executable='lidar_speed_control_node',
                namespace='/cabot',
                name='lidar_speed_condro_node',
                output=output,
                parameters=[*param_files],
            ),
            # Cabot People SPeed Control
            Node(
                package='cabot',
                executable='people_speed_control_node',
                name='people_speed_control_node',
                output=output,
                parameters=[*param_files],
            ),
            # Cabot TF Speed Control
            Node(
                package='cabot',
                executable='tf_speed_control_node',
                namespace='/cabot',
                name='tf_speed_control_node',
                output=output,
                parameters=[*param_files],
            ),

            # Cabot Speed Control
            # This node limit the speed from the move_base based on specified topics
            #   /cabot/lidar_speed           - control by lidar sensor
            #   /cabot/map_speed             - control by map speed poi
            #   /cabot/people_speed          - control by surrounding people
            #   /cabot/queue_speed           - control by queue
            #   /cabot/tf_speed              - control by existence of specific tf
            #   /cabot/touch_speed_switched  - control by touch sensor
            #                  TODO use touch_enabled argument
            #   /cabot/user_speed            - control by user
            Node(
                package='cabot',
                executable='speed_control_node',
                namespace='/cabot',
                name=PythonExpression(['"speed_control_node_touch_', touch_enabled, '"']),
                output=output,
                parameters=[*param_files],
            ),

            # Motor Controller Adapter
            # Convert cmd_vel (linear, rotate) speed to motor target (left, right) speed.
            Node(
                package='motor_adapter',
                executable='odriver_adapter_node',
                namespace='/cabot',
                name='odriver_adapter_node',
                output=output,
                parameters=[*param_files],
                remappings=[
                    ('/imu', '/cabot/imu/data')
                ],
            ),

            # Motor Controller (ODrive)
            Node(
                package='odriver',
                executable='odriver_node',
                namespace='/cabot',
                name='odriver_node',
                output=output,
                parameters=[*param_files],
                remappings=[
                    ('/motorTarget', '/cabot/motorTarget'),
                    ('/motorStatus', '/cabot/motorStatus'),
                ],
            ),

            # Sensor fusion for stabilizing odometry
            Node(
                package='robot_localization',
                executable='ekf_node',
                namespace='/cabot',
                name='ekf_node',
                output=output,
                parameters=[*param_files],
            ),
            ],
            #condition=LaunchConfigurationNotEquals('model', '')
        
    ])
