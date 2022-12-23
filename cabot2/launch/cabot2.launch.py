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
Launch file for all CaBot2

change from ROS1: each model had own launch file in ROS1, but ROS2 launch will handle all models.
  differences are managed by parameter file `<model_name>.yaml`

- Known Model
  - cabot2-gt1   (AIS-2020)
  - cabot2-gtm   (AIS-2021, Miraikan)
  - cabot2-ace   (AIS-2022, Consortium)
  - cabot2-gtmx  (AIS-2021 + Outside, Miraikan)
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import Command
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.descriptions import ParameterFile
# from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    output = 'both'
    pkg_dir = get_package_share_directory('cabot')

    model_name = LaunchConfiguration('model')  # need to be set
    touch_params = LaunchConfiguration('touch_params')  # TODO no default value
    touch_enabled = LaunchConfiguration('touch_enabled')
    use_standalone_wifi_scanner = LaunchConfiguration('use_standalone_wifi_scanner')
    max_speed = LaunchConfiguration('max_speed')

    xacro_for_cabot_model = PathJoinSubstitution([
        get_package_share_directory('cabot_description'),
        'robots',
        PythonExpression(['"', model_name, '.urdf.xacro', '"'])
    ])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_for_cabot_model]),
        value_type=str
    )

    param_files = [
        ParameterFile(PathJoinSubstitution([
                pkg_dir,
                'config',
                'cabot2-common.yaml'
            ]),
            allow_substs=True,
        ),
        ParameterFile(PathJoinSubstitution([
                pkg_dir,
                'config',
                PythonExpression(['"', model_name, '.yaml"'])
            ]),
            allow_substs=True,
        ),
    ]

    # deprecated parameters
    # - offset
    # - no_vibration
    # - output
    # - use_velodyne
    # - use_tf_static

    return LaunchDescription([
        # Kind error message
        LogInfo(
            msg=['You need to specify model parameter'],
            condition=LaunchConfigurationEquals('model', ''),
        ),

        GroupAction([
            DeclareLaunchArgument(
                'model',
                default_value=EnvironmentVariable('CABOT_MODEL'),
                description='CaBot model'
            ),
            DeclareLaunchArgument(
                'touch_params',
                default_value=EnvironmentVariable('CABOT_TOUCH_PARAMS'),
                description='An array of three values for touch detection, like [128, 48, 24]'
            ),
            DeclareLaunchArgument(
                'touch_enabled',
                default_value=EnvironmentVariable('CABOT_TOUCH_ENABLED', default_value='true'),
                description='If true, the touch sensor on the handle is used to control speed'
            ),
            DeclareLaunchArgument(
                'use_standalone_wifi_scanner',
                default_value=EnvironmentVariable('CABOT_STANDALONE_WIFI_SCANNER', default_value='false'),
                description='If true, launch stand alone wifi scanner with ESP32'
                            ' (only for GT/GTM with Arduino), ace can scan wifi by builtin ESP32'
            ),
            DeclareLaunchArgument(
                'max_speed',
                default_value=EnvironmentVariable('CABOT_MAX_SPEED', default_value='1.0'),
                description='Set maximum speed of the robot'
            ),

            # publish robot state
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output=output,
                parameters=[{
                    'publish_frequency': 20.0,
                    'robot_description': robot_description
                }]
            ),
            # publish **local** robot state for local map navigation (getting off elevators)
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='local_robot_state_publisher',
                output=output,
                parameters=[{
                    'publish_frequency': 20.0,
                    'frame_prefix': 'local/',
                    'robot_description': robot_description
                }]
            ),

            # launch velodyne lider related nodes
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        pkg_dir, 'launch', 'include', 'vlp16.launch.py'
                    ])
                ]),
                launch_arguments={
                    'model': model_name
                }.items()
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
        ),
    ])
