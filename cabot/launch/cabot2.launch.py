# Copyright (c) 2022, 2023  Carnegie Mellon University
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
from launch.logging import launch_config

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.event_handlers import OnShutdown
from launch.substitutions import Command
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.descriptions import ParameterValue
from launch_ros.descriptions import ParameterFile

from cabot_common.launch import AppendLogDirPrefix


def generate_launch_description():
    output = 'both'
    pkg_dir = get_package_share_directory('cabot')

    use_sim_time = LaunchConfiguration('use_sim_time')
    model_name = LaunchConfiguration('model')  # need to be set
    touch_params = LaunchConfiguration('touch_params')  # TODO no default value
    touch_enabled = LaunchConfiguration('touch_enabled')
    use_standalone_wifi_scanner = LaunchConfiguration('use_standalone_wifi_scanner')
    max_speed = LaunchConfiguration('max_speed')

    xacro_for_cabot_model = PathJoinSubstitution([
        get_package_share_directory('cabot_description'),
        'robots',
        PythonExpression(['"', model_name, '.urdf.xacro.xml', '"'])
    ])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_for_cabot_model, ' offset:=0.25', ' sim:=', use_sim_time]),
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
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Whether the simulated time is used or not'
        ),
        # append prefix name to the log directory for convenience
        RegisterEventHandler(
            OnShutdown(on_shutdown=[AppendLogDirPrefix("cabot")]),
            condition=UnlessCondition(use_sim_time)
        ),
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

        # Kind error message
        LogInfo(
            msg=['You need to specify model parameter'],
            condition=LaunchConfigurationEquals('model', ''),
        ),

        GroupAction([
            # publish robot state
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output=output,
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'publish_frequency': 100.0,
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
                    'use_sim_time': use_sim_time,
                    'publish_frequency': 100.0,
                    'frame_prefix': 'local/',
                    'robot_description': robot_description
                }]
            ),

            # launch velodyne lider related nodes
            ComposableNodeContainer(
                name='laser_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        pkg_dir, 'launch', 'include', 'vlp16.launch.py'
                    ])
                ]),
                launch_arguments={
                    'target_container': 'laser_container'
                }.items(),
                condition=UnlessCondition(use_sim_time)
            ),

            LoadComposableNodes(
                target_container='/laser_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='pointcloud_to_laserscan',
                        plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
                        namespace='',
                        name='pointcloud_to_laserscan_node',
                        parameters=[*param_files, {'use_sim_time': use_sim_time}],
                        remappings=[
                            ('/cloud_in', '/velodyne_points')
                        ]
                    ),
                    ComposableNode(
                        package='pcl_ros',
                        plugin='pcl_ros::CropBox',
                        namespace='',
                        name='filter_crop_box_node',
                        parameters=[*param_files, {'use_sim_time': use_sim_time}],
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
                parameters=[*param_files, {'use_sim_time': use_sim_time}],
            ),

            # Visualize the current speed on Rviz-
            Node(
                package='cabot',
                executable='speed_visualize_node',
                namespace='/cabot',
                name='speed_visualize_node',
                output=output,
                parameters=[*param_files, {'use_sim_time': use_sim_time}],
            ),

            # Microcontroller (Arduino - gt1/gtm or ESP32 - ace)
            Node(
                package='cabot',
                executable='cabot_serial.py',
                namespace='/cabot',
                name='cabot_serial',
                output=output,
                parameters=[
                    *param_files,
                    {'use_sim_time': use_sim_time, 'touch_params': touch_params}
                ],
                remappings=[
                    # ('/cabot/imu', '/cabot/imu/data'),
                    ('/cabot/touch_speed', '/cabot/touch_speed_raw')
                ],
                condition=IfCondition(use_sim_time)
            ),
            Node(
                package='cabot',
                executable='cabot_serial.py',
                namespace='/cabot',
                name='cabot_serial',
                output=output,
                parameters=[
                    *param_files,
                    {'use_sim_time': use_sim_time, 'touch_params': touch_params}
                ],
                remappings=[
                    ('/cabot/imu', '/cabot/imu/data'),
                    ('/cabot/touch_speed', '/cabot/touch_speed_raw')
                ],
                condition=UnlessCondition(use_sim_time)
            ),

            # optional wifi scanner with ESP32
            Node(
                package='cabot',
                executable='cabot_serial.py',
                namespace='/cabot',
                name='serial_esp32_wifi_scanner',
                output=output,
                parameters=[*param_files, {'use_sim_time': use_sim_time}],
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
                parameters=[*param_files, {'use_sim_time': use_sim_time}],
            ),

            # The diagram of Cabot Odometry Adapter & related nodes (*components)
            # move_base's cmd_vel commands will be filtered through nodes to transform
            # command for the robot rotation center to the actual robot center.
            # Motor status will be used for calculating raw odometry of the robot
            # and will be merged by Robot Localization node to get stabilized
            # odometry. Odom adapter will convert the raw odometry to the odometry
            # of the robot rotating center which is controlled by offset.
            #
            #                    (5~10Hz)                      /cabot/cmd_vel_adapter (passthrough)
            # +================+ /cmd_vel    +===============+                   +===============+
            # |                |============>| *             |==================>| *             |
            # | Controller     |             | OdomAdapter   |                   | SpeedControl  |
            # |                |<============|               |                   |               |
            # +================+ /odom       +===============+                   +===============+
            #                                            A                                    |
            #                                            | /cabot/odometry/filtered           | /cabot/cmd_vel  # noqa: E501
            #                   /cabot/imu/data          | (100Hz)                            | (target_rate: 40hz)
            # +================+             +===================+                            |
            # |*Cabot Sensor   |============>| *                 |   (passthrough)            |
            # |================|             | RobotLocalization |  /cabot/odom_raw           |
            #                                |                   |<==================+        |
            #                                +===================+                   |        |
            #                                                                        |        |
            #                                                     (passthrough)      |        |
            #                           (40Hz)                   /cabot/motorStatus  |        v
            # +================+             +==================+               +===============+
            # |                |============>|                  |==============>| *             |
            # | Motor          |   Serial    | MotorControl     |               | MotorAdapter  |
            # |                |<============|                  |<==============|               |
            # +================+             +==================+               +===============+
            #                           (40Hz)                   /cabot/motorTarget (target_rate: 40Hz)

            Node(
                package='cabot',
                executable='odom_adapter_node',
                namespace='/cabot',
                name='odom_adapter_node',
                output=output,
                parameters=[
                    *param_files,
                    {
                        'use_sim_time': use_sim_time,
                        'max_speed': max_speed
                    },
                ],
            ),
            # for local odom navigation
            Node(
                package='cabot',
                executable='odom_adapter_node',
                namespace='/cabot',
                name='odom_adapter_node2',
                output=output,
                parameters=[*param_files, {'use_sim_time': use_sim_time}],
            ),
            # Cabot Lidar Speed Control
            Node(
                package='cabot',
                executable='lidar_speed_control_node',
                namespace='/cabot',
                name='lidar_speed_control_node',
                output=output,
                parameters=[*param_files, {'use_sim_time': use_sim_time}],
            ),
            # Cabot People SPeed Control
            Node(
                package='cabot',
                executable='people_speed_control_node',
                namespace='/cabot',
                name='people_speed_control_node',
                output=output,
                parameters=[*param_files, {'use_sim_time': use_sim_time}],
            ),
            # Cabot TF Speed Control
            Node(
                package='cabot',
                executable='tf_speed_control_node',
                namespace='/cabot',
                name='tf_speed_control_node',
                output=output,
                parameters=[*param_files, {'use_sim_time': use_sim_time}],
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
                parameters=[*param_files, {'use_sim_time': use_sim_time}],
            ),

            # Motor Controller Adapter
            # Convert cmd_vel (linear, rotate) speed to motor target (left, right) speed.
            Node(
                package='motor_adapter',
                executable='odriver_adapter_node',
                namespace='/cabot',
                name='odriver_adapter_node',
                output=output,
                parameters=[*param_files, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('/imu', '/cabot/imu/data')
                ],
                condition=UnlessCondition(use_sim_time),
            ),

            # Motor Controller (ODrive)
            Node(
                package='odriver',
                executable='odriver_node.py',
                namespace='/cabot',
                name='odriver_node',
                output=output,
                parameters=[*param_files, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('/motorTarget', '/cabot/motorTarget'),
                    ('/motorStatus', '/cabot/motorStatus'),
                ],
                condition=UnlessCondition(use_sim_time),
            ),

            # Sensor fusion for stabilizing odometry
            Node(
                package='robot_localization',
                executable='ekf_node',
                namespace='/cabot',
                name='ekf_node',
                output=output,
                parameters=[*param_files, {'use_sim_time': use_sim_time}],
            ),
        ],
            condition=LaunchConfigurationNotEquals('model', '')
        ),
    ])
