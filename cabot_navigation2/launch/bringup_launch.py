# Copyright (c) 2020  Carnegie Mellon University
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
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('cabot_navigation2')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    params_file2 = LaunchConfiguration('params_file2')
    default_bt_xml_file = LaunchConfiguration('default_bt_xml_file')
    default_bt_xml_file2 = LaunchConfiguration('default_bt_xml_file2')
    autostart = LaunchConfiguration('autostart')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    rviz_config_file2 = LaunchConfiguration('rviz_config_file2')
    use_amcl = LaunchConfiguration('use_amcl')
    show_rviz = LaunchConfiguration('show_rviz')
    show_local_rviz = LaunchConfiguration('show_local_rviz')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    remappings2 = [('/local/tf', 'local/tf'),
                   ('/local/tf_static', 'local/tf_static'),
                   ('/local/cmd_vel', '/cmd_vel'),
                   ('/local/odom', '/odom'),
    ]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'yaml_filename': map_yaml_file,
        'default_bt_xml_filename': default_bt_xml_file
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)
    
    param_substitutions2 = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'yaml_filename': map_yaml_file,
        'default_bt_xml_filename': default_bt_xml_file2
    }

    configured_params2 = RewrittenYaml(
        source_file=params_file2,
        root_key="local",
        param_rewrites=param_substitutions2,
        convert_types=True)
    
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(pkg_dir, 'rviz', 'nav2_default_view.rviz'),
            description='Full path to the RVIZ config file to use'),
    
        DeclareLaunchArgument(
            'rviz_config_file2',
            default_value=os.path.join(pkg_dir, 'rviz', 'nav2_default_view_local.rviz'),
            description='Full path to the RVIZ config file to use'),
    
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_dir, "maps", "4fr_gazebo.yaml"),
            description='Full path to map yaml file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'params', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        DeclareLaunchArgument(
            'params_file2',
            default_value=os.path.join(pkg_dir, 'params', 'nav2_params2.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        DeclareLaunchArgument(
            'default_bt_xml_file',
            default_value=os.path.join(
                get_package_share_directory('cabot_bt'),
                'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
            description='Full path to the behavior tree xml file to use'),

        DeclareLaunchArgument(
            'default_bt_xml_file2',
            default_value=os.path.join(
                get_package_share_directory('cabot_bt'),
                'behavior_trees', 'navigate_w_local_odom.xml'),
            description='Full path to the behavior tree xml file to use'),

        DeclareLaunchArgument(
            'autostart', default_value='false',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'use_remappings', default_value='true',
            description='Arguments to pass to all nodes launched by the file'),

        DeclareLaunchArgument(
            'use_amcl', default_value='true',
            description='Whether using AMCL or not'),

        DeclareLaunchArgument(
            'show_rviz', default_value='true',
            description='Whether showing Rviz'),

        DeclareLaunchArgument(
            'show_local_rviz', default_value='true',
            description='Whether showing local Rviz'),

### default navigator
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings+[('/plan', '/plan_temp')],
        ),
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            remappings=remappings,
#            arguments=['--ros-args', '--log-level', 'debug']
        ),

# not using
#        Node(
#            package='nav2_waypoint_follower',
#            executable='waypoint_follower',
#            name='waypoint_follower',
#            output='screen',
#            parameters=[configured_params],
#            remappings=remappings),
        
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': ['controller_server',
                                        'planner_server',
                                        'recoveries_server',
                                        'bt_navigator',
                                        #'waypoint_follower'
                        ]}]),

### local odom navigator
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace='local',
            output='screen',
            parameters=[configured_params2],
            remappings=remappings2,
#            arguments=["--ros-args", "--log-level", "debug"]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace='local',
            output='screen',
            parameters=[configured_params2],
            remappings=remappings2,
#            arguments=["--ros-args", "--log-level", "debug"]
        ),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            namespace='local',
            output='screen',
            parameters=[configured_params2],
            remappings=remappings2),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace='local',
            output='screen',
            parameters=[configured_params2],
            remappings=remappings2,
#            arguments=['--ros-args', '--log-level', 'debug']
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            namespace='local',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': ['controller_server',
                                        'planner_server',
                                        'recoveries_server',
                                        'bt_navigator',
                        ]}]),

### localization
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            condition=IfCondition(use_amcl),
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            condition=IfCondition(use_amcl),
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': ['map_server',
                                        'amcl'
                        ]}]),

        Node(
            condition=UnlessCondition(use_amcl),
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[configured_params]),

### others
        Node(
            package='cabot_util',
            executable='map_loader',
            name='map_loader',
            output='screen',
            parameters=[configured_params]),

        Node(
            package='cabot_util',
            executable='footprint_publisher',
            name='footprint_publisher',
            parameters=[configured_params],
            output='screen'),

        Node(
            package='cabot_util',
            executable='people_vis',
            name='people_vis',
            parameters=[configured_params],
            output='screen'),

        Node(
            package='cabot_navigation2',
            executable='cabot_scan',
            name='cabot_scan',
            parameters=[configured_params],
            output='screen'),

        Node(
            condition=IfCondition(show_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='log'),

        Node(
            condition=IfCondition(show_local_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2_local',
            namespace='local',
            arguments=['-d', rviz_config_file2],
            parameters=[{'use_sim_time': use_sim_time}],
            output='log'),
        ])
