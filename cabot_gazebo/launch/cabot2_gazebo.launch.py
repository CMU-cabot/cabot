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
import os.path
import tempfile
import traceback
import xml.dom.minidom

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, Substitution
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.event_handlers import OnShutdown
from launch.event_handlers import OnExecutionComplete
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import EnvironmentVariable
from launch.substitutions import OrSubstitution
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

from launch.logging import launch_config
from cabot_common.launch import AppendLogDirPrefix


class AddStatePlugin(Substitution):
    def __init__(self, source_file):
        super().__init__()
        self.source_file = normalize_to_list_of_substitutions(source_file)

    def describe(self):
        return ""

    def perform(self, context):
        xml_filename = perform_substitutions(context, self.source_file)
        rewritten_xml = tempfile.NamedTemporaryFile(mode='w', delete=False,
                                                    prefix='sdf', suffix='.xml')
        try:
            sdf = xml.dom.minidom.parse(xml_filename)
            plugin = xml.dom.minidom.parseString("""
<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
  <ros>
    <namespace>/gazebo</namespace>
  </ros>
  <update_rate>1.0</update_rate>
</plugin>
            """)
            worlds = sdf.getElementsByTagName("world")
            if len(worlds) != 1:
                return xml_filename
            worlds[0].appendChild(plugin.firstChild)
            sdf.writexml(rewritten_xml)
            return rewritten_xml.name
        except:  # noqa: E722
            traceback.print_exc()
        return xml_filename


def generate_launch_description():
    pkg_dir = get_package_share_directory('cabot_gazebo')

    show_gazebo = LaunchConfiguration('show_gazebo')
    show_rviz = LaunchConfiguration('show_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    model_name = LaunchConfiguration('model')
    world_file = LaunchConfiguration('world_file')
    wireless_config_file = LaunchConfiguration('wireless_config_file')

    rviz_conf = os.path.join(
        pkg_dir,
        "launch/test.rviz")

    gazebo_params = os.path.join(
        pkg_dir,
        "params/gazebo.params.yaml")

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic',
            '/robot_description',
            '-entity',
            'mobile_base',
            '-x',
            EnvironmentVariable('CABOT_INITX', default_value='0'),
            '-y',
            EnvironmentVariable('CABOT_INITY', default_value='0'),
            '-z',
            EnvironmentVariable('CABOT_INITZ', default_value='0'),
            '-Y',
            EnvironmentVariable('CABOT_INITAR', default_value='0')
        ]
    )

    modified_world = AddStatePlugin(world_file)

    return LaunchDescription([
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        # append prefix name to the log directory for convenience
        RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("cabot_gazebo")])),

        DeclareLaunchArgument(
            'show_gazebo',
            default_value=EnvironmentVariable('CABOT_SHOW_GAZEBO_CLIENT', default_value='false'),
            description='Show Gazebo client if true'
        ),
        DeclareLaunchArgument(
            'show_rviz',
            default_value='false',
            description='Show rviz2 if true'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'model',
            default_value='',
            description='Robot URDF xacro model name'
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value=PathJoinSubstitution([get_package_share_directory('gazebo_ros'), 'worlds', 'empty.world']),
            description='Gazebo world file to be open'
        ),
        DeclareLaunchArgument(
            'wireless_config_file',
            default_value='',
            description='wireless config file'
        ),

        LogInfo(
            msg=['You need to specify model, world_file parameter'],
            condition=IfCondition(OrSubstitution(
                PythonExpression(['"', model_name, '"==""']),
                PythonExpression(['"', world_file, '"==""'])
            ))
        ),

        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'),
                                              '/launch/gzserver.launch.py']),
                launch_arguments={
                    'verbose': 'true',
                    'world': modified_world,
                    'params_file': str(gazebo_params)
                }.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([pkg_dir,
                                              '/launch/gazebo_wireless_helper.launch.py']),
                launch_arguments={
                    'verbose': 'true',
                    'namespace': 'wireless',
                    'wireless_config_file': wireless_config_file
                }.items(),
                condition=LaunchConfigurationNotEquals('wireless_config_file', '')
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([get_package_share_directory('cabot'),
                                              '/launch/cabot2.launch.py']),
                launch_arguments={
                    'model': model_name,
                    'use_sim_time': 'true'
                }.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'),
                                              '/launch/gzclient.launch.py']),
                condition=IfCondition(show_gazebo),
                launch_arguments={
                    'verbose': 'true'
                }.items()
            ),
            #        Node(
            #            package='joint_state_publisher_gui',
            #            executable='joint_state_publisher_gui',
            #            name='joint_state_publisher',
            #            output='log',
            #            parameters=[{
            #                'use_sim_time': use_sim_time,
            #                'rate': 20.0,
            #            }]
            #        ),
            RegisterEventHandler(
                OnExecutionComplete(
                    target_action=spawn_entity,
                    on_completion=[
                        LogInfo(msg='Spawn finished'),
                        Node(
                            condition=IfCondition(show_rviz),
                            package='rviz2',
                            executable='rviz2',
                            output='screen',
                            parameters=[{
                                'use_sim_time': use_sim_time,
                            }],
                            arguments=['-d', str(rviz_conf)]
                        )
                    ]
                )
            ),
            TimerAction(
                period=15.0,  # TODO: it depends on how long gazebo takes time to be launched. so need to check gazebo status to decide when the robot is spawn
                actions=[spawn_entity],
            )
        ],
            condition=UnlessCondition(OrSubstitution(
                PythonExpression(['"', model_name, '"==""']),
                PythonExpression(['"', world_file, '"==""'])
            ))
        )
    ])
