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


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
from launch.actions import LogInfo
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import OrSubstitution
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    output = 'both'
    pkg_dir = get_package_share_directory('cabot_debug')

    bagfile = LaunchConfiguration('bagfile')
    rate = LaunchConfiguration('rate')
    start = LaunchConfiguration('start')

    rviz_file = PathJoinSubstitution([
        pkg_dir, 'config', 'nav2_default_view.rviz'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'bagfile',
            default_value='',
            description='playback file'
        ),
        DeclareLaunchArgument(
            'rate',
            default_value='1.0',
            description='playback rate'
        ),
        DeclareLaunchArgument(
            'start',
            default_value='0.01',
            description='playback start offset'
        ),

        # Kind error message
        LogInfo(
            msg=['You need to specify bagfile parameter'],
            condition=LaunchConfigurationEquals('bagfile', ''),
        ),

        GroupAction([
            SetParameter('use_sim_time', 'true'),

            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_file],
                output=output,
            ),

            ExecuteProcess(
                cmd=["ros2", "bag", "play",
                     "--clock", "100",
                     "--rate", rate,
                     "--start-offset", start,
                     bagfile]
            ),
        ],
        condition=LaunchConfigurationNotEquals('bagfile', '')
        )
    ])
