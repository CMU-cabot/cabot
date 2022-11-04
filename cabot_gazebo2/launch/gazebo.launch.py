import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import LogInfo
from launch.event_handlers import OnExecutionComplete
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    gui = LaunchConfiguration('gui', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    urdf_file_name = 'robots/cabot2-gt1.urdf.xacro'

    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(
        get_package_share_directory('cabot_description'),
        urdf_file_name)

    rviz_conf = os.path.join(
        get_package_share_directory('cabot_gazebo'),
        "launch/test.rviz")

    world = os.path.join(
        get_package_share_directory('cabot_site_miraikan_3d'),
        'worlds/miraikan-5F_simple.world')

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'my_robot']
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch/gzserver.launch.py']),
            launch_arguments={
                'verbose': 'true',
                'world': str(world)
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch/gzclient.launch.py']),
            condition=IfCondition(gui),
            launch_arguments={
                'verbose': 'true'
            }.items()
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time, 
                'robot_description': ParameterValue(
                    Command(['xacro ', str(urdf)]), value_type=str
                )
            }]
        ),

        RegisterEventHandler(
            OnExecutionComplete(
                target_action=spawn_entity,
                on_completion=[
                    LogInfo(msg='Spawn finished'),
                    Node(
                        package='rviz2',
                        executable='rviz2',
                        output='screen',
                        arguments=['-d', str(rviz_conf)]
                    )
                ]
            )
        ),

        spawn_entity,
    ])

