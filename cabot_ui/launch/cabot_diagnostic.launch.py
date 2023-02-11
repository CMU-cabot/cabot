from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition
from ament_index_python import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('cabot_ui')
    show_robot_monitor = LaunchConfiguration('show_robot_monitor')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument('show_robot_monitor', default_value='true'),
        DeclareLaunchArgument('config_file', default_value=PathJoinSubstitution([pkg_dir, 'config', 'cabot_diagnostic.yaml'])),

        Node(
            package="rqt_robot_monitor",
            executable="rqt_robot_monitor",
            name="rqt_robot_monitor",
            condition=IfCondition(show_robot_monitor)
        ),
        Node(
            package="diagnostic_aggregator",
            executable="aggregator_node",
            name="diagnostic_aggregator",
            parameters=[config_file]
        ),
    ])

"""
<launch>
  <arg name="show_robot_monitor" default="1" />
  <arg name="config_file" default="$(find-pkg-share cabot_ui)/config/cabot_diagnostic.yaml" />
  
  <!-- rosbridge for external BLE server  -->
  <node pkg="rqt_robot_monitor" exec="rqt_robot_monitor"
	name="rqt_robot_monitor" output="log" if="$(var show_robot_monitor)">
  </node>

  <node pkg="diagnostic_aggregator" exec="aggregator_node"
	name="diagnostic_aggregator" output="screen">
    <param from="$(var config_file)" />
  </node>
</launch>
"""
