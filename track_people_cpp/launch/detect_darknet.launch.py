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

from launch.logging import launch_config

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory
from cabot_common.launch import AppendLogDirPrefix


def generate_launch_description():
    map_frame = LaunchConfiguration('map_frame')
    namespace = LaunchConfiguration('namespace')
    camera_link_frame = LaunchConfiguration('camera_link_frame')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    image_rect_topic = LaunchConfiguration('image_rect_topic')
    depth_registered_topic = LaunchConfiguration('depth_registered_topic')
    depth_unit_meter = LaunchConfiguration('depth_unit_meter')
    target_fps = LaunchConfiguration('target_fps')
    use_composite = LaunchConfiguration('use_composite')
    target_container = LaunchConfiguration('target_container')
    publish_simulator_people = LaunchConfiguration('publish_simulator_people')

    # ToDo: workaround https://github.com/CMU-cabot/cabot/issues/86
    jetpack5_workaround = LaunchConfiguration('jetpack5_workaround')

    yolov4_cfg = PathJoinSubstitution([get_package_share_directory('track_people_py'), 'models', 'yolov4.cfg'])
    yolov4_weights = PathJoinSubstitution([get_package_share_directory('track_people_py'), 'models', 'yolov4.weights'])
    coco_names = PathJoinSubstitution([get_package_share_directory('track_people_py'), 'models', 'coco.names'])

    return LaunchDescription([
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        # append prefix name to the log directory for convenience
        RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("track_people_cpp-detect_darknet")])),

        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('namespace', default_value='camera'),
        DeclareLaunchArgument('camera_link_frame', default_value='camera_link'),
        DeclareLaunchArgument('camera_info_topic', default_value='color/camera_info'),
        DeclareLaunchArgument('image_rect_topic', default_value='color/image_raw'),
        DeclareLaunchArgument('depth_registered_topic', default_value='aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument('depth_unit_meter', default_value='false'),
        DeclareLaunchArgument('target_fps', default_value='15.0'),
        DeclareLaunchArgument('use_composite', default_value='false'),
        DeclareLaunchArgument('target_container', default_value='camera_manager'),
        DeclareLaunchArgument('publish_simulator_people', default_value='false'),

        DeclareLaunchArgument('jetpack5_workaround', default_value='false'),

        # overwrite parameters
        SetParameter(name='map_frame', value=map_frame),
        SetParameter(name='camera_id', value=namespace),
        SetParameter(name='camera_link_frame', value=camera_link_frame),
        SetParameter(name='camera_info_topic', value=camera_info_topic),
        SetParameter(name='image_rect_topic', value=image_rect_topic),
        SetParameter(name='depth_registered_topic', value=depth_registered_topic),
        SetParameter(name='depth_unit_meter', value=depth_unit_meter),
        SetParameter(name='target_fps', value=target_fps),
        SetParameter(name='publish_simulator_people', value=publish_simulator_people),
        SetParameter(name='detection_threshold', value=0.25),
        SetParameter(name='minimum_detection_size_threshold', value=50.0),
        SetParameter(name='detect_config_file', value=yolov4_cfg),
        SetParameter(name='detect_weight_file', value=yolov4_weights),
        SetParameter(name='detect_label_file', value=coco_names),

        SetEnvironmentVariable(name='LD_PRELOAD', value='/usr/local/lib/libOpen3D.so', condition=IfCondition(jetpack5_workaround)),
        Node(
            package="track_people_cpp",
            executable="detect_darknet_opencv_node",
            name="detect_darknet_people_cpp",
            namespace=namespace,
            condition=UnlessCondition(use_composite)
        ),

        LoadComposableNodes(
            target_container=PythonExpression(["\"", namespace, "/", target_container, "\""]),
            composable_node_descriptions=[
                ComposableNode(
                    package="track_people_cpp",
                    plugin="track_people_cpp::DetectDarknetOpencv",
                    name="detect_darknet_people_cpp",
                    namespace=namespace,
                ),
            ],
            condition=IfCondition(use_composite),
        )
    ])
