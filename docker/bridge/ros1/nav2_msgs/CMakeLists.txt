cmake_minimum_required(VERSION 3.0.2)
project(nav2_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib_msgs
    geometry_msgs
    message_generation
)

add_action_files(
  DIRECTORY
    action
  FILES
    NavigateToPose.action
    NavigateThroughPoses.action
    Spin.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS actionlib_msgs geometry_msgs message_runtime
)
