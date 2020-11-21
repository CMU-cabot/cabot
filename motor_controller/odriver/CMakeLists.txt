cmake_minimum_required(VERSION 2.8.3)
project(odriver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  odriver_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs odriver_msgs
)

install(PROGRAMS
  scripts/odriver_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
