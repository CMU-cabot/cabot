cmake_minimum_required(VERSION 2.8.3)
project(cabot_description)

find_package(catkin REQUIRED)

catkin_package(
)

#include_directories(
#)

#############
## Install ##
#############

install(DIRECTORY
  launch
  robots
  urdf
  meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
