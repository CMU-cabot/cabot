cmake_minimum_required(VERSION 2.8.3)
project(cabot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roslint
  rospy
  roscpp
  nodelet
  cabot_msgs
  queue_msgs
  roboclaw_msgs
  odriver_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  )

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cabot
  CATKIN_DEPENDS roscpp nodelet
  )

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

## Declare a C++ library
add_library(cabot_nodelet
  src/cabot/cabot-e_sensor_nodelet.cpp
  src/cabot/cabot-f_sensor_nodelet.cpp
  src/cabot/speed_visualize_nodelet.cpp
  src/safety/odom_adapter_nodelet.cpp
  src/safety/clutch_control_nodelet.cpp
  src/safety/speed_control_nodelet.cpp
  src/safety/clearing_tf_nodelet.cpp
  src/safety/lidar_speed_control_nodelet.cpp
  src/safety/people_speed_control_nodelet.cpp
  src/safety/tf_speed_control_nodelet.cpp
  src/safety/util.cpp
  )

add_dependencies(cabot_nodelet
  cabot_msgs_generate_messages_cpp
  )

target_link_libraries(cabot_nodelet
  ${catkin_LIBRARIES}
  )

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(cabot_nodelet ${catkin_EXPORTED_LIBRARIES})
endif()

#############
## Install ##
#############

install(PROGRAMS
  python/cabot_button_node.py
  python/cabot_mobile_base.py
  python/cabot_serial.py
  python/cabot_keyboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(PROGRAMS
  python/cabot/event.py
  python/cabot/util.py
  python/cabot/__init__.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/cabot
  )

install(TARGETS cabot_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY
  launch
  param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES cabot_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

#############
## Testing ##
#############

set(ROSLINT_CPP_OPTS --extensions=cc,cpp,cu,cuh,h,hpp)
roslint_cpp()

catkin_add_nosetests(test)
