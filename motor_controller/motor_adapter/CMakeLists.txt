cmake_minimum_required(VERSION 2.8.3)
project(motor_adapter)

add_compile_options(-std=c++11 -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  std_msgs
  odriver_msgs
  roboclaw_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES motor_adapter
  CATKIN_DEPENDS roscpp nodelet std_msgs odriver_msgs roboclaw_msgs 
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/diff_drive.cpp
  src/odriver_adapter.cpp
  src/roboclaw_adapter.cpp
)

add_dependencies(${PROJECT_NAME}
  roboclaw_msgs_generate_messages_cpp
  odriver_msgs_generate_messages_cpp
  )

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

