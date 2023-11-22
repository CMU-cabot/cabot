cmake_minimum_required(VERSION 2.8.3)
project(route_explore)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rospy std_msgs actionlib actionlib_msgs geometry_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(explore_local_costmap_node src/explore_local_costmap_node.cpp)
target_link_libraries(explore_local_costmap_node ${catkin_LIBRARIES})

#############
## Install ##
#############

#############
## Testing ##
#############
