cmake_minimum_required(VERSION 2.8.3)
project(cabot_explore)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  cabot_explore_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy cabot_explore_msgs
)

###########
## Build ##
###########

include_directories(
)

#############
## Install ##
#############

#############
## Testing ##
#############
