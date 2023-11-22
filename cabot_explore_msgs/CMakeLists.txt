cmake_minimum_required(VERSION 2.8.3)
project(cabot_explore_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  rospy
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   RestartExploreMapping.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

#############
## Testing ##
#############
