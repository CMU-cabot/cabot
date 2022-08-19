cmake_minimum_required(VERSION 2.8.3)
project(cabot_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  rospy
)
## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Encoder.msg
   Motor.msg
   ArduinoSensors.msg
   PoseLog.msg
   Log.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   Speak.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
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

catkin_add_nosetests(test)
