cmake_minimum_required(VERSION 2.8.3)
project(mf_localization_msgs)

set(PACKAGE_DEPENDENCIES
  geometry_msgs
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  ${PACKAGE_DEPENDENCIES}
)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
    StatusResponse.msg
    MFGlobalPosition.msg
    MFLocalPosition.msg
 )

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY srv
  FILES
    MFTrigger.srv
    MFSetInt.srv
    StopLocalization.srv
    StartLocalization.srv
    RestartLocalization.srv
    FloorChange.srv
    ConvertLocalToGlobal.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    ${PACKAGE_DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
    message_runtime
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
