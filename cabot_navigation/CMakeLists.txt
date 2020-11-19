cmake_minimum_required(VERSION 2.8.3)
project(cabot_navigation)

find_package(catkin REQUIRED)

catkin_package(
)

#############
## Install ##
#############

install(PROGRAMS
  src/odomtransformer.py
  src/cg_odomtransformer.py
  src/imu_fixer.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  param
  configuration_files
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

