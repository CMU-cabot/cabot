cmake_minimum_required(VERSION 2.8.3)
project(cabot_navigation)


find_package(catkin REQUIRED COMPONENTS
  nav_core
  )

catkin_package(
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_library(cabot_costmap_plugins
  plugins/navcog_path_layer.cpp
  plugins/people_obstacle_layer.cpp
)

add_library(cabot_planner_plugins
  plugins/navcog_global_planner.cpp
)

#############
## Install ##
#############

install(PROGRAMS
  src/odomtransformer.py
  src/cg_odomtransformer.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )

 install(PROGRAMS
   test/test_navcog_path_layer.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/test
   )
 
install(DIRECTORY
  launch
  param
  configuration_files
  test/data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(cabot_msgs REQUIRED)
  
  add_rostest(test/navcog_path_layer.test)
endif()
