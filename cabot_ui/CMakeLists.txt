cmake_minimum_required(VERSION 2.8.3)
project(cabot_ui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_core
)

find_package(rostest REQUIRED)

catkin_python_setup()

catkin_package(
)

###########
## Build ##
###########

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_library(navcog_global_planner_lib
  src/navcog_global_planner.cpp
)

#############
## Install ##
#############

install(PROGRAMS
  src/tts_node.py
  src/cabot_menu_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS
  src/cabot_ui/geoutil.py
  src/cabot_ui/datautil.py
  src/cabot_ui/geojson.py	
  src/cabot_ui/navcog.py
  src/cabot_ui/menu.py
  src/cabot_ui/__init__.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/cabot_ui
)
install(PROGRAMS
  test/test_navcognode.py
  test/test_geoutil.py
  test/test_geojson.py
  test/test_menu.py
  test/test_tts.py
  test/test_menu_action.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/test
)
install(DIRECTORY
  launch
  rviz
  menu
  test/data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(cabot_msgs REQUIRED)
  
  add_rostest(test/test_menu.test)
  add_rostest(test/test_menu_action.test)
  add_rostest(test/test_tts.test)	
  add_rostest(test/test_navcognode.test)
  add_rostest(test/test_geolibs.test)	
#  catkin_add_nosetests(test/test_tts.py)
endif()

