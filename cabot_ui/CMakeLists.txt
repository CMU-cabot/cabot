cmake_minimum_required(VERSION 2.8.3)
project(cabot_ui)

find_package(catkin REQUIRED COMPONENTS
  rospy
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

#############
## Install ##
#############

install(DIRECTORY
  src/cabot_ui
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/cabot_ui
  FILES_MATCHING
    PATTERN
    "*.py"
)

install(PROGRAMS
  src/cabot_ble.py
  src/cabot_force.py
  src/cabot_gamepad.py
  src/cabot_keyboard.py
  src/cabot_ui_manager.py
  src/tts_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)

install(PROGRAMS
  test/test_navigation.py
  test/test_geoutil.py
  test/test_geojson.py
  test/test_menu.py
  test/test_tts.py
  test/test_menu_action.py
  test/test_spin.py
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
  add_rostest(test/test_cabot_ui_manager.test)
  add_rostest(test/test_geoutil.py)
  add_rostest(test/test_geojson.py)
endif()

