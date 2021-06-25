cmake_minimum_required(VERSION 3.5)
project(cabot_navigation2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(angles REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(people_msgs REQUIRED)
find_package(dwb_core REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
)

set(dependencies
  geometry_msgs
  rclcpp
  nav2_costmap_2d
  nav2_core
  angles
  nav_msgs
  pluginlib
  tf2
  people_msgs
  sensor_msgs
  dwb_core
  )

add_executable(cabot_scan
  src/cabot_scan.cpp
  )

ament_target_dependencies(cabot_scan
  ${dependencies}
  )

### costmap layers
add_library(cabot_layers SHARED
  plugins/navcog_path_layer.cpp
  plugins/people_obstacle_layer.cpp
  )

target_compile_definitions(cabot_layers PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(cabot_layers
  ${dependencies}
  )

install(TARGETS
  cabot_layers
  cabot_scan
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

install(FILES costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
  )

### goal checker
add_library(cabot_goals SHARED
  plugins/cabot_simple_goal_checker.cpp
  plugins/util.cpp
  )

target_compile_definitions(cabot_goals PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(cabot_goals
  ${dependencies}
  )

install(TARGETS
  cabot_goals
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

install(FILES goal_plugins.xml
  DESTINATION share/${PROJECT_NAME}
  )

### dwb critics
add_library(cabot_critics SHARED
  plugins/pose_align.cpp
  )

target_compile_definitions(cabot_critics PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(cabot_critics
  ${dependencies}
  )

install(TARGETS
  cabot_critics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

install(FILES critics_plugins.xml
  DESTINATION share/${PROJECT_NAME}
  )


install(DIRECTORY params/ DESTINATION share/${PROJECT_NAME}/params/)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz/)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch/)

ament_export_include_directories(include)
ament_export_libraries(layers)
ament_export_libraries(cabot_goals)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)
pluginlib_export_plugin_description_file(nav2_core goal_plugins.xml)
pluginlib_export_plugin_description_file(dwb_core critics_plugins.xml)

ament_package()
