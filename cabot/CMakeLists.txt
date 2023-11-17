cmake_minimum_required(VERSION 3.8)
project(cabot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wfatal-errors)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(odriver_msgs REQUIRED)
find_package(people_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  diagnostic_updater
  geometry_msgs
  nav_msgs
  odriver_msgs
  people_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

find_package(rclcpp REQUIRED)

include_directories(
  include
)

add_executable(cabot_serial_node
  src/cabot/cabot_serial.cpp
  src/cabot/arduino_serial.cpp
)
ament_target_dependencies(cabot_serial_node
  ${dependencies}
)

add_library(speed_visualize SHARED
  src/cabot/speed_visualize_node.cpp
)
ament_target_dependencies(speed_visualize
  ${dependencies}
)
rclcpp_components_register_node(
  speed_visualize
  PLUGIN "CaBot::SpeedVisualizeNode"
  EXECUTABLE speed_visualize_node
)

add_library(cabot SHARED
  src/safety/util.cpp
)

add_library(safety_nodes SHARED
  src/safety/clearing_tf_node.cpp
  src/safety/lidar_speed_control_node.cpp
  src/safety/odom_adapter_node.cpp
  src/safety/people_speed_control_node.cpp
  src/safety/speed_control_node.cpp
  src/safety/tf_speed_control_node.cpp
  src/safety/util.cpp
)

ament_target_dependencies(cabot
  ${dependencies}
)

ament_target_dependencies(safety_nodes
  ${dependencies}
)

rclcpp_components_register_node(
  safety_nodes
  PLUGIN "CaBotSafety::ClearingTFNode"
  EXECUTABLE clearing_tf_node
)
rclcpp_components_register_node(
  safety_nodes
  PLUGIN "CaBotSafety::LiDARSpeedControlNode"
  EXECUTABLE lidar_speed_control_node
)
rclcpp_components_register_node(
  safety_nodes
  PLUGIN "CaBotSafety::OdomAdapterNode"
  EXECUTABLE odom_adapter_node
  EXECUTOR "MultiThreadedExecutor"
)
rclcpp_components_register_node(
  safety_nodes
  PLUGIN "CaBotSafety::PeopleSpeedControlNode"
  EXECUTABLE people_speed_control_node
)
rclcpp_components_register_node(
  safety_nodes
  PLUGIN "CaBotSafety::SpeedControlNode"
  EXECUTABLE speed_control_node
)
rclcpp_components_register_node(
  safety_nodes
  PLUGIN "CaBotSafety::TFSpeedControlNode"
  EXECUTABLE tf_speed_control_node
)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/cabot_serial.py
  scripts/cabot_handle_v2_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install executables
install(TARGETS
  cabot_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install shared libraries
install(TARGETS
  speed_visualize
  safety_nodes
  cabot
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch/)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories("include/")
ament_export_libraries(cabot)
ament_export_dependencies(${dependencies})

ament_package()
