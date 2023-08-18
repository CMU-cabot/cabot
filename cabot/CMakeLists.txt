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

add_library(cabot_serial SHARED
        src/cabot/cabot_serial.cpp
)
ament_target_dependencies(cabot_serial
  ${dependencies}
)
rclcpp_components_register_node(
  cabot_serial
  PLUGIN "CaBot::CaBotSerialNode"
  EXECUTABLE cabot_serial_node
)

add_library(arduino_serial SHARED
        src/cabot/arduino_serial.cpp
)
ament_target_dependencies(arduino_serial
  ${dependencies}
)
rclcpp_components_register_node(
  arduino_serial
  PLUGIN "CaBot::ArduinoSerialNode"
  EXECUTABLE arduino_serial_node
)

#[[

add_library(cabot_arduino_serial_delegate SHARED
        src/cabot/cabot_arduino_serial_delegate.cpp
)
ament_target_dependencies(cabot_arduino_serial_delegate
  ${dependencies}
)
rclcpp_components_register_node(
  cabot_arduino_serial_delegate
  PLUGIN "CaBot::CaBotArduinoSerialDelegateNode"
  EXECUTABLE cabot_arduino_serial_delegate_node
)

add_library(cabot_arduino_serial SHARED
	src/cabot/cabot_arduino_serial.cpp
)
ament_target_dependencies(cabot_arduino_serial
  ${dependencies}
)
rclcpp_components_register_node(
  cabot_arduino_serial
  PLUGIN "CaBot::CaBotArduinoSerialNode"
  EXECUTABLE cabot_arduino_serial_node
)

add_library(topic_check_task SHARED
        src/cabot/topic_check_task.cpp
)
ament_target_dependencies(topic_check_task
  ${dependencies}
)
rclcpp_components_register_node(
  topic_check_task
  PLUGIN "CaBot::TopicCheckTaskNode"
  EXECUTABLE topic_check_task_node
)

add_library(check_connection_task SHARED
        src/cabot/check_connection_task.cpp
)
ament_target_dependencies(check_connection_task
  ${dependencies}
)
rclcpp_components_register_node(
  check_connection_task
  PLUGIN "CaBot::CheckConnectionTaskNode"
  EXECUTABLE check_connection_task_node
)

add_library(cabot_serial SHARED
  src/cabot/cabot_serial.cpp
)
ament_target_dependencies(cabot_serial
  ${dependencies}
)
rclcpp_components_register_node(
  cabot_serial
  PLUGIN "CaBot::CaBotSerialNode"
  EXECUTABLE cabot_serial_node
)
#]]

add_library(safety_nodes SHARED
  src/safety/clearing_tf_node.cpp
  src/safety/lidar_speed_control_node.cpp
  src/safety/odom_adapter_node.cpp
  src/safety/people_speed_control_node.cpp
  src/safety/speed_control_node.cpp
  src/safety/tf_speed_control_node.cpp
  src/safety/util.cpp
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


# todo add components / executables

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/cabot_serial.py
  scripts/cabot_handle_v2_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install shared libraries
install(TARGETS
  cabot_serial
  arduino_serial
  #cabot_arduino_serial_delegate
  #cabot_arduino_serial
  safety_nodes
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch/)

ament_package()
