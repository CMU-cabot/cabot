cmake_minimum_required(VERSION 3.5)
project(motor_adapter)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(odriver_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_components
  nav_msgs
  odriver_msgs
  sensor_msgs
  std_msgs
  tf2
)

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/diff_drive.cpp
  src/odriver_adapter.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

rclcpp_components_register_nodes(
  ${PROJECT_NAME}
  "MotorAdapter::ODriverNode"
)

add_executable(odriver_adapter_node
  src/odriver_adapter_node.cpp
)

ament_target_dependencies(odriver_adapter_node
  ${dependencies}
)

target_link_libraries(odriver_adapter_node
  ${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
#  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(TARGETS
  odriver_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

#ament_export_libraries(export_${PROJECT_NAME})
#ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
#ament_export_dependencies(${dependencies})

ament_package()
