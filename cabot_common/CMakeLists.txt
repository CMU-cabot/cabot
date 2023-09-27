cmake_minimum_required(VERSION 3.8)
project(cabot_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wfatal-errors)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cabot_msgs REQUIRED)
find_package(people_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)

set(dependencies
  rclcpp
  people_msgs
  visualization_msgs
  tf2
)

add_executable(people_vis_node
  src/people_vis_node.cpp
)

ament_target_dependencies(people_vis_node
  ${dependencies}
)

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

add_library(lookup_transform_service SHARED
  src/lookup_transform_service.cpp
)

ament_target_dependencies(lookup_transform_service
  cabot_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2_ros
  rclcpp
  rclcpp_components
)

rclcpp_components_register_node(
  lookup_transform_service
  PLUGIN "CaBot::LookupTransformServiceNode"
  EXECUTABLE lookup_transform_service_node
)

install(PROGRAMS
  scripts/footprint_publisher.py
  scripts/map_loader.py
  scripts/people_vis.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  lookup_transform_service
  people_vis_node
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_package()
