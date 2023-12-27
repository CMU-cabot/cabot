cmake_minimum_required(VERSION 3.8)
project(cabot_diagnostics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wfatal-errors)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(diagnostic_aggregator REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(pluginlib REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

add_library(cabot_diagnostics_analyzers SHARED
  src/cabot_analyzer.cpp)

ament_target_dependencies(
  cabot_diagnostics_analyzers
  diagnostic_aggregator
  pluginlib
  diagnostic_msgs
  rclcpp
)

install(TARGETS
  cabot_diagnostics_analyzers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

target_include_directories(cabot_diagnostics_analyzers PRIVATE
  ${diagnostic_aggregator_INCLUDE_DIRS}
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

pluginlib_export_plugin_description_file(diagnostic_aggregator plugin_description.xml)

ament_package()
