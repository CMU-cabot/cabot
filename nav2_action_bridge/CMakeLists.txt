cmake_minimum_required(VERSION 3.5)
project(nav2_action_bridge)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_bridge REQUIRED)


# find ROS 1 packages
include(cmake/find_ros1_package.cmake)

find_package(PkgConfig REQUIRED)

find_ros1_package(roscpp REQUIRED)
find_ros1_package(actionlib REQUIRED)
find_ros1_package(nav2_msgs REQUIRED)

include_directories(include)

add_executable(nav2_navigate_to_pose_bridge_node
  "src/nav2_navigate_to_pose_bridge.cpp"
)
add_executable(nav2_navigate_through_poses_bridge_node
  "src/nav2_navigate_through_poses_bridge.cpp"
)
add_executable(nav2_spin_bridge_node
  "src/nav2_spin_bridge.cpp"
)

set(node_dependencies
  "ros1_roscpp"
  "ros1_actionlib"
  "ros1_nav2_msgs"
  
  "rclcpp"
  "rclcpp_action"
  "nav2_msgs"
  "action_bridge"
)

ament_target_dependencies(nav2_navigate_to_pose_bridge_node ${node_dependencies})
ament_target_dependencies(nav2_navigate_through_poses_bridge_node ${node_dependencies})
ament_target_dependencies(nav2_spin_bridge_node ${node_dependencies})


install(TARGETS nav2_navigate_to_pose_bridge_node nav2_navigate_through_poses_bridge_node nav2_spin_bridge_node
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()
