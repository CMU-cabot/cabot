cmake_minimum_required(VERSION 3.5)
project(cabot_bt CXX)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wfatal-errors)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(dwb_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(people_msgs REQUIRED)
find_package(queue_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(shared_queues_vendor REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(
  include
  )

set(dependencies
  rcl
  rclcpp
  nav2_msgs
  nav2_util
  people_msgs
  queue_msgs
  behaviortree_cpp_v3
  nav2_behavior_tree
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

add_library(cabot_people_exist_bt_node SHARED
  plugins/condition/people_exist.cpp
)
list(APPEND plugin_libs cabot_people_exist_bt_node)

add_library(cabot_someone_on_path_bt_node SHARED
  plugins/condition/someone_on_path.cpp
)
list(APPEND plugin_libs cabot_someone_on_path_bt_node)

add_library(cabot_someone_not_in_queue_bt_node SHARED
  plugins/condition/someone_not_in_queue.cpp
)
list(APPEND plugin_libs cabot_someone_not_in_queue_bt_node)

add_library(cabot_far_enough_bt_node SHARED
  plugins/condition/far_enough.cpp
)
list(APPEND plugin_libs cabot_far_enough_bt_node)

add_library(cabot_need_to_avoid_bt_node SHARED
  plugins/condition/need_to_avoid.cpp
)
list(APPEND plugin_libs cabot_need_to_avoid_bt_node)

add_library(cabot_check_path_bt_node SHARED
  plugins/condition/check_path.cpp
)
list(APPEND plugin_libs cabot_check_path_bt_node)

add_library(cabot_can_pass_person_bt_node SHARED
  plugins/condition/can_pass_person.cpp
)
list(APPEND plugin_libs cabot_can_pass_person_bt_node)

add_library(cabot_ignore_people_bt_node SHARED
  plugins/action/ignore_people.cpp
)
list(APPEND plugin_libs cabot_ignore_people_bt_node)

add_library(cabot_person_stops_bt_node SHARED
  plugins/condition/person_stops.cpp
)
list(APPEND plugin_libs cabot_person_stops_bt_node)

add_library(cabot_change_param_bt_node SHARED
  plugins/action/change_param.cpp
)
list(APPEND plugin_libs cabot_change_param_bt_node)

add_library(cabot_restore_config_bt_node SHARED
  plugins/decorator/restore_config.cpp
)
list(APPEND plugin_libs cabot_restore_config_bt_node)

add_library(cabot_clear_costmap_around_robot_bt_node SHARED
  plugins/action/clear_costmap_service.cpp
)
list(APPEND plugin_libs cabot_clear_costmap_around_robot_bt_node)

add_library(cabot_wait_float_bt_node SHARED
  plugins/action/wait_float.cpp
)
list(APPEND plugin_libs cabot_wait_float_bt_node)

add_library(cabot_publish_topic_bt_node SHARED
  plugins/action/publish_topic.cpp
)
list(APPEND plugin_libs cabot_publish_topic_bt_node)


foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(check_log
  src/check_log.cpp
)
add_executable(check_eval
  src/check_eval.cpp
  )

set(dependencies
  rclcpp
  rosbag2
  rosbag2_cpp
  rosbag2_transport
  rosbag2_storage
  shared_queues_vendor
  nav2_msgs
  dwb_msgs
  geometry_msgs
  )

ament_target_dependencies(check_log ${dependencies})
ament_target_dependencies(check_eval ${dependencies})


install(TARGETS ${library_name}
  ${plugin_libs}
  check_log
  check_eval
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)
install(TARGETS check_log DESTINATION bin/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(
  ${library_name}
  ${plugin_libs}
)

ament_export_dependencies(${dependencies})

ament_package()
