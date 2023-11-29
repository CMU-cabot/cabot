cmake_minimum_required(VERSION 3.8)
project(mf_localization)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY configuration_files/ DESTINATION share/${PROJECT_NAME}/configuration_files)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
endif()

ament_python_install_package(${PROJECT_NAME})

add_executable(multi_floor_topic_proxy
  src/multi_floor_topic_proxy.cpp
  )

ament_target_dependencies(multi_floor_topic_proxy
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  yaml_cpp_vendor
)

install(TARGETS
  multi_floor_topic_proxy
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  script/fix_filter.py
  script/imu_frame_renamer.py
  script/multi_floor_manager.py
  script/multi_floor_map_server.py
  script/multi_floor_wireless_rss_localizer.py
  script/tf2_beacons_listener.py
  script/tf2_listener.py
  script/trajectory_restarter.py
  script/ublox_converter.py
  script/wireless_rss_localizer_node.py
  script/wireless_rss_localizer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
