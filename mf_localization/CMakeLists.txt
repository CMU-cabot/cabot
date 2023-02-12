cmake_minimum_required(VERSION 3.8)
project(mf_localization)

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY configuration_files/ DESTINATION share/${PROJECT_NAME}/configuration_files)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
endif()

ament_python_install_package(${PROJECT_NAME})

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
