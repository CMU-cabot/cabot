cmake_minimum_required(VERSION 3.8)
project(mf_localization_mapping)

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY configuration_files/ DESTINATION share/${PROJECT_NAME}/configuration_files)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
endif()

install(PROGRAMS
  script/check_topic_size.py
  script/convert_rosbag_for_cartographer.py
  script/extract_floormap_info_from_yaml.py
  script/filter_rosbag_files.sh
  script/filter_rss_samples.py
  script/initial_pose_publisher.py
  script/run_realtime_cartographer_2d_VLP16_for_recording.sh
  script/split_rosbag_files.py
  script/timeout_thread.py
  script/topic_checker.py

  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
