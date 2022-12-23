cmake_minimum_required(VERSION 3.5)
project(track_people_py)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoundingBox.msg"
  "msg/TrackedBox.msg"
  "msg/TrackedBoxes.msg"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY params/ DESTINATION share/${PROJECT_NAME}/params)
## TODO
install(DIRECTORY scripts/ DESTINATION share/${PROJECT_NAME}/scripts)
install(DIRECTORY scripts/ DESTINATION lib/${PROJECT_NAME}/)
install(PROGRAMS
  scripts/detect_darknet_opencv_people.py
  scripts/track_sort_3d_people.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
