cmake_minimum_required(VERSION 3.5)
project(track_people_msgs)

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

ament_package()
