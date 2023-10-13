cmake_minimum_required(VERSION 3.5)
project(cabot_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/PoseLog.msg
  msg/Log.msg
  msg/StopReason.msg
  srv/Speak.srv
  srv/LookupTransform.srv
  DEPENDENCIES geometry_msgs std_msgs tf2_msgs
)

ament_package()
