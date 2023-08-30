cmake_minimum_required(VERSION 3.5)
project(odriver)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

install(PROGRAMS
  script/odriver_node.py
  script/odriver_s1_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
