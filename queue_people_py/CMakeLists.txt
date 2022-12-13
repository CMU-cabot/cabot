cmake_minimum_required(VERSION 3.5)
project(queue_people_py)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(queue_msgs REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY params/ DESTINATION share/${PROJECT_NAME}/params)
install(PROGRAMS
  scripts/detect_queue_people.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
