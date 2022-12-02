cmake_minimum_required(VERSION 3.5)
project(cabot_debug)


find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch/)

install(PROGRAMS
  src/command_logger.py
  src/plot_cpu_load.py
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()