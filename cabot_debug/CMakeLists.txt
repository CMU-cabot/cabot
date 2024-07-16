cmake_minimum_required(VERSION 3.5)
project(cabot_debug)


find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch/)

install(PROGRAMS
  src/command_logger.py
  src/plot_cpu_load.py
  src/plot_speed.py
  src/check_bag_topic_size.py
  src/plot_points2_timestamp.py
  src/plot_odom.py
  src/plot_ctrl.py
  src/plot_temperature.py
  src/plot_pressure.py
  src/print_topics.py
  src/run_test.py
  src/check_people.py
  src/check_speed_control.py
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
