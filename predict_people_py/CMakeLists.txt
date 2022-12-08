cmake_minimum_required(VERSION 3.5)
project(predict_people_py)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(people_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(track_people_py REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY scripts/ DESTINATION lib/${PROJECT_NAME}/)
install(PROGRAMS
  scripts/predict_kf_people.py
  scripts/predict_kf_obstacle.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
