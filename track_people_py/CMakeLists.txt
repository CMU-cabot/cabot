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

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)
install(PROGRAMS
  scripts/detect_darknet_opencv_people.py
  scripts/track_sort_3d_people.py
  scripts/predict_kf_obstacle.py
  scripts/predict_kf_people.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
