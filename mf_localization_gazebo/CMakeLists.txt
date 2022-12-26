cmake_minimum_required(VERSION 3.8)
project(mf_localization_gazebo)

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)


install(PROGRAMS
  script/floor_transition_node.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
