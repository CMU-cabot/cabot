cmake_minimum_required(VERSION 3.5)
project(cabot_description)

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes/)

ament_package()
