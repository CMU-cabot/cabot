cmake_minimum_required(VERSION 3.5)
project(cabot_site_cmu)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/)
install(DIRECTORY i18n/ DESTINATION share/${PROJECT_NAME}/i18n/)
install(DIRECTORY maps/ DESTINATION share/${PROJECT_NAME}/maps/)
install(DIRECTORY menu/ DESTINATION share/${PROJECT_NAME}/menu/)
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds/)

ament_package()
