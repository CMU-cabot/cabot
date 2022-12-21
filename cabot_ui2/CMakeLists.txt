cmake_minimum_required(VERSION 3.8)
project(cabot_ui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(std_msgs REQUIRED)


set(dependencies
  rclpy
  geometry_msgs
  nav_msgs
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

   # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)

  #ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)

endif()

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/navcog_map.py
  scripts/cabot_keyboard.py
  scripts/cabot_ui_manager.py
  scripts/stop_reasons_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch/)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz/)
install(DIRECTORY i18n/ DESTINATION share/${PROJECT_NAME}/i18n/)
install(DIRECTORY test/launch/ DESTINATION share/${PROJECT_NAME}/test/launch/)

# install(PROGRAMS
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
