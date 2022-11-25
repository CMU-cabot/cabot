cmake_minimum_required(VERSION 3.5)
project(cabot_gazebo)

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY params/ DESTINATION share/${PROJECT_NAME}/params)
install(DIRECTORY script/ DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  script/floor_transition_node.py
  script/wireless_rss_simulator_node.py
  script/wireless_sample_simulator_node.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
