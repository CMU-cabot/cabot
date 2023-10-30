find_package(ament_cmake_pytest REQUIRED)
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_compression REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(people_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cabot REQUIRED)

set(dependencies
  rclcpp
  rosbag2
  rosbag2_cpp
  rosbag2_compression
  rosbag2_transport
  rosbag2_storage
  rosidl_typesupport_cpp
  geometry_msgs
  nav_msgs
  people_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  cabot
)


add_executable(test_stop_reasons_node
  ../src/stop_reasoner.cpp
  ./stop_reasons_node.cpp
)

ament_target_dependencies(test_stop_reasons_node
  ${dependencies}
)

install(TARGETS
  test_stop_reasons_node
  DESTINATION lib/${PROJECT_NAME}
)


set(_pytest_tests
  test_event.py
  test_geojson.py
  test_geoutil.py
  test_i18n.py
  test_navigation.py
  test_status.py
  # Add other test files here
)

foreach(_test_path ${_pytest_tests})
  get_filename_component(_test_name ${_test_path} NAME_WE)
  ament_add_pytest_test(${_test_name} ${_test_path}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/
    TIMEOUT 60
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_CURRENT_BINARY_DIR}/test_ament_index
  )
endforeach()
