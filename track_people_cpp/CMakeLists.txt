cmake_minimum_required(VERSION 3.5)
project(track_people_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wfatal-errors)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(track_people_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(diagnostic_updater REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Open3D REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Open3D_INCLUDE_DIRS}
)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

set(dependencies
  rclcpp
  rclcpp_components
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  track_people_msgs
  nav_msgs
  cv_bridge
  diagnostic_updater
  EIGEN3
  Open3D
  OpenCV
)

## detect_darknet_opencv
add_library(detect_darknet_opencv SHARED
  src/detect_darknet_opencv.cpp
)

ament_target_dependencies(detect_darknet_opencv
  ${dependencies}
)

target_link_libraries(detect_darknet_opencv
  ${OpenCV_LIBRARIES}
  ${Open3D_LIBRARIES}
)

rclcpp_components_register_node(
  detect_darknet_opencv
  PLUGIN "track_people_cpp::DetectDarknetOpencv"
  EXECUTABLE detect_darknet_opencv_node
)


## detect_obstacle_on_path
add_library(detect_obstacle_on_path SHARED
  src/detect_obstacle_on_path
)

ament_target_dependencies(detect_obstacle_on_path
  ${dependencies}
)

target_link_libraries(detect_darknet_opencv
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(
  detect_obstacle_on_path
  PLUGIN "track_people_cpp::DetectObstacleOnPath"
  EXECUTABLE detect_obstacle_on_path_node
)


## convert_obstacle_node
# add_executable(convert_obstacle_node
#   src/convert_obstacle_node.cpp
# )

# ament_target_dependencies(convert_obstacle_node
#   # obstacle_detector_generate_messages_cpp
#   ${dependencies}
# )

# target_link_libraries(convert_obstacle_node
#   ${catkin_LIBRARIES}
#   )


install(TARGETS
  # convert_obstacle_node
  detect_darknet_opencv
  detect_obstacle_on_path
  EXPORT export_track_people_cpp
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_export_libraries(detect_darknet_opencv)
ament_export_targets(export_track_people_cpp HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

ament_package()
