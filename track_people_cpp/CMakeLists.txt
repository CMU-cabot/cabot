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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(track_people_py REQUIRED)
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
  sensor_msgs
  std_msgs
  std_srvs
  track_people_py
  cv_bridge
  diagnostic_updater
  EIGEN3
  Open3D
  OpenCV
)

## detect_darknet_opencv_node
add_library(detect_darknet_opencv SHARED
  src/detect_darknet_opencv_node.cpp
  src/detect_darknet_opencv.cpp
)

ament_target_dependencies(detect_darknet_opencv
  # track_people_py_generate_messages_cpp
  ${dependencies}
)

target_link_libraries(detect_darknet_opencv
  ${OpenCV_LIBRARIES}
  ${Open3D_LIBRARIES}
)

rclcpp_components_register_nodes(
  detect_darknet_opencv
  "TrackPeopleCPP::DetectDarknetOpencvNode"
#  at least this works is working with rolling, not sure with humble
#  PLUGIN "TrackPeopleCPP::DetectDarknetOpencvNode" 
#  EXECUTABLE detect_darknet_opencv_node
)

## manual_composition (for TEST)
add_executable(manual_composition
  src/manual_composition.cpp
)

ament_target_dependencies(manual_composition
  ${dependencies})

target_link_libraries(manual_composition
  detect_darknet_opencv)

# ## detect_obstacle_on_path_node
# add_executable(detect_obstacle_on_path_node
#   src/detect_obstacle_on_path_node.cpp
#   src/detect_obstacle_on_path.cpp
# )

# ament_target_dependencies(detect_obstacle_on_path_node
#   ${dependencies}
# )

# target_link_libraries(detect_obstacle_on_path_node
#   ${OpenCV_LIBRARIES}
# )

# ## convert_obstacle_node
# add_executable(convert_obstacle_node
#   src/convert_obstacle_node.cpp
# )

# ament_target_dependencies(convert_obstacle_node
#   # obstacle_detector_generate_messages_cpp
#   ${dependencies}
# )

# # target_link_libraries(convert_obstacle_node
# #   ${catkin_LIBRARIES}
# #   )


# ## Declare a C++ library
# add_library(track_people_cpp_nodelet
#   src/detect_darknet_opencv_nodelet.cpp
#   src/detect_darknet_opencv.cpp
#   src/detect_obstacle_on_path_nodelet.cpp
#   src/detect_obstacle_on_path.cpp
# )

# ament_target_dependencies(track_people_cpp_nodelet
#   # track_people_py_generate_messages_cpp
#   ${dependencies}
# )

# target_link_libraries(track_people_cpp_nodelet
#   ${OpenCV_LIBRARIES}
#   ${Open3D_LIBRARIES}
# )

# # if(catkin_EXPORTED_LIBRARIES)
# #   add_dependencies(track_people_cpp_nodelet ${catkin_EXPORTED_LIBRARIES})
# # endif()



install(TARGETS
  #track_people_cpp_nodelet
  # detect_obstacle_on_path_node
  # convert_obstacle_node
  detect_darknet_opencv
  #detect_darknet_opencv_node # TODO: need to fix this work around for galactic
  EXPORT export_track_people_cpp
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)


install(TARGETS
  manual_composition
  DESTINATION lib/${PROJECT_NAME}
)


# install(FILES nodelet.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#   )

ament_export_libraries(detect_darknet_opencv)
ament_export_targets(export_track_people_cpp HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

ament_package()







# find_package(catkin REQUIRED COMPONENTS
#   cv_bridge
#   nodelet
#   roscpp
#   sensor_msgs
#   std_msgs
#   tf2_ros
#   track_people_py
#   diagnostic_updater
#   obstacle_detector
# )

# find_package(OpenCV REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(Open3D REQUIRED)

# catkin_package(
#   CATKIN_DEPENDS 
#   cv_bridge
#   nodelet
#   roscpp
#   sensor_msgs
#   std_msgs
#   tf2_ros
#   track_people_py
#   diagnostic_updater
#   obstacle_detector

#   DEPENDS
#   EIGEN3
#   Open3D
#   OpenCV

#   LIBRARIES ${PROJECT_NAME}
# )

# ###########
# ## Build ##
# ###########

# include_directories(
#   ${catkin_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIRS}
#   ${Open3D_INCLUDE_DIRS}  
#   ${OpenCV_INCLUDE_DIRS}  
# )

# ## detect_darknet_opencv_node
# add_executable(detect_darknet_opencv_node
#   src/detect_darknet_opencv_node.cpp
#   src/detect_darknet_opencv.cpp
# )

# add_dependencies(detect_darknet_opencv_node
#   track_people_py_generate_messages_cpp
#   ${catkin_EXPORTED_TARGETS}
# )

# target_link_libraries(detect_darknet_opencv_node
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${Open3D_LIBRARIES}
# )

# ## detect_obstacle_on_path_node
# add_executable(detect_obstacle_on_path_node
#   src/detect_obstacle_on_path_node.cpp
#   src/detect_obstacle_on_path.cpp
# )

# add_dependencies(detect_obstacle_on_path_node
#   ${catkin_EXPORTED_TARGETS}
# )

# target_link_libraries(detect_obstacle_on_path_node
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   )

# ## convert_obstacle_node
# add_executable(convert_obstacle_node
#   src/convert_obstacle_node.cpp
# )

# add_dependencies(convert_obstacle_node
#   obstacle_detector_generate_messages_cpp
#   ${catkin_EXPORTED_TARGETS}
# )

# target_link_libraries(convert_obstacle_node
#   ${catkin_LIBRARIES}
#   )


# ## Declare a C++ library
# add_library(track_people_cpp_nodelet
#   src/detect_darknet_opencv_nodelet.cpp
#   src/detect_darknet_opencv.cpp
#   src/detect_obstacle_on_path_nodelet.cpp
#   src/detect_obstacle_on_path.cpp
# )

# add_dependencies(track_people_cpp_nodelet
#   track_people_py_generate_messages_cpp
#   ${catkin_EXPORTED_TARGETS}
# )

# target_link_libraries(track_people_cpp_nodelet
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${Open3D_LIBRARIES}
# )

# if(catkin_EXPORTED_LIBRARIES)
#   add_dependencies(track_people_cpp_nodelet ${catkin_EXPORTED_LIBRARIES})
# endif()

# #############
# ## Install ##
# #############

# install(TARGETS
#   #track_people_cpp_nodelet
#   detect_obstacle_on_path_node
#   convert_obstacle_node
#   detect_darknet_opencv_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   )

# install(FILES nodelet.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#   )

# #############
# ## Testing ##
# #############
