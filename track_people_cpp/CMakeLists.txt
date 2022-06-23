cmake_minimum_required(VERSION 3.0.2)
project(track_people_cpp)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  nodelet
  roscpp
  sensor_msgs
  std_msgs
  tf2_ros
  track_people_py
  diagnostic_updater
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Open3D REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
  cv_bridge
  nodelet
  roscpp
  sensor_msgs
  std_msgs
  tf2_ros
  track_people_py
  diagnostic_updater

  DEPENDS
  EIGEN3
  Open3D
  OpenCV

  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Open3D_INCLUDE_DIRS}  
  ${OpenCV_INCLUDE_DIRS}  
)

add_executable(detect_darknet_opencv_node
  src/detect_darknet_opencv_node.cpp
  src/detect_darknet_opencv.cpp
)

add_dependencies(detect_darknet_opencv_node
  track_people_py_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(detect_darknet_opencv_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Open3D_LIBRARIES}
)

## Declare a C++ library
add_library(track_people_cpp_nodelet
  src/detect_darknet_opencv_nodelet.cpp
  src/detect_darknet_opencv.cpp
)

add_dependencies(track_people_cpp_nodelet
  track_people_py_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(track_people_cpp_nodelet
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Open3D_LIBRARIES}
)

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(track_people_cpp_nodelet ${catkin_EXPORTED_LIBRARIES})
endif()

#############
## Install ##
#############

install(TARGETS
  #track_people_cpp_nodelet
  detect_darknet_opencv_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

#############
## Testing ##
#############
