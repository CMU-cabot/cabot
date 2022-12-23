#!/bin/bash

colcon build \
       --cmake-args \
       -DCMAKE_BUILD_TYPE=Release \
       --packages-ignore \
       cv_bridge \
       image_transport \
       image_geometry \
       camera_calibration_parsers \
       camera_info_manager \
       compressed_depth_image_transport \
       polled_camera \
       theora_image_transport \
       compressed_image_transport \
       ddynamic_reconfigure \
       image_common \
       image_transport_plugins \
       obstacle_detector \
       mf_localization_msgs \
       opencv_tests \
       vision_opencv \
       queue_utils_py \
       people_tracking_filter \
       face_detector \
       leg_detector \
       people_velocity_tracker \
       people \
       --symlink-install
       # --packages-select \
       # track_people_cpp \
       # track_people_py \
       # predict_people_py \
       # queue_people_py \
       # people_msgs \
       # queue_msgs \
       # cabot_people \
       # cabot_site_coredo_3d \
       # realsense2_camera \
       # realsense2_camera_msgs \
       # realsense2_description \
