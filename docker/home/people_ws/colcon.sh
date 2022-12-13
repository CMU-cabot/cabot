#!/bin/bash

colcon build \
       --cmake-args \
       -DCMAKE_BUILD_TYPE=Release \
       --packages-select \
       track_people_cpp \
       track_people_py \
       predict_people_py \
       queue_people_py \
       people_msgs \
       queue_msgs \
       cabot_people \
       cabot_site_coredo_3d \
       realsense2_camera \
       realsense2_camera_msgs \
       realsense2_description \
       --packages-ignore \
       cv_bridge \
       image_transport \
       --symlink-install
