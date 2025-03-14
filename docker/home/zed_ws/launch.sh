#!/bin/bash

trap 'SIGINT' cleanup

function cleanup() {
    echo "Stopping all processes..."
    for pid in "${pids[@]}"; do
        kill -SIGINT "$pid"
    done
    wait
    exit 0
}

function snore()
{
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read ${1:+-t "$1"} -u $_snore_fd || :
}

log_name=${CABOT_LOG_NAME:-"please-specify-a-log-name"}
pids=()

ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2 &
pids+=($!)

snore 5

ros2 service call /zed/zed_node/start_svo_rec zed_msgs/srv/StartSvoRec \
   "bitrate: 60000
compression_mode: 1
target_framerate: 30
input_transcode: false
svo_filename: '/recordings/${log_name}_data3d.svo'" &
pids+=($!)


ros2 bag record -o /recordings/${log_name}_ros2bag \
    /zed/zed_node/left/camera_info \
    /zed/zed_node/right/camera_info \
    /zed/zed_node/stereo/image_rect_color/compressed \
    /zed/zed_node/depth/camera_info \
    /zed/zed_node/depth/depth_registered/compressedDepth
#    /zed/zed_node/point_cloud/cloud_registered/zstd \
#    /zed/zed_node/confidence/confidence_map


# all zed topics
# /zed/joint_states
# /zed/plane
# /zed/plane_marker
# /zed/robot_description
# /zed/zed_node/atm_press
# /zed/zed_node/confidence/confidence_map
# /zed/zed_node/depth/camera_info
# /zed/zed_node/depth/depth_info
# /zed/zed_node/depth/depth_registered
# /zed/zed_node/depth/depth_registered/compressed
# /zed/zed_node/depth/depth_registered/compressedDepth
# /zed/zed_node/depth/depth_registered/ffmpeg
# /zed/zed_node/depth/depth_registered/theora
# /zed/zed_node/disparity/disparity_image
# /zed/zed_node/imu/data
# /zed/zed_node/imu/data_raw
# /zed/zed_node/imu/mag
# /zed/zed_node/left/camera_info
# /zed/zed_node/left/image_rect_color
# /zed/zed_node/left/image_rect_color/compressed
# /zed/zed_node/left/image_rect_color/compressedDepth
# /zed/zed_node/left/image_rect_color/ffmpeg
# /zed/zed_node/left/image_rect_color/theora
# /zed/zed_node/left_cam_imu_transform
# /zed/zed_node/left_gray/camera_info
# /zed/zed_node/left_gray/image_rect_gray
# /zed/zed_node/left_gray/image_rect_gray/compressed
# /zed/zed_node/left_gray/image_rect_gray/compressedDepth
# /zed/zed_node/left_gray/image_rect_gray/ffmpeg
# /zed/zed_node/left_gray/image_rect_gray/theora
# /zed/zed_node/left_raw/camera_info
# /zed/zed_node/left_raw/image_raw_color
# /zed/zed_node/left_raw/image_raw_color/compressed
# /zed/zed_node/left_raw/image_raw_color/compressedDepth
# /zed/zed_node/left_raw/image_raw_color/ffmpeg
# /zed/zed_node/left_raw/image_raw_color/theora
# /zed/zed_node/left_raw_gray/camera_info
# /zed/zed_node/left_raw_gray/image_raw_gray
# /zed/zed_node/left_raw_gray/image_raw_gray/compressed
# /zed/zed_node/left_raw_gray/image_raw_gray/compressedDepth
# /zed/zed_node/left_raw_gray/image_raw_gray/ffmpeg
# /zed/zed_node/left_raw_gray/image_raw_gray/theora
# /zed/zed_node/odom
# /zed/zed_node/path_map
# /zed/zed_node/path_odom
# /zed/zed_node/point_cloud/cloud_registered
# /zed/zed_node/point_cloud/cloud_registered/draco
# /zed/zed_node/point_cloud/cloud_registered/zlib
# /zed/zed_node/point_cloud/cloud_registered/zstd
# /zed/zed_node/pose
# /zed/zed_node/pose/status
# /zed/zed_node/pose_with_covariance
# /zed/zed_node/rgb/camera_info
# /zed/zed_node/rgb/image_rect_color
# /zed/zed_node/rgb/image_rect_color/compressed
# /zed/zed_node/rgb/image_rect_color/compressedDepth
# /zed/zed_node/rgb/image_rect_color/ffmpeg
# /zed/zed_node/rgb/image_rect_color/theora
# /zed/zed_node/rgb_gray/camera_info
# /zed/zed_node/rgb_gray/image_rect_gray
# /zed/zed_node/rgb_gray/image_rect_gray/compressed
# /zed/zed_node/rgb_gray/image_rect_gray/compressedDepth
# /zed/zed_node/rgb_gray/image_rect_gray/ffmpeg
# /zed/zed_node/rgb_gray/image_rect_gray/theora
# /zed/zed_node/rgb_raw/camera_info
# /zed/zed_node/rgb_raw/image_raw_color
# /zed/zed_node/rgb_raw/image_raw_color/compressed
# /zed/zed_node/rgb_raw/image_raw_color/compressedDepth
# /zed/zed_node/rgb_raw/image_raw_color/ffmpeg
# /zed/zed_node/rgb_raw/image_raw_color/theora
# /zed/zed_node/rgb_raw_gray/camera_info
# /zed/zed_node/rgb_raw_gray/image_raw_gray
# /zed/zed_node/rgb_raw_gray/image_raw_gray/compressed
# /zed/zed_node/rgb_raw_gray/image_raw_gray/compressedDepth
# /zed/zed_node/rgb_raw_gray/image_raw_gray/ffmpeg
# /zed/zed_node/rgb_raw_gray/image_raw_gray/theora
# /zed/zed_node/right/camera_info
# /zed/zed_node/right/image_rect_color
# /zed/zed_node/right/image_rect_color/compressed
# /zed/zed_node/right/image_rect_color/compressedDepth
# /zed/zed_node/right/image_rect_color/ffmpeg
# /zed/zed_node/right/image_rect_color/theora
# /zed/zed_node/right_gray/camera_info
# /zed/zed_node/right_gray/image_rect_gray
# /zed/zed_node/right_gray/image_rect_gray/compressed
# /zed/zed_node/right_gray/image_rect_gray/compressedDepth
# /zed/zed_node/right_gray/image_rect_gray/ffmpeg
# /zed/zed_node/right_gray/image_rect_gray/theora
# /zed/zed_node/right_raw/camera_info
# /zed/zed_node/right_raw/image_raw_color
# /zed/zed_node/right_raw/image_raw_color/compressed
# /zed/zed_node/right_raw/image_raw_color/compressedDepth
# /zed/zed_node/right_raw/image_raw_color/ffmpeg
# /zed/zed_node/right_raw/image_raw_color/theora
# /zed/zed_node/right_raw_gray/camera_info
# /zed/zed_node/right_raw_gray/image_raw_gray
# /zed/zed_node/right_raw_gray/image_raw_gray/compressed
# /zed/zed_node/right_raw_gray/image_raw_gray/compressedDepth
# /zed/zed_node/right_raw_gray/image_raw_gray/ffmpeg
# /zed/zed_node/right_raw_gray/image_raw_gray/theora
# /zed/zed_node/stereo/image_rect_color
# /zed/zed_node/stereo/image_rect_color/compressed
# /zed/zed_node/stereo/image_rect_color/compressedDepth
# /zed/zed_node/stereo/image_rect_color/ffmpeg
# /zed/zed_node/stereo/image_rect_color/theora
# /zed/zed_node/stereo_raw/image_raw_color
# /zed/zed_node/stereo_raw/image_raw_color/compressed
# /zed/zed_node/stereo_raw/image_raw_color/compressedDepth
# /zed/zed_node/stereo_raw/image_raw_color/ffmpeg
# /zed/zed_node/stereo_raw/image_raw_color/theora
# /zed/zed_node/temperature/imu
# /zed/zed_node/temperature/left
# /zed/zed_node/temperature/right
