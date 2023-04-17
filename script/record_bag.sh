#!/bin/bash

echo "recording to $ROS_LOG_DIR"
source install/setup.bash

record_cam=0
while getopts "r" arg; do
    case $arg in
        r)
            record_cam=1
            ;;
    esac
done
shift $((OPTIND-1))

bag_include_pat=".*"
bag_exclude_pat="/carto.*|/gazebo.*"
bag_exclude_pat="${bag_exclude_pat}|.*costmap.*|.*transition_event"
bag_exclude_pat="${bag_exclude_pat}|/velodyne_packets|/velodyne_points_cropped|/scan_matched_points2"

if [[ $record_cam -eq 1 ]]; then
    bag_exclude_pat="${bag_exclude_pat}|/[^/]+/(aligned_depth_to_color/|color/|depth/|extrinsics/|infra1/|infra2/)[^/]*"
else
    bag_exclude_pat="${bag_exclude_pat}|/[^/]+/(aligned_depth_to_color/|color/|depth/|extrinsics/|infra1/|infra2/).*"
fi

qos_option="--qos-profile-overrides-path rosbag2-qos-profile-overrides.yaml --include-hidden-topics"

ros2 bag record -e "$bag_include_pat" -x "$bag_exclude_pat" -o $ROS_LOG_DIR/ros2_topics $qos_option
