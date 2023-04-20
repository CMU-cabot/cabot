#!/bin/bash

# change directory to where this script exists
pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`
pid=

# load utility functions
source $scriptdir/cabot_util.sh

trap signal INT TERM

function signal() {
    echo "trap record_bag.sh"
    kill -2 $pid

    while kill -0 $pid 2> /dev/null; do
	if [[ $count -eq 15 ]]; then
	    blue "escalate to SIGTERM $pid"
	    com="kill -TERM $pid"
	    eval $com
	fi
	if [[ $count -eq 30 ]]; then
	    blue "escalate to SIGKILL $pid"
	    com="kill -KILL $pid"
	    eval $com
	fi
        echo "waiting $0 $pid"
        snore 1
	count=$((count+1))
    done

    exit
}

echo "recording to $ROS_LOG_DIR"
source $scriptdir/../install/setup.bash

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

qos_option="--qos-profile-overrides-path $scriptdir/../rosbag2-qos-profile-overrides.yaml --include-hidden-topics"

ros2 bag record -e "$bag_include_pat" -x "$bag_exclude_pat" -o $ROS_LOG_DIR/ros2_topics $qos_option &
pid=$!

## wait until it is terminated by the user
while [ 1 -eq 1 ];
do
    # blue "snore"
    snore 1
done
