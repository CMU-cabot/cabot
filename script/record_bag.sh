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

backend=${CABOT_ROSBAG_BACKEND:=sqlite3}
compression=${CABOT_ROSBAG_COMPRESSION:=message}

exclude_topics_file="rosbag2-exclude-topics.txt"
exclude_camera_topics="/.*/image_raw.*"
if [[ $record_cam -eq 1 ]]; then
    exclude_camera_topics='/.*/image_raw$'  # record compressed images
fi
lines=$(cat $scriptdir/${exclude_topics_file})
exclude_topics=$(echo -n "${lines}" | tr '\n' '|')
exclude_topics="${exclude_topics}|${exclude_camera_topics}"
#hidden_topics="--include-hidden-topics"
hidden_topics="" # workaround - if this is specified with '-s mcap', only a few topics can be recorded
qos_option="--qos-profile-overrides-path $scriptdir/rosbag2-qos-profile-overrides.yaml"
if [[ $compression == "none" ]]; then
    compression=""
else
    compression="--compression-mode $compression --compression-format zstd"
fi

interval=1000

if [[ -z $ROS_LOG_DIR ]]; then
    # for debug only
    com="ros2 bag record -s ${backend} ${compression} -p ${interval} -a -x \"${exclude_topics}\" ${hidden_topics} ${qos_option}&"
else
    com="ros2 bag record -s ${backend} ${compression} -p ${interval} -a -x \"${exclude_topics}\" ${hidden_topics} ${qos_option} -o $ROS_LOG_DIR/ros2_topics &"
fi
echo $com
eval $com
pid=$!

## wait until it is terminated by the user
while [ 1 -eq 1 ];
do
    # blue "snore"
    snore 1
done
