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
    echo "trap play_bag.sh"
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

# environment variables
: ${CABOT_SHOW_ROS2_LOCAL_RVIZ:=0}

source $scriptdir/../install/setup.bash

rate=1.0
start=0.01
while getopts "r:s:m:" arg; do
    case $arg in
        r)
            rate=$OPTARG
            ;;
	s)
	    start=$OPTARG
	    ;;
    esac
done
shift $((OPTIND-1))

bag=$1
if [[ -z $bag ]]; then
    echo "Usage: $0 <bag_file>"
    exit 1
fi
echo "play $bag"


if (( $(echo "$start > 0.01" | bc -l) )); then
    map=$(ros2 run cabot_debug print_topics.py -f $bag -s $start -1 -r -t /current_map_filename 2> /dev/null)
    temp_str="${map#package://}"
    package="${temp_str%%/*}"
    prefix=$(ros2 pkg prefix $package)
    path="${temp_str#*/}"
    map_path="map:=$prefix/share/$package/$path"

    tf_frame="frame:=$(ros2 run cabot_debug print_topics.py -f $bag -s $start -1 -r -t /current_frame)"
    
    temp_file="$(mktemp)"
    ros2 run cabot_debug print_topics.py -f $bag -1 -r -t /robot_description > $temp_file
    temp_file="robot:=$temp_file"
fi

com="ros2 launch cabot_debug play_bag.launch.py \
   bagfile:=$bag \
   start:=$start \
   rate:=$rate \
   $map_path \
   $temp_file \
   $tf_frame \
   show_local_rviz:=$CABOT_SHOW_ROS2_LOCAL_RVIZ &"

echo $com
eval $com
pid=$!

if (( $(echo "$start > 0.01" | bc -l) )); then
    ros2 run cabot_debug print_topics.py -f $bag -d $start -t /global_costmap/costmap -P
fi

## wait until it is terminated by the user
while [ 1 -eq 1 ];
do
    # blue "snore"
    snore 1
done
