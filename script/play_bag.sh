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

source $scriptdir/../install/setup.bash

rate=1.0
start=0.01
while getopts "r:s:" arg; do
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

com="ros2 launch cabot_debug play_bag.launch.py bagfile:=$bag start:=$start rate:=$rate"
echo $com
eval $com
pid=$!

## wait until it is terminated by the user
while [ 1 -eq 1 ];
do
    # blue "snore"
    snore 1
done
