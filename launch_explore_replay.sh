#!/bin/bash

# Copyright (c) 2023  Carnegie Mellon University, IBM Corporation, and others
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

trap ctrl_c INT QUIT TERM

function ctrl_c() {
    ## kill recoding node first ensure the recording is property finished
    # echo "killing recording nodes..."
    rosnode list 2> /dev/null | grep record | while read -r line
    do
        rosnode kill $line > /dev/null 2>&1
    done

    if [ $verbose -eq 1 ]; then
	$dccom down
    else
	$dccom down > /dev/null 2>&1
    fi

    for pid in ${pids[@]}; do
        signal=2
	if [[ "${termpids[*]}" =~ "$pid" ]]; then
            signal=15
        fi
        if [ $verbose -eq 1 ]; then
            echo "killing $0 $pid"
            kill -s $signal $pid
        else
            kill -s $signal $pid > /dev/null 2>&1
        fi
    done
    for pid in ${pids[@]}; do
        if [ $verbose -eq 1 ]; then
            while kill -0 $pid; do
                echo "waiting $0 $pid"
                snore 1
            done
        else
            while kill -0 $pid > /dev/null 2>&1; do
                snore 1
            done
        fi
    done
    exit
}
function err {
    >&2 red "[ERROR] "$@
}
function red {
    echo -en "\033[31m"  ## red
    echo $@
    echo -en "\033[0m"  ## reset color
}
function blue {
    echo -en "\033[36m"  ## blue
    echo $@
    echo -en "\033[0m"  ## reset color
}
function snore()
{
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read ${1:+-t "$1"} -u $_snore_fd || :
}

function help()
{
    echo "Usage:"
    echo "-h             show this help"
    echo "-p <name>      docker-compose's project name"
    echo "-v             verbose option"
    echo "-b <bagfile>   bagfile"
}


project_option=
verbose=0
bagfile=

while getopts "hp:vb:" arg; do
    case $arg in
        h)
            help
            exit
            ;;
        p)
            project_option="-p $OPTARG"
            ;;
        v)
            verbose=1
            ;;
        b)
            bagfile=$OPTARG
            ;;
    esac
done
shift $((OPTIND-1))

if [ "$bagfile" == "" ]; then
    echo "Please specify bagfile"
    help
    exit
fi

## private variables
pids=()
termpids=()

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`
source $scriptdir/.env

## check required environment variables
error=0
if [ -z $CABOT_MODEL ]; then
    err "CABOT_MODEL: environment variable should be specified (ex. cabot2-gt1"
    error=1
fi
if [ -z $CABOT_NAME ]; then
    err "CABOT_NAME : environment variable should be specified (ex. alpha"
    error=1
fi

if [ $error -eq 1 ]; then
   exit 1
fi

log_name=${log_prefix}_`date +%Y-%m-%d-%H-%M-%S`
export ROS_LOG_DIR="/home/developer/.ros/log/${log_name}"
export CABOT_LOG_NAME=$log_name

# prepare ROS host_ws
blue "build host_ws"
cd $scriptdir/host_ws
if [ $verbose -eq 0 ]; then
    catkin_make > /dev/null
else
    catkin_make
fi
if [ $? -ne 0 ]; then
   exit $!
fi
source $scriptdir/host_ws/devel/setup.bash

## launch docker-compose
cd $scriptdir
dcfile=docker-compose-explore.yaml

if [ ! -e $dcfile ]; then
    err "There is not $dcfile"
    exit
fi

dccom="docker-compose $project_option -f $dcfile"
host_ros_log_dir=$scriptdir/docker/home/.ros/log/$log_name
blue "log dir is : $host_ros_log_dir"
mkdir -p $host_ros_log_dir

if [ $verbose -eq 0 ]; then
    com2="bash -c \"$dccom --ansi never up --no-build ros2 bridge \" > $host_ros_log_dir/docker-compose.log &"
else
    com2="bash -c \"$dccom up --no-build ros2 bridge \" | tee $host_ros_log_dir/docker-compose.log &"
fi
if [ $verbose -eq 1 ]; then
    blue "$com2"
fi
eval $com2
dcpid=($!)
pids+=($!)
blue "[$dcpid] $dccom up"

if [ $verbose -eq 0 ]; then
    com2="bash -c \"$dccom run ros1 launch.sh -u -E -B \" > $host_ros_log_dir/docker-compose.log &"
else
    com2="bash -c \"$dccom run ros1 launch.sh -u -E -B \" | tee $host_ros_log_dir/docker-compose.log &"
fi
if [ $verbose -eq 1 ]; then
    blue "$com2"
fi
eval $com2
dcpid=($!)
pids+=($!)
blue "[$dcpid] $dccom up"

if [ $verbose -eq 0 ]; then
    com2="bash -c \"$dccom run explore /launch.sh replay \" > $host_ros_log_dir/docker-compose.log &"
else
    com2="bash -c \"$dccom run explore /launch.sh replay \" | tee $host_ros_log_dir/docker-compose.log &"
fi
if [ $verbose -eq 1 ]; then
    blue "$com2"
fi
eval $com2
dcpid=($!)
pids+=($!)
blue "[$dcpid] $dccom up"

# wait roscore
snore 5
rosnode list > /dev/null 2>&1
test=$?
while [ $test -eq 1 ]; do
    snore 5

    # check docker-compose process is running
    kill -0 $dcpid > /dev/null 2>&1
    if [ $? -eq 1 ]; then
        exit
    fi
    c=$((c+1))
    echo "wait roscore" $c
    rosnode list > /dev/null 2>&1
    test=$?
done

# start bag replay
echo "start play bag file $bagfile"
setsid xterm -e rosbag play $bagfile --pause --topics /clock /tf /tf_static /velodyne_points /cabot/imu/data /cabot_explore/explore_event; read &

while [ 1 -eq 1 ];
do
    snore 1
done