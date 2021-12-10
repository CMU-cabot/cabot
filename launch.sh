#!/bin/bash

# Copyright (c) 2021  IBM Corporation
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
    
    for pid in ${pids[@]}; do
	echo "killing $pid..."
	kill -s 2 $pid
    done

    ## we need to wait all nodes are terminated
    rlc=0
    while [ `ps -A | grep docker-compose | wc -l` -ne 0 ];
    do
	snore 1
	echo -ne "waiting containers are completely terminated ($rlc)"\\r
	rlc=$((rlc+1))
    done

    # stop docker containers
    cd $scriptdir
    eval "$com down"

    printf '\033[2J\033[H'
    exit
}

function red {
    echo -en "\033[31m"  ## red
    echo $1
    echo -en "\033[0m"  ## reset color
}
function blue {
    echo -en "\033[36m"  ## blue
    echo $1
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
    echo "-h          show this help"
    echo "-s          simulation mode"
    echo "-r          record camera (only robot mode)"
    echo "-p <name>   docker-compose's project name"
    echo "-n <name>   set log name prefix"
    echo "-v          verbose option"
}


simulation=0
record_cam=0
project_option=
log_prefix=cabot
verbose=0
while getopts "srhp:n:v" arg; do
    case $arg in
	s)
	    simulation=1
	    ;;
	h)
	    help
	    exit
	    ;;
	r)
	    record_cam=1
	    ;;
	p)
	    project_option="-p $OPTARG"
	    ;;
	n)
	    log_prefix=$OPTARG
	    ;;
	v)
	    verbose=1
	    ;;
    esac
done
shift $((OPTIND-1))

log_name=${log_prefix}_`date +%Y-%m-%d-%H-%M-%S`
export ROS_LOG_DIR="/home/developer/.ros/log/${log_name}"
export CABOT_LOG_NAME=$log_name

## private variables
pids=()

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

# prepare ROS host_ws
blue "build host_ws"
mkdir -p $scriptdir/host_ws/src
cp -r $scriptdir/cabot_debug $scriptdir/host_ws/src
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
com=
if [ $simulation -eq 1 ]; then
    blue "launch docker for simulation"
    com="docker-compose $project_option"
else
    blue "change nvidia gpu and bluetooth settings"
    # if need to reduce GPU computing wattage
    $scriptdir/tools/change_nvidia-smi_settings.sh
    # if use CaBot-app, improve BLE connection stability
    $scriptdir/tools/change_supervision_timeout.sh

    blue "launch docker for production"
    if [ $record_cam -eq 1 ]; then
	blue "recording realsense camera images"
	com="docker-compose $project_option -f docker-compose.yaml -f docker-compose-production.yaml -f docker-compose-production-record-camera.yaml"
    else
	com="docker-compose $project_option -f docker-compose.yaml -f docker-compose-production.yaml"
    fi
fi
host_ros_log_dir=$scriptdir/docker/home/.ros/log/$log_name
mkdir -p $host_ros_log_dir
if [ $verbose -eq 0 ]; then
    eval "$com --ansi never up > $host_ros_log_dir/docker-compose.log &"
else
    eval "$com up | tee $host_ros_log_dir/docker-compose.log &"
fi
pids+=($!)

# wait roscore
snore 5
rosnode list 2&> /dev/null
test=$?
while [ $test -eq 1 ]; do
    snore 5
    c=$((c+1))
    echo "wait roscore" $c
    rosnode list 2&> /dev/null
    test=$?
done

# launch command_logger with the host ROS
cd $scriptdir/host_ws
source devel/setup.bash
export ROS_LOG_DIR=$host_ros_log_dir
if [ $verbose -eq 0 ]; then
    roslaunch cabot_debug record_system_stat.launch > /dev/null &
else
    roslaunch cabot_debug record_system_stat.launch &
fi
pids+=($!)

while [ 1 -eq 1 ];
do
    snore 1
done
