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

    printf '\033[2J\033[H'
    exit
}

function snore()
{
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read ${1:+-t "$1"} -u $_snore_fd || :
}

simulation=0
while getopts "s" arg; do
    case $arg in
	s)
	    simulation=1
	    ;;
    esac
done
shift $((OPTIND-1))

## private variables
pids=()

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`


# prepare ROS host_ws
mkdir -p $scriptdir/host_ws/src
cp -r $scriptdir/cabot_debug $scriptdir/host_ws/src
cd $scriptdir/host_ws
catkin_make
if [ $? -ne 0 ]; then
   exit $!
fi
source $scriptdir/host_ws/devel/setup.bash


## launch docker-compose
cd $scriptdir/docker
if [ $simulation -eq 1 ]; then
    docker-compose up &
else
    docker-compose -f docker-compose.yaml -f docker-compose-production.yaml up &
fi
pids+=($!)

# wait roscore
rosnode list
test=$?
while [ $test -eq 1 ]; do
    snore 0.1
    c=$((c+1))
    echo "wait roscore" $c
    rosnode list
    test=$?
done

# launch command_logger
cd $scriptdir/host_ws
source devel/setup.bash
roslaunch cabot_debug record_system_stat.launch &
pids+=($!)

while [ 1 -eq 1 ];
do
    snore 1
done
