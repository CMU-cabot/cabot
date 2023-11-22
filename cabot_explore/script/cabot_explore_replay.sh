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

## termination hook
trap ctrl_c INT QUIT TERM

function ctrl_c() {
    ## killing all nodes
    # comment out because this will kill remote simulator nodes
    # echo "killing all ros nodes..."
    # rosnode kill -a

    for pid in ${pids[@]}; do
        echo "killing $pid..."
        kill -s 2 $pid
    done

    ## we need to wait gazebo is terminated
    rlc=0
    while [ `ps -A | grep roslaunch | wc -l` -ne 0 ];
    do
        snore 1
        ps -A
        echo -ne "waiting process is completely terminated ($rlc)"\\r
        rlc=$((rlc+1))
    done

    echo \\n
    exit
}

function snore()
{
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read ${1:+-t "$1"} -u $_snore_fd || :
}

## private variables
pids=()

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

### default variables

## debug
debug=0
command=''
commandpost='&'

# for localization
points2_topic='/velodyne_points'
imu_topic='/cabot/imu/data'

use_sim_time=true

### usage print function
function usage {
    echo "Usage"
    echo "    run this script after running cabot.sh in another terminal"
    echo "ex)"
    echo $0 "-O"
    echo ""
    echo "-h                       show this help"
    echo "-d                       debug"
    exit
}

while getopts "hd" arg; do
    case $arg in
    h)
        usage
        exit
        ;;
    d)
        debug=1
        command="setsid xterm -e '"
        commandpost=";read'&"
        ;;
  esac
done
shift $((OPTIND-1))

## debug output
echo "Debug         : $debug ($command, $commandpost)"

rosnode list
test=$?
while [ $test -eq 1 ]; do
    snore 0.1
    c=$((c+1))
    echo "wait roscore" $c
    rosnode list
    test=$?
done

### launch explore replay node
echo "launch explore_replay.launch"
com="$command roslaunch route_explore explore_replay.launch \
    $commandpost"
eval $com
pids+=($!)

### wait IMU topic for avoiding cartgrapher error "Check failed: (orientation_ * gravity_vector_).z() > 0" on real robot.
until rostopic echo -n1 $imu_topic &> /dev/null; do
    echo "waiting for IMU topic becomes available before launching online cartographer"
    sleep 1
done

### launch cartographer node for explore
echo "launch cartographer_explore.launch"
com="$command roslaunch cabot_explore cartographer_explore.launch \
    use_sim_time:=$use_sim_time \
    configuration_basename:=cartographer_2d_explore.lua \
    points2_topic:=$points2_topic \
    imu_topic:=$imu_topic \
    $bagoption \
    $commandpost"
eval $com
pids+=($!)

## wait until it is terminated by the user
while [ 1 -eq 1 ];
do
    snore 1
done
