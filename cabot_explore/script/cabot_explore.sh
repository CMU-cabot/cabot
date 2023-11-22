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
#minimum=0
debug=0
command=''
commandpost='&'

# for localization
points2_topic='/velodyne_points'
imu_topic='/cabot/imu/data'

gazebo=0
use_sim_time=false

show_rviz=1

bagfile=
bagdir=~/.ros

### usage print function
function usage {
    echo "Usage"
    echo "    run this script after running cabot.sh in another terminal"
    echo "ex)"
    echo $0 "-O"
    echo ""
    echo "-h                       show this help"
    echo "-d                       debug"
    echo "-s                       specify its on simulation (gazebo)"
    echo "-O                       performance (no rviz)"
    echo "-b <name>                bag file name"
    exit
}

while getopts "hdsOb:" arg; do
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
    s)
        gazebo=1
        use_sim_time=true
        ;;
    O)
        show_rviz=0
        ;;
    b)
        bagfile=$OPTARG
        ;;
  esac
done
shift $((OPTIND-1))

## debug output
echo "Debug         : $debug ($command, $commandpost)"
echo "Simulation    : $gazebo"

# roscore
#rosnode list
#if [ $? -eq 1 ]; then
#    eval "$command roscore $commandpost"
#    pids+=($!)
#fi

rosnode list
test=$?
while [ $test -eq 1 ]; do
    snore 0.1
    c=$((c+1))
    echo "wait roscore" $c
    rosnode list
    test=$?
done

### wait IMU topic for avoiding cartgrapher error "Check failed: (orientation_ * gravity_vector_).z() > 0" on real robot.
until rostopic echo -n1 $imu_topic &> /dev/null; do
    echo "waiting for IMU topic becomes available before launching online cartographer"
    sleep 1
done

### launch rviz
if [ $show_rviz -eq 1 ]; then
    echo "launch rviz"
    eval "$command rosrun rviz rviz -d $scriptdir/cabot_explore.rviz $commandpost"
    pids+=($!)
fi


bagoption=""
if [ "$bagfile" != "" ]; then
    if [ ! -z $ROS_HOME ]; then
        bagdir=$ROS_HOME/log
    fi
    if [ ! -z $ROS_LOG_DIR ]; then
        bagdir=$ROS_LOG_DIR
    fi
    mkdir -p $bagdir
    bagoption="bagfile:=$bagdir/$bagfile"
fi

### launch cartographer nonde for explore
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

### launch explore node
echo "launch route_explore.launch"
com="$command roslaunch route_explore explore_manager.launch \
    costmap_topic:=/map \
    $commandpost"
eval $com
pids+=($!)

## wait until it is terminated by the user
while [ 1 -eq 1 ];
do
    snore 1
done
