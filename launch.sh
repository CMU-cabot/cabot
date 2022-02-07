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
    # echo "killing recording nodes..."
    rosnode list | grep record | while read -r line
    do
        rosnode kill $line
    done


    for pid in ${pids[@]}; do
        signal=2
        if [ $pid -eq $jlpid ]; then
            signal=15
        fi
        if [ $verbose -eq 1 ]; then
            echo "killing $0 $pid"
            kill -s $signal $pid
        else
            kill -s $signal $pid 2&> /dev/null
        fi
    done
    for pid in ${pids[@]}; do
        if [ $verbose -eq 1 ]; then
            while kill -0 $pid; do
                echo "waiting $0 $pid"
                snore 1
            done
        else
            while kill -0 $pid 2&> /dev/null; do
                snore 1
            done
        fi
    done
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
    echo "-d          do not record"
    echo "-r          record camera"
    echo "-p <name>   docker-compose's project name"
    echo "-n <name>   set log name prefix"
    echo "-v          verbose option"
}


simulation=0
do_not_record=0
record_cam=0
no_gpu=0
project_option=
log_prefix=cabot
verbose=0
while getopts "hsdrp:n:v" arg; do
    case $arg in
        s)
            simulation=1
            ;;
        h)
            help
            exit
            ;;
        d)
            do_not_record=1
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

if [ -z `which nvidia-smi` ]; then
    no_gpu=1
fi

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

## run script to change settings
if [ $simulation -eq 0 ]; then
    blue "change bluetooth settings"
    $scriptdir/tools/change_supervision_timeout.sh
    if [ $no_gpu -eq 0 ]; then
        blue "change nvidia gpu settings"
        $scriptdir/tools/change_nvidia-smi_settings.sh
    fi
fi

## launch docker-compose
cd $scriptdir
dcfile=

if [ $simulation -eq 1 ]; then
    if [ $no_gpu -eq 1 ]; then
        dcfile=docker-compose-nuc.yaml
    else
        dcfile=docker-compose.yaml
    fi
else
    if [ $no_gpu -eq 1 ]; then
        dcfile=docker-compose-nuc-production.yaml
    else
        dcfile=docker-compose-production.yaml
    fi
fi

dccom="docker-compose $project_option -f $dcfile"
host_ros_log_dir=$scriptdir/docker/home/.ros/log/$log_name
blue "log dir is : $host_ros_log_dir"
blue "docker-compose up"
mkdir -p $host_ros_log_dir
if [ $verbose -eq 0 ]; then
    com2="bash -c \"$dccom --ansi never up --no-build\" 2&> $host_ros_log_dir/docker-compose.log &"
else
    com2="bash -c \"$dccom up --no-build \" 2>&1 | tee $host_ros_log_dir/docker-compose.log &"
fi
if [ $verbose -eq 1 ]; then
    blue "$com2"
fi
eval $com2
dcpid=($!)
pids+=($!)

# wait roscore
snore 5
rosnode list 2&> /dev/null
test=$?
while [ $test -eq 1 ]; do
    snore 5

    # check docker-compose process is running
    kill -0 $dcpid 2&> /dev/null
    if [ $? -eq 1 ]; then
        exit
    fi
    c=$((c+1))
    echo "wait roscore" $c
    rosnode list 2&> /dev/null
    test=$?
done

## launch jetson
if [ $no_gpu -eq 1 ]; then
    blue "launch jetson"
    source $scriptdir/.env

    : "${CABOT_JETSON_USER:=cabot}"

    jlpid=0
    if [ ! -z "$CABOT_JETSON_CONFIG" ]; then
        simopt=
        if [ $simulation -eq 1 ]; then simopt="-s"; fi

        if [ $verbose -eq 1 ]; then
            com="./jetson-launch.sh -v -u $CABOT_JETSON_USER -c \"$CABOT_JETSON_CONFIG\" $simopt &"
        else
            com="./jetson-launch.sh -v -u $CABOT_JETSON_USER -c \"$CABOT_JETSON_CONFIG\" $simopt 2&> $host_ros_log_dir/jetson-launch.log &"
        fi

        if [ $verbose -eq 1 ]; then
            blue "$com"
        fi
        eval $com
        jlpid=$!
        pids+=($jlpid)
    fi
fi

# launch command_logger with the host ROS
cd $scriptdir/host_ws
source devel/setup.bash
export ROS_LOG_DIR=$host_ros_log_dir
if [ $verbose -eq 0 ]; then
    roslaunch cabot_debug record_system_stat.launch 2&> /dev/null &
else
    roslaunch cabot_debug record_system_stat.launch &
fi
pids+=($!)


if [ $do_not_record -eq 0 ]; then
    blue "recording ROS1 topics"
    bag_exclude_pat=".*(image_raw|carto|gazebo|^/map).*"

    if [ $record_cam -eq 1 ]; then
        blue "recording realsense camera images"
        bag_exclude_pat=".*(depth/image_raw|infra./image_raw|aligned_depth_to_color/image_raw|carto|gazebo|^/map).*"
    fi

    if [ $verbose -eq 0 ]; then
        com="rosbag record -a -x \"$bag_exclude_pat\" -O $ROS_LOG_DIR/ros1_topics.bag &"
    else
        com="rosbag record -a -x \"$bag_exclude_pat\" -O $ROS_LOG_DIR/ros1_topics.bag &"
    fi
    blue "$com"
    eval $com
    pids+=($!)
else
    blue "do not record ROS1 topics"
fi

while [ 1 -eq 1 ];
do
    snore 1
done
