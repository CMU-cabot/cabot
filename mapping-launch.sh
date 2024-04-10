#!/bin/bash

# Copyright (c) 2022  Carnegie Mellon University
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

function err {
    red >&2 "[ERROR] "$@
}
function red {
    echo -en "\033[31m" ## red
    echo $@
    echo -en "\033[0m" ## reset color
}
function blue {
    echo -en "\033[36m" ## blue
    echo $@
    echo -en "\033[0m" ## reset color
}
function snore() {
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read ${1:+-t "$1"} -u $_snore_fd || :
}
function help() {
    echo "Usage:"
    echo " this program runs recording for mapping"
    echo ""
    echo "-h          show this help"
    echo "-c          run realtime cartographer"
    echo "-a          use arduino for IMU topic"
    echo "-e          use esp32 for IMU topic"
    echo "-x          use xsens for IMU topic"
    echo "-o <name>   output prefix (default=mapping)"
    echo "-p <file>   post process the recorded bag"
    echo "-w          do not wait when rosbag play is finished"
    echo "-n          not use cached result for post processing"
    echo "-r <rate>   rosbag play rate for cartographer (default=1.0)"
    echo "-R <rate>   rosbag play rate for converting pointcloud2 to laserscan (default=1.0)"
    echo "-s          mapping for simulation"
    echo "-S          mapping for simulation and boot gazebo. only for gazebo"
    echo "-m          manipulate suitcase with controller. only for gazebo"
}

OUTPUT_PREFIX=${OUTPUT_PREFIX:=mapping}
RUN_CARTOGRAPHER=false
USE_ARDUINO=false
USE_ESP32=false
USE_XSENS=false
USE_VELODYNE=true
PLAYBAG_RATE_CARTOGRAPHER=1.0
PLAYBAG_RATE_PC2_CONVERT=1.0

post_process=
wait_when_rosbag_finish=1
no_cache=0
gazebo=0
boot=0
manipulate=0
container=

while getopts "hcaexo:p:wnr:R:sSm" arg; do
    case $arg in
    h)
        help
        exit
        ;;
    c)
        RUN_CARTOGRAPHER=true
        ;;
    a)
        USE_ARDUINO=true
        ;;
    e)
        USE_ESP32=true
        ;;
    x)
        USE_XSENS=true
        ;;
    o)
        OUTPUT_PREFIX=$OPTARG
        ;;
    p)
        post_process=$(realpath $OPTARG)
        ;;
    w)
        wait_when_rosbag_finish=0
        ;;
    n)
        no_cache=1
        ;;
    r)
        PLAYBAG_RATE_CARTOGRAPHER=$OPTARG
        ;;
    R)
        PLAYBAG_RATE_PC2_CONVERT=$OPTARG
        ;;
    s)
        gazebo=1
        ;;
    S)
        gazebo=1
        boot=1
        ;;
    m)
        manipulate=1
        ;;
    esac
done
shift $((OPTIND - 1))

pwd=$(pwd)
scriptdir=$(dirname $0)
cd $scriptdir
scriptdir=$(pwd)

if [[ -n $post_process ]]; then
    if [[ ! -e $post_process ]]; then
        err "could not find $post_process file"
        exit
    fi
    blue "processing $post_process"
    post_process_dir=$(dirname $post_process)
    post_process_name=$(basename $post_process)

    mkdir -p $scriptdir/docker/home/post_process
    if [[ $no_cache -eq 1 ]]; then
        rm -r $scriptdir/docker/home/post_process/${post_process_name}*
    fi
    if ls $scriptdir/docker/home/post_process/ | grep ${post_process_name}; then
        echo "${post_process_name} exists, pass copy"
    else
        cp -r $post_process $scriptdir/docker/home/post_process/
    fi
    QUIT_WHEN_ROSBAG_FINISH=true
    if [[ $wait_when_rosbag_finish -eq 1 ]]; then
        QUIT_WHEN_ROSBAG_FINISH=false
    fi
    export QUIT_WHEN_ROSBAG_FINISH
    export BAG_FILENAME=${post_process_name%.*}
    export PLAYBAG_RATE_CARTOGRAPHER
    export PLAYBAG_RATE_PC2_CONVERT
    if [[ $gazebo -eq 1 ]]; then
        export PROCESS_GAZEBO_MAPPING=1
    fi
    blue "docker compose -f docker-compose-mapping-post-process.yaml run post-process"
    docker compose -f docker-compose-mapping-post-process.yaml run post-process
    exit
fi

echo "OUTPUT_PREFIX=$OUTPUT_PREFIX"
echo "RUN_CARTOGRAPHER=$RUN_CARTOGRAPHER"
echo "USE_ARDUINO=$USE_ARDUINO"
echo "USE_ESP32=$USE_ESP32"
echo "USE_XSENS=$USE_XSENS"
echo "USE_VELODYNE=$USE_VELODYNE"
echo "Gazebo=$gazebo"
echo "USE_CONTROLLER=$manipulate"

cd $scriptdir
log_name=mapping_$(date +%Y-%m-%d-%H-%M-%S)
export ROS_LOG_DIR="/home/developer/.ros/log/${log_name}"
export OUTPUT_PREFIX=$OUTPUT_PREFIX
export RUN_CARTOGRAPHER=$RUN_CARTOGRAPHER
export USE_ARDUINO=$USE_ARDUINO
export USE_ESP32=$USE_ESP32
export USE_XSENS=$USE_XSENS
export USE_VELODYNE=$USE_VELODYNE

host_ros_log=$scriptdir/docker/home/.ros/log
host_ros_log_dir=$host_ros_log/$log_name
mkdir -p $host_ros_log_dir
ln -snf $host_ros_log_dir $host_ros_log/latest
blue "log dir is : $host_ros_log_dir"

# set profile arg to run wifi_scan service only if USE_ESP32 is false
if "$USE_ESP32"; then
    PROFILE_ARG=""
else
    PROFILE_ARG="--profile wifi_scan" # run wifi_scan service
fi

dcfile=docker-compose-mapping.yaml
if [[ $gazebo -eq 1 ]]; then
    dcfile=docker-compose-mapping-gazebo.yaml
    if [[ $boot -eq 1 ]]; then
        if ls | grep -q cabot-navigation; then
            cd cabot-navigation
            if [[ -z $(docker ps -q -f "name=cabot-navigation-navigation") ]]; then
                container="$container navigation"
            else
                echo "already boot cabot-navigation-navigation"
            fi
            if [[ -z $(docker ps -q -f "name=cabot-navigation-gui") ]]; then
                container="$container gui"
            else
                echo "already boot cabot-navigation-gui"
            fi
            if [[ -z $(docker ps -q -f "name=cabot-navigation-gazebo") ]]; then
                container="$container gazebo"
            else
                echo "already boot cabot-navigation-gazebo"
            fi
            if [[ -n $container ]]; then
                docker compose up -d $container
                sleep 6
            fi
            cd ..
        else
            echo "cannot find cabot-navigaton directory"
        fi
    fi
    if [[ $manipulate -eq 1 ]]; then
        if [[ -z $(docker ps -q -f "name=cabot-navigation-navigation") ]]; then
            echo "need to launch inside container of cabot-navigation for manipulate"
        else
            command="docker exec -itd cabot-navigation-navigation-1 bash -c 'source install/setup.bash && ros2 launch cabot_ui teleop_gamepad.launch.py'"
            eval $command
        fi
    fi
fi
docker compose -f $dcfile $PROFILE_ARG up -d &
snore 3
docker compose -f $dcfile logs -f >$host_ros_log_dir/docker-compose.log 2>&1 &
pid=$!

trap ctrl_c INT QUIT TERM

function ctrl_c() {
    if ls | grep -q cabot-navigation; then
        cd cabot-navigation
        if [[ $boot -eq 1 ]] && [[ -n $container ]]; then
            docker compose down $container
        fi
        cd ..
    fi
    docker compose -f $dcfile $PROFILE_ARG down
    exit 0
}

while [ 1 -eq 1 ]; do
    snore 1
done
