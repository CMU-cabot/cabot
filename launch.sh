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
start=`date +%s.%N`

trap ctrl_c INT QUIT TERM

terminating=0
launched=0

function ctrl_c() {
    red "catch the signal"
    user=$1
    terminating=1
    cd $scriptdir
    if [[ ! -z $dccom ]]; then
	while [[ $launched -lt 5 ]]; do
	    snore 1
	    launched=$((launched+1))
	done

	red "$dccom down"
	if [ $verbose -eq 1 ]; then
	    $dccom down &
	else
	    $dccom down > /dev/null 2>&1 &
	fi
	count=1
	snore 3  # need to wait a bit after docker-compose down, otherwise it can hung up
	red "Waiting docker-compose downs the all containers ($count)"
	result=$($dccom ps -q | wc -l)
	while [[ $result -gt 0 ]];
	do
	    count=$((count+1))
	    snore 3
	    red "Waiting docker-compose downs the all containers ($count)"
	    result=$($dccom ps -q | wc -l)
	done
    fi
    if [[ ! -z $bag_dccom ]]; then
	red "$bag_dccom down"
	if [ $verbose -eq 1 ]; then
	    $bag_dccom down
	else
	    $bag_dccom down > /dev/null 2>&1
	fi
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
    echo "-h          show this help"
    echo "-s          simulation mode"
    echo "-d          do not record"
    echo "-r          record camera"
    echo "-p <name>   docker-compose's project name"
    echo "-n <name>   set log name prefix"
    echo "-v          verbose option"
    echo "-c <name>   config name (default=) docker-compose(-<name>)(-production).yaml will use"
    echo "            if there is no nvidia-smi and config name is not set, automatically set to 'nuc'"
    echo "-3          equivalent to -c rs3"
    echo "-S          record screen cast"
    echo "-y          do not confirm"
}


simulation=0
do_not_record=0
record_cam=0
use_nuc=0
nvidia_gpu=0
project_option=
log_prefix=cabot
verbose=0
config_name=
local_map_server=0
debug=0
reset_all_realsence=0
screen_recording=0
yes=0

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`
source $scriptdir/.env

if [ -n "$CABOT_LAUNCH_CONFIG_NAME" ]; then
    config_name=$CABOT_LAUNCH_CONFIG_NAME
fi
if [ -n "$CABOT_LAUNCH_DO_NOT_RECORD" ]; then
    do_not_record=$CABOT_LAUNCH_DO_NOT_RECORD
fi
if [ -n "$CABOT_LAUNCH_RECORD_CAMERA" ]; then
    record_cam=$CABOT_LAUNCH_RECORD_CAMERA
fi
if [ -n "$CABOT_LAUNCH_LOG_PREFIX" ]; then
    log_prefix=$CABOT_LAUNCH_LOG_PREFIX
fi

while getopts "hsdrp:n:vc:3DSy" arg; do
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
	c)
	    config_name=$OPTARG
	    ;;
	3)
	    config_name=rs3
	    ;;
	D)
	    debug=1
	    ;;
	S)
	    screen_recording=1
	    ;;
	y)
	    yes=1
	    ;;
    esac
done
shift $((OPTIND-1))

## private variables
pids=()
termpids=()

## check nvidia-smi
if [ -z `which nvidia-smi` ]; then
    if [ -z $config_name ]; then
	red "[WARNING] cannot find nvidia-smi, so config_name is changed to 'nuc'"
	config_name=nuc
    fi
else
    nvidia_gpu=1
fi

## check required environment variables
error=0
if [ -z $CABOT_MODEL ]; then
    err "CABOT_MODEL: environment variable should be specified (ex. cabot2-gt1"
    error=1
fi
if [ -z $CABOT_SITE ]; then
    err "CABOT_SITE : environment variable should be specified (ex. cabot_site_cmu_3d"
    error=1
fi

if [ "$config_name" = "rs3" ]; then
    if [ -z $CABOT_REALSENSE_SERIAL_1 ]; then
	err "CABOT_REALSENSE_SERIAL_1: environment variable should be specified"
	error=1
    fi
    if [ -z $CABOT_REALSENSE_SERIAL_2 ]; then
	err "CABOT_REALSENSE_SERIAL_2: environment variable should be specified"
	error=1
    fi
    if [ -z $CABOT_REALSENSE_SERIAL_3 ]; then
	err "CABOT_REALSENSE_SERIAL_3: environment variable should be specified"
	error=1
    fi
    reset_all_realsence=1
fi

if [[ "$config_name" = "nuc" ]]; then
    if [[ -z $CABOT_JETSON_CONFIG ]]; then
	err "CABOT_JETSON_CONFIG: environment variable should be specified to launch people on Jetson"
	error=1
    fi
fi

if [ $error -eq 1 ]; then
   exit 1
fi

cabot_site_dir=$(find $scriptdir/cabot_sites -name $CABOT_SITE)
if [ -e $cabot_site_dir/server_data ]; then
    local_map_server=1
fi

log_name=${log_prefix}_`date +%Y-%m-%d-%H-%M-%S`
export ROS_LOG_DIR="/home/developer/.ros/log/${log_name}"
export ROS_LOG_DIR_ROOT="/root/.ros/log/${log_name}"
export CABOT_LOG_NAME=$log_name
host_ros_log=$scriptdir/docker/home/.ros/log
host_ros_log_dir=$host_ros_log/$log_name
ln -sf $host_ros_log_dir $host_ros_log/latest
blue "log dir is : $host_ros_log_dir"
mkdir -p $host_ros_log_dir
cp $scriptdir/.env $host_ros_log_dir/env-file

## run script to change settings
if [ $simulation -eq 0 ]; then
    if [ $nvidia_gpu -eq 1 ]; then
        blue "change nvidia gpu settings"
        $scriptdir/tools/change_nvidia-smi_settings.sh
    fi
fi

# prepare ROS host_ws
blue "build host_ws"
cd $scriptdir/host_ws
source /opt/ros/galactic/setup.bash  # todo
if [ $verbose -eq 0 ]; then
    colcon build > /dev/null
else
    colcon build
fi
if [ $? -ne 0 ]; then
   exit $!
fi

# launch command_logger with the host ROS
cd $scriptdir/host_ws
source install/setup.bash
if [ $verbose -eq 0 ]; then
    ROS_LOG_DIR=$host_ros_log_dir ros2 launch cabot_debug record_system_stat.launch.xml > $host_ros_log_dir/record-system-stat.log  2>&1 &
else
    ROS_LOG_DIR=$host_ros_log_dir ros2 launch cabot_debug record_system_stat.launch.xml &
fi
termpids+=($!)
pids+=($!)
blue "[$!] launch system stat $( echo "$(date +%s.%N) - $start" | bc -l )"

# launch docker image for bag recording
cd $scriptdir
additional_record_topics=()
if [ $do_not_record -eq 0 ]; then
    bag_dccom="docker-compose -f docker-compose-bag.yaml"
    sim_option=""
    if [[ $simulation -eq 1 ]]; then
	# sim_option="-s"
	sim_option=""  # workaround the problem with replay
    fi
    if [[ $record_cam -eq 1 ]]; then
	com="setsid $bag_dccom run --rm bag /launch-bag.sh record -r $sim_option > $host_ros_log_dir/docker-compose-bag.log 2>&1"
	blue $com
	eval $com &

	red "override CABOT_DETECT_VERION = 2"
	export CABOT_DETECT_VERSION=2
    else
	com="setsid $bag_dccom run --rm bag /launch-bag.sh record $sim_option > $host_ros_log_dir/docker-compose-bag.log 2>&1"
	blue $com
	eval $com &
    fi
    pids+=($!)
    blue "[$!] recording ROS2 topics $( echo "$(date +%s.%N) - $start" | bc -l )"
else
    blue "do not record ROS2 topics"
fi

if [[ $terminating -eq 1 ]]; then
    exit
fi

## launch docker-compose
cd $scriptdir
dcfile=

dcfile=docker-compose
if [ ! -z $config_name ]; then dcfile="${dcfile}-$config_name"; fi
if [ $simulation -eq 0 ]; then dcfile="${dcfile}-production"; fi
if [ $debug -eq 1 ]; then dcfile=docker-compose-debug; fi            # only basic debug
dcfile="${dcfile}.yaml"

if [ ! -e $dcfile ]; then
    err "There is not $dcfile (config_name=$config_name, simulation=$simulation)"
    exit
fi

dccom="docker-compose $project_option -f $dcfile"

if [ $local_map_server -eq 1 ]; then
    blue "Checking the map server is available $( echo "$(date +%s.%N) - $start" | bc -l )"
    curl http://localhost:9090/map/map/floormaps.json --fail > /dev/null 2>&1
    test=$?
    launching_server=0
    while [[ $test -ne 0 ]]; do
	if [[ $launching_server -eq 1 ]]; then
	    snore 5
	    blue "waiting the map server is ready..."
	    curl http://localhost:9090/map/map/floormaps.json --fail > /dev/null 2>&1
	    test=$?
	else
	    if [[ $yes -eq 0 ]]; then
		red "Note: launch.sh no longer launch server in the script"
		red -n "You need to run local web server for $CABOT_SITE, do you want to launch the server [Y/N]: "
		read -r ans
	    else
		ans=y
	    fi
	    if [[ $ans = 'y' ]] || [[ $ans = 'Y' ]]; then
		launching_server=1
		gnome-terminal -- bash -c "./server-launch.sh -d $cabot_site_dir/server_data; exit"
	    else
		echo ""
		exit 1
	    fi
	fi
    done
fi

if [ $reset_all_realsence -eq 1 ]; then
    # sudo resetsh.sh
    docker-compose run --rm people sudo /resetrs.sh $CABOT_REALSENSE_SERIAL_1
    docker-compose run --rm people sudo /resetrs.sh $CABOT_REALSENSE_SERIAL_2
    docker-compose run --rm people sudo /resetrs.sh $CABOT_REALSENSE_SERIAL_3
fi

if [ $verbose -eq 0 ]; then
    com2="bash -c \"setsid $dccom --ansi never up --no-build --abort-on-container-exit\" > $host_ros_log_dir/docker-compose.log &"
else
    com2="bash -c \"setsid $dccom up --no-build --abort-on-container-exit\" | tee $host_ros_log_dir/docker-compose.log &"
fi
if [ $verbose -eq 1 ]; then
    blue "$com2"
fi

if [[ $terminating -eq 1 ]]; then
    exit
fi

eval $com2
dcpid=($!)
pids+=($!)
blue "[$dcpid] $dccom up $( echo "$(date +%s.%N) - $start" | bc -l )"

## launch jetson
if [[ ! -z $CABOT_JETSON_CONFIG ]]; then
    : "${CABOT_JETSON_USER:=cabot}"
    : "${CABOT_CAMERA_RGB_FPS:=30}"
    : "${CABOT_CAMERA_DEPTH_FPS:=15}"
    : "${CABOT_CAMERA_RESOLUTION:=640}"
    : "${CABOT_DETECT_VERSION:=3}"

    serial_nums=
    if [ ! -z $CABOT_CAMERA_NAME_1 ] && [ ! -z $CABOT_REALSENSE_SERIAL_1 ]; then
        serial_nums="$serial_nums $CABOT_CAMERA_NAME_1:$CABOT_REALSENSE_SERIAL_1"
    fi
    if [ ! -z $CABOT_CAMERA_NAME_2 ] && [ ! -z $CABOT_REALSENSE_SERIAL_2 ]; then
        serial_nums="$serial_nums $CABOT_CAMERA_NAME_2:$CABOT_REALSENSE_SERIAL_2"
    fi
    if [ ! -z $CABOT_CAMERA_NAME_3 ] && [ ! -z $CABOT_REALSENSE_SERIAL_3 ]; then
        serial_nums="$serial_nums $CABOT_CAMERA_NAME_3:$CABOT_REALSENSE_SERIAL_3"
    fi

    simopt=
    if [ $simulation -eq 1 ]; then simopt="-s"; fi

    if [ $verbose -eq 1 ]; then
        com="./jetson-launch.sh -v -u $CABOT_JETSON_USER -c \"$CABOT_JETSON_CONFIG\" -S \"$serial_nums\" -f $CABOT_CAMERA_RGB_FPS -p $CABOT_CAMERA_DEPTH_FPS -r $CABOT_CAMERA_RESOLUTION -o $CABOT_DETECT_VERSION $simopt &"
    else
        com="./jetson-launch.sh -v -u $CABOT_JETSON_USER -c \"$CABOT_JETSON_CONFIG\" -S \"$serial_nums\" -f $CABOT_CAMERA_RGB_FPS -p $CABOT_CAMERA_DEPTH_FPS -r $CABOT_CAMERA_RESOLUTION -o $CABOT_DETECT_VERSION $simopt > $host_ros_log_dir/jetson-launch.log &"
    fi

    if [ $verbose -eq 1 ]; then
        blue "$com"
    fi
    eval $com
    termpids+=($!)
    pids+=($!)
    blue "[$!] launch jetson $( echo "$(date +%s.%N) - $start" | bc -l )"
fi

if [[ $screen_recording -eq 1 ]]; then
    blue "Recording screen"
    $scriptdir/record_screen.sh -d $host_ros_log_dir > /dev/null 2>&1 &
    termpids+=($!)
    pids+=($!)
fi

while [[ $launched -lt 5 ]]; do
    snore 1
    launched=$((launched+1))
done

blue "All launched: $( echo "$(date +%s.%N) - $start" | bc -l )"
while [ 1 -eq 1 ];
do
    # check if any of container got Exit status
    if [[ $terminating -eq 0 ]] && [[ `$dccom ps | grep Exit | wc -l` -gt 0 ]]; then
	red "docker-compose may have some issues. Check errors in the log or run with '-v' option."
	ctrl_c 1
	exit
    fi
    snore 1
done
