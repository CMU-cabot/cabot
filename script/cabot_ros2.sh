#!/bin/bash

# Copyright (c) 2020, 2022  Carnegie Mellon University
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

# change directory to where this script exists
pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

pids=()

## debug
debug=0
command_prefix=''
command_postfix='&'

# load utility functions
source $scriptdir/cabot_util.sh

trap signal INT TERM

function signal() {
    blue "trap cabot_ros2.sh "

    kill -INT -1
    for pid in ${pids[@]}; do
	count=0
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
    done
    
    exit
}

# initialize environment variables
# required variables
: ${CABOT_SITE:=}
: ${CABOT_MODEL:=}
: ${CABOT_TOUCH_PARAMS:=}
# variables with default value
: ${CABOT_GAZEBO:=0}
: ${CABOT_INITX:=0}
: ${CABOT_INITY:=0}
: ${CABOT_INITZ:=0}
: ${CABOT_INITA:=0}  # in degree
export CABOT_INITAR=$(echo "$CABOT_INITA * 3.1415926535 / 180.0" | bc -l)

# TODO
# : ${CABOT_FOOTPRINT_RADIUS:=0.45}
# : ${CABOT_OFFSET:=0.25}
: ${CABOT_LANG:=en}
: ${CABOT_TOUCH_ENABLED:=true}
: ${CABOT_GAMEPAD:=ps4}
: ${CABOT_SHOW_GAZEBO_CLIENT:=0}
: ${CABOT_SHOW_ROS2_RVIZ:=0}
: ${CABOT_SHOW_ROS2_LOCAL_RVIZ:=0}
: ${CABOT_SHOW_ROBOT_MONITOR:=1}
# optional variables
# TODO
: ${CABOT_INIT_SPEED:=1.0}
## if 1, use IBM Watson tts service for '/speak_robot' service (PC speaker)
: ${CABOT_USE_ROBOT_TTS:=0}
: ${TEXT_TO_SPEECH_APIKEY:=}
: ${TEXT_TO_SPEECH_URL:=}

: ${CABOT_BLE_VERSION:=1}
## if 0, do not use CaBot iPhone app
## others, use external ble server for '/speak' and other services
use_ble=$CABOT_BLE_VERSION


# check required environment variables
error_flag=0
if [[ -z $CABOT_SITE ]]; then
    err "CABOT_SITE sould be configured"
    error_flag=1
fi
if [[ -z $CABOT_MODEL ]]; then
    err "CABOT_MODEL should be configured"
    error_flag=1
fi
if [[ $CABOT_TOUCH_ENABLED = 'true' ]] && [[ -z $CABOT_TOUCH_PARAMS ]]; then
    err "CABOT_TOUCH_PARAMS should be configured when CABOT_TOUCH_ENABLED is true"
    error_flag=1
fi
if [[ $error_flag -ne 0 ]]; then
    exit
fi

# initialize local variables
pid=
use_cache=0
use_sim_time=false
if [ $CABOT_GAZEBO -eq 1 ]; then use_sim_time=true; fi

# configuration for cabot site scripts
gazebo=$CABOT_GAZEBO   # read by config script



function usage {
    echo "Usage"
    echo ""
    echo "-h       show this help"
    echo "-c       use built cache"
    echo "-d       debug (with xterm)"
    exit
}


while getopts "hcd" arg; do
    case $arg in
	h)
	    usage
	    exit
	    ;;
	c)
	    use_cache=1
	    ;;
	d)
	    debug=1
	    command_prefix="setsid xterm -e \""
	    command_postfix=";read\"&"
	    ;;
    esac
done
shift $((OPTIND-1))

while [ ${PWD##*/} != "ros2_ws" ]; do
    cd ..
done
ros2_ws=`pwd`
cd $ros2_ws
if [ $use_cache -eq 0 ]; then
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo 
    if [ $? -ne 0 ]; then
	exit
    fi
else
    echo "Skip building workspace"
fi
source $ros2_ws/install/setup.bash

# TODO: Remove cabot_site_configuration from shell script
if [ ! -z $CABOT_SITE ]; then
    sitedir=`ros2 pkg prefix $CABOT_SITE`/share/$CABOT_SITE
    echo $sitedir
    source $sitedir/config/config.sh
    if [ "$map" == "" ]; then
	echo "Please check config/config.sh in site package ($sitedir) to set map and world"
	exit
    fi
else
    if [ "$map" == "" ]; then
	echo "-T <site> or -m <map> should be specified"
	exit
    fi
fi



echo "CABOT_GAZEBO              : $CABOT_GAZEBO"
echo "CABOT_SITE                : $CABOT_SITE"
echo "CABOT_SHOW_ROS2_RVIZ      : $CABOT_SHOW_ROS2_RVIZ"
echo "CABOT_SHOW_ROS2_LOCAL_RVIZ: $CABOT_SHOW_ROS2_LOCAL_RVIZ"
echo "Map                       : $map"
echo "Use Sim Time              : $use_sim_time"
echo "Use BLE                 : $use_ble"

if [[ $CABOT_GAZEBO -eq 1 ]]; then
    blue "launch gazebo"
    com="$command_prefix ros2 launch cabot_gazebo cabot2_gazebo.launch.py \
        model:=$CABOT_MODEL \
        world_file:=$world \
        wireless_config_file:=$wireless_config \
        $command_postfix"
    echo $com
    eval $com
    pids+=($!)
    blue "launch cabot_keyboard teleop"
    com="setsid xterm -e ros2 run cabot_ui cabot_keyboard.py &"
    echo $com
    eval $com
    pids+=($!)
else
    blue "bringup $CABOT_MODEL"
    com="$command_prefix ros2 launch cabot cabot2.launch.py $command_postfix"
    echo $com
    eval $com
    pids+=($!)
fi

# launch ui_manager
blue "launch cabot handle menu"
com="$command_prefix ros2 launch cabot_ui cabot_ui.launch.py \
        anchor_file:='$map' \
        init_speed:='$CABOT_INIT_SPEED' \
        language:='$CABOT_LANG' \
        site:='$CABOT_SITE' \
        show_topology:=true \
        $command_postfix"
echo $com
eval $com
pids+=($!)

com="$command_prefix ros2 launch cabot_navigation2 bringup_launch.py \
    map:=$map \
    autostart:=true \
    use_sim_time:=$use_sim_time \
    show_rviz:=$CABOT_SHOW_ROS2_RVIZ \
    show_local_rviz:=$CABOT_SHOW_ROS2_LOCAL_RVIZ \
    record_bt_log:=false \
    $command_postfix"
echo $com
eval $com
pids+=($!)


#  TODO
#  record_bt_log:=$CABOT_RECORD_ROSBAG2 \
    
echo "launch cabot diagnostic"
com="$command_prefix ros2 launch cabot_ui cabot_diagnostic.launch.xml \
        show_robot_monitor:=$CABOT_SHOW_ROBOT_MONITOR \
        $command_postfix"
echo $com
eval $com
pids+=($!)

if [ $use_ble -ne 0 ]; then
    echo "launch rosbridge for cabot BLE"
    com="$command ros2 launch cabot_ui cabot_ext_ble.launch.xml \
	     $command_postfix"
    echo $com
    eval $com
    pids+=($!)
fi


## wait until it is terminated by the user
while [ 1 -eq 1 ];
do
    # blue "snore"
    snore 1
done


