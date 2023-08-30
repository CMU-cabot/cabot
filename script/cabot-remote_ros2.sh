#!/bin/bash

# Copyright (c) 2023  Carnegie Mellon University and IBM Corporation
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

# variables for process termination
pids=()
termpids=()
checks=()

## debug
debug=0
command_prefix=''
command_postfix='&'

# load utility functions
source $scriptdir/cabot_util.sh

trap signal INT TERM

function signal() {
    blue "trap cabot_ros2.sh "

    # ps -Af
    kill -INT -1
    for pid in ${termpids[@]}; do
	kill -TERM $pid
    done
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
	    # ps -Af
            snore 1
	    count=$((count+1))
        done
    done

    exit
}

# environment variables
# required variables
: ${CABOT_MODEL:=}
: ${CABOT_TOUCH_PARAMS:=}

# optional variables
: ${CABOT_GAMEPAD:=}
: ${CABOT_REMOTE_USE_KEYBOARD:=false}
: ${CABOT_REMOTE_USE_IMU:=false}

function show_config {
    echo "CABOT_MODEL               : $CABOT_MODEL"
    echo "CABOT_TOUCH_PARAMS        : $CABOT_TOUCH_PARAMS"
    echo "CABOT_GAMEPAD             : $CABOT_GAMEPAD"
    echo "CABOT_REMOTE_USE_KEYBOARD : $CABOT_REMOTE_USE_KEYBOARD"
    echo "CABOT_REMOTE_USE_IMU      : $CABOT_REMOTE_USE_IMU"
}

function usage {
    echo "Usage"
    echo "./cabot-remote_ros2.sh"
    echo "-h       show this help"
    echo "-l       show the list of parameter values"
    exit
}

# get options
while getopts "hl" arg; do
    case $arg in
    h)
	    usage
	    exit
	    ;;
    l)
	    show_config
	    exit
	    ;;
    esac
done
shift $((OPTIND-1))

show_config

# source ros2_ws
source ~/ros2_ws/install/setup.bash

# run teleop twist keyboard in another terminal
if $CABOT_REMOTE_USE_KEYBOARD; then
    blue "run teleop_twist_keyboard"
    com="setsid xterm -e ros2 run teleop_twist_keyboard teleop_twist_keyboard &"
    echo $com
    eval $com
    pids+=($!)
fi

# check cabot major version to switch venv and launch file
cabot_major=${CABOT_MODEL:0:6} # cabotN
venv_path=/opt/venv/$cabot_major/bin/activate
cabot_remote_launch_py=$cabot_major.launch.py

blue "launch $cabot_major-remote"
com="$command_prefix \
    source $venv_path && \
    ros2 launch cabot $cabot_major-remote.launch.py \
    use_imu:=$CABOT_REMOTE_USE_IMU"
    # use_keyboard is not set here because teleop-twist-keyboard is run in another terminal
# append gamepad if defined
if [[ ! -z $CABOT_GAMEPAD ]]; then
    com=$com" gamepad:="$CABOT_GAMEPAD
fi
com=$com" $command_postfix"

echo $com
eval $com
checks+=($!)
pids+=($!)


## wait until it is terminated by the user
while [ 1 -eq 1 ];
do
    # blue "snore"
    for pid in ${checks[@]}; do
        kill -0 $pid 2> /dev/null
	if [[ $? -ne 0 ]]; then
	    red "process (pid=$pid) is not running, please check logs"
	    exit
	fi
    done
    snore 1
done
