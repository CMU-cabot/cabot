#!/bin/bash

# Copyright (c) 2022  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sellcho
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
    for pid in ${pids[@]}; do
	echo "killing $pid..."
	kill -s 2 $pid
    done

    ## we need to wait all nodes are terminated
    rlc=0
    while [ `ps -A | grep  roslaunch | wc -l` -ne 0 ];
    do
	snore 1
	echo -ne "waiting nodes are completely terminated ($rlc)"\\r
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

while [ ${PWD##*/} != "catkin_ws" ]; do
    cd ..
done
catkin_ws=`pwd`

### default variables

## debug
debug=0
command=''
commandpost='&'

# default environment variable
: ${CABOT_MODEL:=cabot2-gt1}
: ${CABOT_NAME:=cabot_name_needs_to_be_specified}
: ${CABOT_SITE:=}
: ${CABOT_LANG:=en}
: ${CABOT_OFFSET:=0.25}
: ${CABOT_TOUCH_PARAMS:='[128,48,24]'}
: ${CABOT_INIT_SPEED:=}

: ${CABOT_GAMEPAD:=gamepad}
: ${CABOT_SHOW_GAZEBO_CLIENT:=0}
: ${CABOT_SHOW_ROS1_RVIZ:=0}

: ${CABOT_INITX:=0}
: ${CABOT_INITY:=0}
: ${CABOT_INITZ:=0}
: ${CABOT_INITA:=0}  # in degree

: ${CABOT_GAZEBO:=0}
: ${CABOT_TOUCH_ENABLED:1}


# command line options
cabot_ui_manager=0
teleop=0
show_topology=0
use_cache=0

# swithces for mapping purpose
publish_state=1
camera_type=realsense
use_arduino=1

# configuration for cabot site scripts
gazebo=$CABOT_GAZEBO   # read by config script
global_map_name=map
map=
anchor=
world=

# only for experiments
no_vibration=false

# fixed parameters
use_tf_static=1
use_tts=0
use_ble=1

### usage print function
function usage {
    echo "Usage"
    echo ""
    echo "-h                       show this help"
    echo "-d                       debug (without xterm)"
    echo "-m <map file>            specify a map file"
    echo "-a <anchor file>         specify a anchor file, use map file if not specified"
    echo "-w <world file>          specify a world file"
    echo "-r                       use teleop"
    echo "-t                       show topology"
    echo "-u                       use cabot menu"
    echo "-n                       no vibration"
    echo "-c                       use built cache"
    echo "-M                       for gazebo mapping"
    exit
}

while getopts "hdm:a:w:rtuncM" arg; do
    case $arg in
	h)
	    usage
	    exit
	    ;;
	d)
	    debug=1
	    command="setsid xterm -e \""
	    commandpost=";read\"&"
	    ;;
	m)
	    map=$OPTARG
	    ;;
	a)
	    anchor=$OPTARG
	    ;;
	w)
	    world=$OPTARG
	    ;;
	r)
	    teleop=1
	    ;;
	t)
	    show_topology=1
	    ;;
	u)
	    cabot_ui_manager=1
	    ;;
	n)
	    no_vibration=true
	    ;;
	M)
	    publish_state=0
	    camera_type=none
	    use_arduino=0
	    use_speedlimit=0
	    ;;
	c)
	    use_cache=1
	    ;;
  esac
done
shift $((OPTIND-1))

## run catkin_make to make sure it is built before running
cd $catkin_ws
if [ $use_cache -eq 0 ]; then
    if [ $debug -eq 1 ]; then
	catkin_make #_isolated --use-ninja
    else
	catkin_make > /dev/null
    fi
    if [ $? -ne 0 ]; then
	exit
    fi
else
    echo "Skip building workspace"
fi
source $catkin_ws/devel/setup.bash

if [ "$CABOT_SITE" != "" ]; then
    gazebo=$CABOT_GAZEBO
    sitedir=`rospack find $CABOT_SITE`
    source $sitedir/config/config.sh
    if [ "$map" == "" ] && [ "$world" == "" ]; then
	echo "Please check config/config.sh in site package ($sitedir) to set map and world"
	exit
    fi
else
    if [ "$map" == "" ]; then
	echo "-T <site> or -m <map> should be specified"
	exit
    fi
    if [ $CABOT_GAZEBO -eq 1 ] && [ "$world" == "" ]; then
	echo "-T <site> or -w <world> should be specified"
	exit
    fi
fi

## check variables
if [ "$anchor" == "" ]; then
    pat=".*\.pbstream"
    if [[ $map =~ $pat ]]; then
	echo "if you use $map, you need to specify a anchor file with -n option"
    exit
fi
    anchor=$map
fi

## get absolute path to files
function realpath() {
    local file=$1
    cd $pwd
    cd $(dirname $file)
    local dir=`pwd`
    local name=$(basename $file)
    echo "$dir/$name"
}
world=$(realpath $world)
map=$(realpath $map)
anchor=$(realpath $anchor)


## check required files
echo -n ""
files=(`rospack find cabot`/launch/${CABOT_MODEL}.launch \
       `rospack find cabot_gazebo`/launch/includes/${CABOT_MODEL}.launch.xml \
       `rospack find cabot_description`/robots/${CABOT_MODEL}.urdf.xacro \
       `rospack find cabot_description`/urdf/${CABOT_MODEL}_description.urdf \
       )
error=0
for file in ${files[@]}
do
    if [ ! -e $file ]; then
	echo `realpath $file` "does not exists"
	error=1
    fi
done

if [ $error -eq 1 ]; then
    echo "Check files or restart with -F option to skip check"	
    exit
fi

CABOT_INITAR=`echo "$CABOT_INITA * 3.1415926535 / 180.0" | bc -l`

## debug output
echo "Environment variables---"
echo "CABOT_MODEL             : $CABOT_MODEL"
echo "CABOT_NAME              : $CABOT_NAME"
echo "CABOT_SITE              : $CABOT_SITE"
echo "CABOT_LANG              : $CABOT_LANG"
echo "CABOT_OFFSET            : $CABOT_OFFSET"
echo "CABOT_TOUCH_PARAMS      : $CABOT_TOUCH_PARAMS"
echo "CABOT_INIT_SPEED        : $CABOT_INIT_SPEED"
echo "CABOT_GAZEBO            : $CABOT_GAZEBO"
echo "CABOT_TOUCH_ENABLED     : $CABOT_TOUCH_ENABLED"
echo "Debug variables---------"
echo "CABOT_GAMEPAD           : $CABOT_GAMEPAD"
echo "CABOT_SHOW_GAZEBO_CLIENT: $CABOT_SHOW_GAZEBO_CLIENT"
echo "CABOT_SHOW_ROS1_RVIZ    : $CABOT_SHOW_ROS1_RVIZ"
echo "Gazebo init loc---------"
echo "CABOT_INITX             : $CABOT_INITX"
echo "CABOT_INITY             : $CABOT_INITY"
echo "CABOT_INITZ             : $CABOT_INITZ"
echo "CABOT_INITA             : $CABOT_INITA"
echo ""
echo "Options ----------------"
echo "Debug                   : $debug ($command, $commandpost)"
echo "World                   : $world"
echo "Map                     : $map"
echo "Global Map              : $global_map_name"
echo "Anchor                  : $anchor"
echo ""
echo "Cabot UI Mng.           : $cabot_ui_manager"
echo "Teleop                  : $teleop"
echo "Show Topology           : $show_topology"
echo "No vibration            : $no_vibration"
echo "Use TF Static           : $use_tf_static"
echo "Use TTS                 : $use_tts"
echo "Use BLE                 : $use_ble"

rosnode list
if [ $? -eq 1 ]; then
    eval "$command roscore $commandpost"
    pids+=($!)
fi

rosnode list
test=$?
while [ $test -eq 1 ]; do
    snore 0.1
    c=$((c+1))
    echo "wait roscore" $c
    rosnode list
    test=$?
done

### For GAZEBO simulation
if [ $CABOT_GAZEBO -eq 1 ]; then
    echo "launch $CABOT_MODEL on gazebo"
    eval "$command roslaunch cabot_gazebo cabot_world.launch \
    	      offset:=$CABOT_OFFSET robot:=$CABOT_MODEL no_vibration:=$no_vibration \
	      initial_pose_x:=$CABOT_INITX \
              initial_pose_y:=$CABOT_INITY \
	      initial_pose_z:=$CABOT_INITZ \
	      initial_pose_a:=$CABOT_INITAR \
	      enable_touch:=$CABOT_TOUCH_ENABLED \
	      world_file:=$world \
	      publish_state:=$publish_state \
              use_tf_static:=$use_tf_static \
	      gui:=$CABOT_SHOW_GAZEBO_CLIENT \
	      touch_params:=$CABOT_TOUCH_PARAMS \
	      camera_type:=$camera_type \
	      use_arduino:=$use_arduino \
	      use_speedlimit:=$use_speedlimit \
              $commandpost"
	
    pids+=($!)

    echo "simulate cabot button with keyboard"
    # always launch with xterm
    eval "setsid xterm -e roslaunch cabot_ui cabot_keyboard.launch &"
    pids+=($!)
    snore 5
else
    echo "bringup $CABOT_MODEL"
    eval "$command roslaunch cabot $CABOT_MODEL.launch \
              offset:=$CABOT_OFFSET no_vibration:=$no_vibration \
              use_tf_static:=$use_tf_static \
              enable_touch:=$CABOT_TOUCH_ENABLED \
	      touch_params:=$CABOT_TOUCH_PARAMS \
              $commandpost"
    pids+=($!)
fi


## launch rviz
if [ $CABOT_SHOW_ROS1_RVIZ -eq 1 ]; then
    echo "launch rviz"
    eval "$command roslaunch cabot_ui view_cabot.launch $commandpost"
    pids+=($!)
fi

## launch teleop
if [ $teleop -eq 1 ]; then
    echo "launch teleop"
    # always launch with xterm
    eval "setsid xterm -e roslaunch cabot_ui teleop_gamepad.launch gamepad:=$CABOT_GAMEPAD &"
    pids+=($!)
fi

# launch menu after navigation stack
if [ $cabot_ui_manager -eq 1 ]; then
    echo "launch cabot handle menu"
    mkdir -p $scriptdir/db
    com="$command roslaunch cabot_ui cabot_menu.launch \
    	     anchor_file:='$anchor' \
    	     db_path:='$scriptdir/db' \
             init_speed:='$CABOT_INIT_SPEED' \
	     language:='$CABOT_LANG' \
	     global_map_name:='$global_map_name' \
	     use_tts:=$use_tts \
             use_ble:=$use_ble \
	     ble_team:='$CABOT_NAME' \
             site:='$CABOT_SITE' \
	     show_topology:='$show_topology' \
	     $commandpost"
    echo $com
    eval $com
    pids+=($!)
fi

com="$command roslaunch cabot_debug record_sar.launch $commandpost"
echo $com
eval $com
pids+=($!)

## launch error console
if [ $debug -eq 1 ]; then
    eval "$command rqt_console $commandpost"
    pids+=($!)
fi
   
## wait until it is terminated by the user
while [ 1 -eq 1 ];
do
    snore 1
done
