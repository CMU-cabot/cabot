#!/bin/bash

# Copyright (c) 2020 Carnegie Mellon University
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
    ## kill recoding node first ensure the recording is property finished
    
    echo "killing recording nodes..."
    rosnode list | grep record | while read -r line
    do
	rosnode kill $line
    done

    ## then killing all other nodes
    echo "killing all ros nodes..."
    rosnode kill -a

    for pid in ${pids[@]}; do
	echo "killing $pid..."
	kill -s 15 $pid
    done

    ## we need to wait gazebo is terminated
    gzc=0
    while [ `ps -A | grep gz | wc -l` -ne 0 ];
    do
	sleep 1
	echo -ne "waiting gazebo is completely terminated ($gzc)"\\r
	gzc=$((gzc+1))
    done
    echo \\n
    exit
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
# default params
map=$scriptdir/maps/cmu4F_gazebo.yaml
anchor=
world=$scriptdir/worlds/cmu4F.world

explore=0

## debug
minimum=0
debug=0
command='setsid xterm -e'
commandpost='&'
skip=0

gplanner='base_global_planner:=navfn/NavfnROS'
lplanner='base_local_planner:=dwa_local_planner/DWAPlannerROS'
mode='normal'

robot='cabot-f'
robot_desc='cabot-f'
odom_topic='odom'
cmd_vel_topic='/cabot/raw_cmd_vel'

hector=0

initx=0
inity=0
inita=0

gazebo=0

obstacles=0
human_detector=0
with_human=1
offset=0.25 # for cabot
cabot_menu=0
teleop=0
bagfile=
skip_check=0
no_vibration=false
init_speed=
use_amcl=1
show_rviz=1
language=en
isolated=0

### usage print function
function usage {
    echo "Usage"
    echo "ex)"
    echo $0 "-l teb -r cabot-f -s"
    echo ""
    echo "-h                       show this help"
    echo "-E                       explore mode (use gmapping)"
    echo "-i                       minimum mode, only launch robot"
    echo "-d                       debug (without xterm)"
    echo "-p                       skip launching gazebo and rviz (for debug)"
    echo "-m <map file>            specify a map file"
    echo "-n <anchor file>         specify a anchor file, use map file if not specified"
    echo "-w <world file>          specify a world file"
    echo "-g navfn|navcog          specify a global planner"
    echo "-l dwa|teb|tra           specify a local planner"
    echo "-r cabot-e|cabot-f       specify a robot"
    echo "-e                       use hector mapping to get odometry instead of using robot's odometry"
    echo "-x <initial x>           specify initial position of x"
    echo "-y <initial y>           specify initial position of y"
    echo "-a <initial angle>       specify initial angle (degree)"
    echo "-s                       specify its on simulation (gazebo)"
    echo "-o                       use obstacle detector and obstacle bridge"
    echo "-v                       use human detector"
    echo "-f                       use robot footprint without human"
    echo "-t <offset in meters>    use offset"
    echo "-u                       use cabot menu"
    echo "-z                       use teleop"
    echo "-b <prefix>              record bagfile with prefix"
    echo "-F                       skip device check"
    echo "-N                       no vibration"
    echo "-S <initial speed>       set initial maximum speed"
    echo "-c                       static (no amcl)"
    echo "-O                       performance (no rviz)"
    echo "-L <language code>       language code"
	echo "-I                       use isolated build"
    exit
}

### parse options
while getopts "hEidem:n:w:g:l:x:y:a:r:psoft:uzvb:FNS:cOL:I" arg; do
    case $arg in
	h)
	    usage
	    exit
	    ;;
	E)
	    explore=1
	    ;;
	i)
	    minimum=1
	    ;;
	d)
	    debug=1
	    command="setsid xterm -e '"
	    commandpost=";read'&"
	    ;;
	e)
	    hector=1
	    ;;
	m)
	    map=$OPTARG
	    ;;
	n)
	    anchor=$OPTARG
	    ;;
	w)
	    world=$OPTARG
	    ;;
	g)
	    if [ "$OPTARG" == "navfn" ]; then
		gplanner='base_global_planner:=navfn/NavfnROS'
	    elif [ "$OPTARG" == "navcog" ]; then
		gplanner='base_global_planner:=navcog_global_planner/NavCogGlobalPlanner'
		mode="navcog"
	    fi
	    ;;
	l)
	    if [ "$OPTARG" == "dwa" ]; then
		lplanner='base_local_planner:=dwa_local_planner/DWAPlannerROS'
	    elif [ "$OPTARG" == "teb" ]; then
		lplanner='base_local_planner:=teb_local_planner/TebLocalPlannerROS'
	    elif [ "$OPTARG" == "tra" ]; then
		lplanner='base_local_planner:=base_local_planner/TrajectoryPlannerROS'
	    fi
	    ;;
	r)
	    robot=$OPTARG
	    robot_desc=$robot
	    ;;
	x)
	    initx=$OPTARG
	    ;;
	y)
	    inity=$OPTARG
	    ;;
	a)
	    inita=`echo "$OPTARG * 3.1415926535 / 180.0" | bc -l`
	    ;;
	p)
	    skip=1
	    ;;
	s)
	    gazebo=1
	    ;;
	o)
	    obstacles=1
	    ;;
	v)
	    human_detector=1
	    ;;
        f)
	    with_human=0
	    offset=0
            ;;
	t)
	    offset=$OPTARG
	    ;;
	u)
	    cabot_menu=1
	    ;;
	z)
	    teleop=1
	    ;;
	b)
	    bagfile=$OPTARG
	    ;;
	F)
	    skip_check=1
	    ;;
	N)
	    no_vibration=true
	    ;;
	S)
	    init_speed="init_speed:=$OPTARG"
	    ;;
	c)
	    use_amcl=0
	    ;;
	O)
	    show_rviz=0
	    ;;
	L)
	    language=$OPTARG
	    ;;
	I)
		isolated=1
		;;
  esac
done
shift $((OPTIND-1))

## check variables
if [ "$anchor" == "" ]; then
    pat=".*\.pbstream"
    if [[ $map =~ $pat ]]; then
	echo "if you use $map, you need to specify a anchor file with -n option"
	exit
    fi
    anchor=$map
fi

if [[ $robot =~ ^(cabot-e|cabot-f)$ ]]; then
    echo -n ""
else
    echo "Unknown robot : $robot"
    exit
fi

## run catkin_make to make sure it is built before running
cd $catkin_ws
if [ $debug -eq 1 ]; then
	if [ $isolated -eq 1 ]; then
		catkin_make_isolated --use-ninja
	else
    	catkin_make
	fi
else
	if [ $isolated -eq 1 ]; then
		catkin_make_isolated --use-ninja > /dev/null
	else
    	catkin_make > /dev/null
	fi
fi
if [ $? -ne 0 ]; then
    exit
fi
source $catkin_ws/devel/setup.bash


## get absolute path to files
cd $pwd
cd $(dirname $world)
worlddir=`pwd`
worldname=$(basename $world)
world=$worlddir/$worldname

cd $pwd
cd $(dirname $map)
mapdir=`pwd`
mapname=$(basename $map)
map=$mapdir/$mapname

cd $pwd
cd $(dirname $anchor)
anchordir=`pwd`
anchorname=$(basename $anchor)
anchor=$anchordir/$anchorname


## robot specific actions
if [[ $robot =~ cabot* ]]; then
    odom_topic='/cabot/odom'
    hector=0
fi

if [ $robot == 'cabot-e' ]; then
  cmd_vel_topic='/cabot/raw_cmd_vel'
fi

## this checks all the related devices/powers are ready
if [ $robot == 'cabot-f' ]; then
    cmd_vel_topic='/cabot/raw_cmd_vel'

    ## check robot status and wait everything is ready
    if [ $gazebo -eq 0 -a $skip_check -eq 0 ]; then
	while [ 1 -eq 1 ]
	do
	    error=0

	    ## if it is laptop, if it is powered
	    if [[ `upower -i /org/freedesktop/UPower/devices/battery_BAT1 | grep state` =~ .*discharging.* ]]; then
		echo -e "\e[31mLaptop is not powered\e[0m"
		error=1
	    else
		echo "Laptop is powered!"
	    fi

	    ## cabot board
	    if [ `ls /dev/ttyCABOT 2> /dev/null | wc -l` -ne 1 ]; then
		echo -e "\e[31mCabot is not connected or power is off\e[0m"
		error=1
	    else
		echo "Cabot is connected!"
	    fi

	    ## Arduino for handle
	    if [ `ls /dev/ttyVIB1 2> /dev/null  | wc -l` -ne 1 ]; then
		echo -e "\e[31mVibration handle is not connected\e[0m"
		error=1
	    else
		echo "Vibration is connected!"
	    fi

	    ## Roboclaw motor controller
	    if [ `ls /dev/ttyROBOCLAW 2> /dev/null  | wc -l` -ne 1 ]; then
		echo -e "\e[31mRoboclaw motor controller is not connected\e[0m"
		error=1
	    else
		echo "Roboclaw is connected!"
	    fi

	    ## Hokuyo lidar
	    ping 192.168.0.10 -c 1 2> /dev/null > /dev/null
	    if [ $? -ne 0 ]; then
		echo -e "\e[31mLidar Network is not configured\e[0m"
		error=1
	    else
		echo "Lidar is connected!"
	    fi

	    ## ZED if human detector is on
	    if [ $human_detector -eq 1 ]; then
		if [ `lsusb | grep 2b03:f582 | wc -l` -ne 1 ]; then
		    echo -e "\e[31mZED camera is not connected\e[0m"
		    error=1
		else
		    echo "ZED is connected!"
		fi
	    fi
	    ## gamepad if teleop is on
	    if [ $teleop -eq 1 ]; then
		if [ `ls /dev/input/js0 2> /dev/null | wc -l` -ne 1 ]; then
		    echo -e "\e[31mGamepad is not connected\e[0m"
		    error=1
		else
		    echo "Gamepad is connected!"
		fi
	    fi
	    ## if there is no error, break
	    if [ $error -eq 0 ]; then
		break
	    fi
	    echo "Check connection or restart with -F option to skip check"
	    sleep 1
	    UPLINE=$(tput cuu1)
	    ERASELINE=$(tput el)
	    ## this remove previous line 
	    echo -n "$UPLINE$ERASELINE"
	    echo -n "$UPLINE$ERASELINE"
	    echo -n "$UPLINE$ERASELINE"
	    echo -n "$UPLINE$ERASELINE"
	    echo -n "$UPLINE$ERASELINE"
	    echo -n "$UPLINE$ERASELINE"
	    if [ $human_detector -eq 1 ]; then
		echo -n "$UPLINE$ERASELINE"
	    fi
	    if [ $teleop -eq 1 ]; then
		echo -n "$UPLINE$ERASELINE"
	    fi
	done
    fi
fi

## debug output
echo "Explore       : $explore"
echo "Minimum       : $minimum"
echo "Debug         : $debug ($command, $commandpost)"
echo "Skip          : $skip"
echo "World         : $world"
echo "Map           : $map"
echo "Anchor        : $anchor"
echo "Global planner: $gplanner"
echo "Navigatin Mode: $mode"
echo "Local planner : $lplanner"
echo "Robot         : $robot"
echo "Robot Desc    : $robot_desc"
echo "Odom topic    : $odom_topic"
echo "CmdVel topic  : $cmd_vel_topic"
echo "Hector        : $hector"
echo "Init x        : $initx"
echo "Init y        : $inity"
echo "Init a        : $inita"
echo "Simulation    : $gazebo"
echo "Obstacle brige: $obstacles"
echo "With human    : $with_human"
echo "Human Detector: $human_detector"
echo "Robot offset  : $offset"
echo "Cabot menu    : $cabot_menu"
echo "Teleop        : $teleop"
echo "Bagfile       : $bagfile"
echo "Skip Check    : $skip_check"
echo "No vibration  : $no_vibration"
echo "Init Speed    : $init_speed"


## TODO: this should be fixed
## implicit variable passing to amcl_demo.launch file
export TURTLEBOT_MAP_FILE=$map
export ROBOT_INITIAL_POSE="-x $initx -y $inity -Y $inita"

## launch ROS core
rosnode list
if [ $? -eq 1 ]; then
    eval "$command roscore $commandpost"
    pids+=($!)
fi

rosnode list
test=$?
while [ $test -eq 1 ]; do
    sleep 0.1
    c=$((c+1))
    echo "wait roscore" $c
    rosnode list
    test=$?
done

## launch robot
if [ $skip -eq 0 ]; then
    ### For GAZEBO simulation
    if [ $gazebo -eq 1 ]; then
	    echo "launch $robot on gazebo"
	    eval "$command roslaunch cabot_gazebo cabot_world.launch world_file:=$world\
		      offset:=$offset robot:=$robot_desc no_vibration:=$no_vibration $commandpost"
	    pids+=($!)

	    echo "simulate cabot button with keyboard"
	    eval "$command roslaunch cabot_ui cabot_keyboard.launch $commandpost"
	    pids+=($!)
		sleep 5
    else
    ### For real robot
	    echo "bringup $robot"
	    eval "$command roslaunch cabot $robot.launch offset:=$offset no_vibration:=$no_vibration $commandpost"
	    pids+=($!)
    fi

    ## launch rviz
    if [ $minimum -eq 0 ] && [ $show_rviz -eq 1 ]; then
	echo "launch rviz"
	eval "$command roslaunch cabot_ui view_cabot.launch $commandpost"
	pids+=($!)
    fi

    ## launch teleop
    if [ $teleop -eq 1 ]; then
	echo "launch teleop"
	eval "$command roslaunch cabot_ui teleop_gamepad.launch $commandpost"
	pids+=($!)
    fi
fi


## make direcotry for a bagfile
if [ "$bagfile" != "" ]; then
    mkdir -p `dirname $scriptdir/bags/$bagfile`
fi

## launch main navigation launch file
if [ $minimum -eq 0 ]; then
    launch="amcl_demo.launch"
    if [ $explore -eq 1 ]; then
	launch="gmapping_demo.launch"
    fi

    echo "launch $launch with hector=$hector"
    eval "$command roslaunch cabot_navigation $launch \
      odom_topic:=$odom_topic \
      cmd_vel_topic:=$cmd_vel_topic \
      hector:=$hector $lplanner $gplanner \
      initial_pose_x:=$initx initial_pose_y:=$inity \
      initial_pose_a:=$inita \
      robot:=$robot_desc obstacles:=$obstacles \
      with_human:=$with_human \
      use_amcl:=$use_amcl \
      $commandpost"

    pids+=($!)

    ## deprecated
    if [ $obstacles -eq 1 ]; then
	eval "$command roslaunch obstacle_bridge nodes.launch $commandpost"
	pids+=($!)
    fi

    ## launch human detector
    if [ $human_detector -eq 1 ]; then
	eval "$command roslaunch human_detector human_detector.launch avi_path:=\"$scriptdir/bags/$bagfile\" $commandpost"
	pids+=($!)
    fi

    # launch menu after navigation stack
    if [ $cabot_menu -eq 1 ]; then
	echo "launch cabot handle menu"
	mkdir -p $scriptdir/db
	eval "$command roslaunch cabot_ui cabot_menu.launch \
	     anchor_file:='$anchor' db_path:='$scriptdir/db' $init_speed \
	     language:=$language $commandpost"
	pids+=($!)
    fi
fi

## record bag file
if [ "$bagfile" != "" ]; then
    read -r -d '' bagcommand <<- EOF
$command rosbag record -o $scriptdir/bags/$bagfile
/cabot/clutch
/cabot/event
/cabot/imu_fixed
/cabot/imu/data
/cabot/imu/data_raw
/cabot/motorStatus
/cabot/motorTarget
/cabot/cabot_e_sensor/wrench_norm
/cabot/wrench
/cabot/odom
/cabot/odom_hector
/cabot/odom_raw
/cabot/odometry/filtered
/cabot/raw_cmd_vel
/cabot/cmd_vel
/cabot/poi
/cabot/speak
/cabot/notification
/cabot/vibrator1
/cabot/vibrator2
/cabot/vibrator3
/cabot/vibrator4
/cabot/user_speed
/cabot/map_speed
/map
/map_metadata
/move_base/DWAPlannerROS/global_plan
/move_base/DWAPlannerROS/local_plan
/move_base/NavfnROS/plan
/move_base/global_costmap/costmap
/move_base/global_costmap/costmap_updates
/move_base/global_costmap/footprint
/move_base/local_costmap/costmap
/move_base/local_costmap/costmap_updates
/move_base/local_costmap/footprint
/move_base_simple/goal
/navcog/cancel
/navcog/destination
/scan
/scan/parameter_descriptions
/scan/parameter_updates
/tf
/tf_static
/tracked_humans
$commandpost
EOF
    eval $bagcommand

    pids+=($!)
fi

## launch error console
if [ $debug -eq 1 ]; then
    eval "$command rqt_console $commandpost"
    pids+=($!)
fi
   
## wait until it is terminated by the user
while [ 1 -eq 1 ];
do
    sleep 1
done
