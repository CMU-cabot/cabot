#!/bin/bash

# Copyright (c) 2020  Carnegie Mellon University
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
    # echo "killing all ros nodes..."
    # rosnode kill -a

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
# default params
explore=0

## debug
minimum=0
debug=0
command=''
commandpost='&'
skip=0

gplanner='base_global_planner:=navfn/NavfnROS'
lplanner='base_local_planner:=dwa_local_planner/DWAPlannerROS'
mode='normal'

robot='cabot2-e2'
robot_desc='cabot2-e2'
odom_topic='odom'
cmd_vel_topic='cmd_vel'
plan_topic='/move_base/NavfnROS/plan'

initx=0
inity=0
initz=0
inita=0

gazebo=0
gazebo_gui=1

obstacles=0
human_detector=0
with_human=1
offset=0.25 # for cabot
cabot_menu=0
teleop=0
bagdir=$HOME/.ros/log
bagfile=
skip_check=0
no_vibration=false
init_speed=
use_amcl=1
show_rviz=1
language=en
site=
publish_state=1
use_tf_static=1
action_name=/move_base
enable_speed_handle=false
global_map_name=map
gamepad=gamepad
extra_topics=
use_tts=1
use_ble=0
ble_team=cabot_name_needs_to_be_specified
use_cache=0
touch_params='[128,48,24]'
camera_type=realsense
use_arduino=1
use_speedlimit=1
show_topology=0

### usage print function
function usage {
    echo "Usage"
    echo "ex)"
    echo $0 "-l teb -r cabot2-e2 -s"
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
    echo "-r cabot2-e2|cabot1-f    "
    echo "   cabot2-s1|cabot2-gt1  specify a robot"
    echo "-x <initial x>           specify initial position of x"
    echo "-y <initial y>           specify initial position of y"
    echo "-Z <initial z>           specify initial position of z in gazebo"
    echo "-a <initial angle>       specify initial angle (degree)"
    echo "-s                       specify its on simulation (gazebo)"
    echo "-H                       headless simulation (no gazebo GUI)" 
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
    echo "-T <site package>        packge name for the robot site"
    echo "-B                       Configure for ROS2 bridge (no static tf, action client name)"
    echo "-X                       engage touch feedback for robot"
    echo "-G                       specify a gamepad"
    echo "-A <topics>              extra topic names to be recorded"
    echo "-e <cabot name>          use ble connection (disable default TTS and use ble TTS)"
    echo "-D                       disable TTS (external TTS service)"
    echo "-c                       use built cache"
    echo "-P <touch param>         touch threshold parameters like '[baseline,touch,release]'"
    echo "-M                       for gazebo mapping"
    echo "-Y                       show topology"
    exit
}

while getopts "hEidm:n:w:g:l:x:y:Z:a:r:psHoft:uzvb:FNS:cOL:T:BXG:A:e:DcP:MY" arg; do
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
	    command="setsid xterm -e \""
	    commandpost=";read\"&"
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
		gplanner='base_global_planner:=cabot_navigation/NavCogGlobalPlanner'
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
	Z)
	    initz=$OPTARG
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
	H)
	    gazebo_gui=0
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
	    init_speed=$OPTARG
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
	T)
	    site=$OPTARG
		;;
	B)
	    action_name=/navigate_to_pose
	    plan_topic=/plan
	    ;;
	X)  
            enable_speed_handle=true
            ;;
	G)
	    gamepad=$OPTARG
	    ;;
	A)
	    extra_topics="$extra_topics $OPTARG"
	    ;;
	e)
	    use_tts=0
	    use_ble=1
	    ble_team=$OPTARG
	    ;;
	D)
	    use_tts=0
	    use_ble=0
	    ;;
	c)
	    use_cache=1
	    ;;
	P)
	    touch_params=$OPTARG
	    ;;
	M)
	    publish_state=0
	    camera_type=none
	    use_arduino=0
	    use_speedlimit=0
	    ;;
	Y)
	    show_topology=1
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

if [ "$site" != "" ]; then
    sitedir=`rospack find $site`
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
    if [ $gazebo -eq 1 ] && [ "$world" == "" ]; then
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
if [[ $robot =~ ^(cabot2-e2|cabot2-s1|cabot2-gt1|cabot1-f)$ ]]; then
    echo -n ""
    files=(`rospack find cabot`/launch/${robot}.launch \
	   `rospack find cabot_gazebo`/launch/includes/${robot}.launch.xml \
	   `rospack find cabot_description`/robots/${robot}.urdf.xacro \
	   `rospack find cabot_description`/urdf/${robot}_description.urdf \
	   `rospack find cabot_navigation`/param/${robot}_footprint.yaml\
	   `rospack find cabot_navigation`/param/${robot}_human_footprint.yaml\
	  )
    error=0
    for file in ${files[@]}
    do
	if [ ! -e $file ]; then
	    echo `realpath $file` "does not exists"
	    error=1
fi
    done

    if [ $skip_check -eq 0 ]; then
	if [ $error -eq 1 ]; then
	    echo "Check files or restart with -F option to skip check"	
	    exit
	fi
    fi
else
    echo "Unknown robot : $robot"
    exit
fi

if [ $robot == 'cabot1-f' ]; then
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
	    snore 1
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
echo "Init x        : $initx"
echo "Init y        : $inity"
echo "Init Z        : $initz"
echo "Init a        : $inita"
echo "Simulation    : $gazebo"
echo "Simulation GUI: $gazebo_gui"
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
echo "Use TF Static : $use_tf_static"
echo "Action Name   : $action_name"
echo "Touch Mode    : $enable_speed_handle"
echo "Global Map    : $global_map_name"
echo "Gamepad       : $gamepad"
echo "Extra Topics  : $extra_topics"
echo "Use TTS       : $use_tts"
echo "Use BLE       : $use_ble"
echo "BLE team      : $ble_team"
echo "Touch Params  : $touch_params"
echo "Show Rviz     : $show_rviz"
echo "Show Topology : $show_topology"

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

## launch robot
if [ $skip -eq 0 ]; then
    ### For GAZEBO simulation
    if [ $gazebo -eq 1 ]; then
	    echo "launch $robot on gazebo"
	eval "$command roslaunch cabot_gazebo cabot_world.launch \
	      offset:=$offset robot:=$robot_desc no_vibration:=$no_vibration \
	      initial_pose_x:=$initx initial_pose_y:=$inity \
	      initial_pose_z:=$initz \
	      initial_pose_a:=$inita \
	      enable_touch:=$enable_speed_handle \
	      world_file:=$world \
	      publish_state:=$publish_state \
              use_tf_static:=$use_tf_static \
	      gui:=$gazebo_gui \
	      touch_params:=$touch_params \
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
	    echo "bringup $robot"
	eval "$command roslaunch cabot $robot.launch \
              offset:=$offset no_vibration:=$no_vibration \
              use_tf_static:=$use_tf_static \
              enable_touch:=$enable_speed_handle \
	      touch_params:=$touch_params \
              $commandpost"
	    pids+=($!)
    fi
fi

## launch rviz
if [ $show_rviz -eq 1 ]; then
    echo "launch rviz"
    eval "$command roslaunch cabot_ui view_cabot.launch $commandpost"
    pids+=($!)
fi

## launch teleop
if [ $teleop -eq 1 ]; then
    echo "launch teleop"
    # always launch with xterm
    eval "setsid xterm -e roslaunch cabot_ui teleop_gamepad.launch gamepad:=$gamepad &"
    pids+=($!)
fi


## make direcotry for a bagfile
if [ "$bagfile" != "" ]; then
    if [ ! -z $ROS_HOME ]; then
	bagdir=$ROS_HOME/log
    fi
    if [ ! -z $ROS_LOG_DIR ]; then
	bagdir=$ROS_LOG_DIR
    fi
    mkdir -p $bagdir
fi

## launch main navigation launch file
if [ $minimum -eq 0 ]; then
    launch="amcl_demo.launch"
    if [ $explore -eq 1 ]; then
	launch="gmapping_demo.launch"
    fi

    echo "launch $launch"
    eval "$command roslaunch cabot_navigation $launch \
      map_file:='$map' \
      odom_topic:=$odom_topic \
      cmd_vel_topic:=$cmd_vel_topic \
      $lplanner $gplanner \
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
	eval "$command roslaunch human_detector human_detector.launch avi_path:=\"$bagdir/$bagfile\" $commandpost"
	pids+=($!)
    fi
fi

# launch menu after navigation stack
if [ $cabot_menu -eq 1 ]; then
    echo "launch cabot handle menu"
    mkdir -p $scriptdir/db
    com="$command roslaunch cabot_ui cabot_menu.launch \
    	     anchor_file:='$anchor' \
    	     db_path:='$scriptdir/db' \
             init_speed:='$init_speed' \
	     language:='$language' \
	     action_name:='$action_name' \
	     global_map_name:='$global_map_name' \
	     plan_topic:='$plan_topic' \
	     use_tts:=$use_tts \
             use_ble:=$use_ble \
	     ble_team:='$ble_team' \
             site:='$site' \
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

## record bag file
if [ "$bagfile" != "" ]; then
    read -r -d '' bagcommand <<- EOF
$command rosbag record -O $bagdir/$bagfile
/cmd_vel
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
/cabot/touch_raw
/cabot/touch
/cabot/notification
/cabot/vibrator1
/cabot/vibrator2
/cabot/vibrator3
/cabot/vibrator4
/cabot/user_speed
/cabot/map_speed
/cabot/lidar_speed
/cabot/touch_speed_raw
/cabot/touch_speed
/cabot/touch_speed_switched
/cabot/people_speed
/initialpose
/joy
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
/navigate_to_pose/goal
/navigate_to_pose/feedback
/particlecloud
/path
/sar
/scan
/scan1
/scan/parameter_descriptions
/scan/parameter_updates
/spin/goal
/spin/feedback
/tf
/tf_static
$extra_topics
$commandpost
EOF
    echo $bagcommand
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
    snore 1
done
