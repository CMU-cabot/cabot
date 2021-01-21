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
	snore 1
	echo -ne "waiting gazebo is completely terminated ($gzc)"\\r
	gzc=$((gzc+1))
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

pids=()

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

while [ ${PWD##*/} != "catkin_ws" ]; do
    cd ..
done
catkin_ws=`pwd`

map=
world=$scriptdir/worlds/cmu4F.world

odom_topic='/cabot/odom'
initx=0
inity=0
initq="0 0 0 1"
gazebo=0
bag=""
bag_start_time=0
bag_duration=86400 # one day
lplanner='base_local_planner:=teb_local_planner/TebLocalPlannerROS'

command='setsid xterm -e'
commandpost='&'
mapping=cartographer
robot='turtlebot'
obstacles=0
teleop=0

function usage {
    echo "Usage"
    echo "ex)"
    echo $0 "-l teb -r turtlebot -s"
    echo ""
    echo "-h                       show this help"
    echo "-d                       debug (without xterm)"
    echo "-m <map file>            specify a map file"
    echo "-w <world file>          specify a world file"
    echo "-x <initial x>           specify initial position of x"
    echo "-y <initial y>           specify initial position of y"
    echo "-q <quotanion>           specify initial quatanion"
    echo "-a cartographer|hector|gmapping   specify mapping method"
    echo "-r cabot|turtlebot       specify a robot"
    echo "-s                       specify its on simulation (gazebo)"
    echo "-b                       bag"
    echo "-t                       bag start time"
    echo "-u                       bag duration"
    echo "-o                       use obstacle detector and obstacle bridge"    
    echo "-z                       use teleop"
    exit
}


while getopts "hdm:w:r:x:y:q:sa:b:t:u:oz" arg; do
    case $arg in
	h)
	    usage
	    exit
	    ;;
	d)
	    command="setsid xterm -e '"
	    commandpost=";read'&"
	    ;;
	m)
	    map=$OPTARG
	    ;;
	w)
	    world=$OPTARG
	    ;;
	r)
	    robot=$OPTARG
	    ;;
	x)
	    initx=$OPTARG
	    ;;
	y)
	    inity=$OPTARG
	    ;;
	q)
	    initq=$OPTARG
	    ;;
	a)
	    mapping=$OPTARG
	    ;;
	s)
	    gazebo=1
	    ;;
	b)
	    bag=$OPTARG
	    ;;
	t)
	    bag_start_time=$OPTARG
	    ;;
	u)
	    bag_duration=$OPTARG
	    ;;
	o)
	    obstacles=1
	    ;;
	z)
	    teleop=1
	    ;;
    esac
done
shift $((OPTIND-1))

if [ "$mapping" == "cartographer" ]; then
	cd $catkin_ws
	catkin_make_isolated --use-ninja 
	source $catkin_ws/devel_isolated/setup.bash
else
	cd $catkin_ws
	catkin_make
	source $catkin_ws/devel_isolated/setup.bash
fi

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


echo "Mappint is $mapping"
echo "Use $world as a world"
echo "Use $map as a map"
echo "Init x = $initx"
echo "Init y = $inity"
echo "Robot is $robot"
echo "Obstacles is $obstacles"
echo "Teleop is $teleop"

export TURTLEBOT_MAP_FILE=$map
export ROBOT_INITIAL_POSE="-x $initx -y $inity"

use_sim_time=false

if [ $gazebo -eq 1 ]; then
    use_sim_time=true
fi

if [ "$bag" == "" ]; then
    if [ $gazebo -eq 1 ]; then
		echo "launch cabot on gazebo"
		echo "$command roslaunch cabot_gazebo cabot_world.launch world_file:=$world $commandpost"
		eval "$command roslaunch cabot_gazebo cabot_world.launch world_file:=$world $commandpost"
		pids+=($!)
		sleep 5
		echo "simulate cabot button with keyboard"
		eval "$command roslaunch cabot_ui cabot_keyboard.launch $commandpost"
		pids+=($!)
		sleep 2
    else
	    echo "bringup cabot"
	    eval "$command roslaunch cabot $robot.launch $commandpost"
	    pids+=($!)
	    sleep 5
	    
	    echo "launch cabot handle menu"
	    mkdir -p $scriptdir/db
	    eval "$command roslaunch cabot_ui cabot_menu.launch \
	     	  db_path:='$scriptdir/db' $commandpost"
	    pids+=($!)
	    sleep 5	    
    fi
fi

launch="${mapping}_mapping.launch"
#robot="cabot"
bagoption="offline:=false use_sim_time:=$use_sim_time"

if [ $teleop -eq 1 ]; then
    echo "launch teleop"
    eval "$command roslaunch cabot_ui teleop_gamepad.launch $commandpost"	
    pids+=($!)
    snore 2
fi


if [ "$bag" != "" ]; then
    if [ "$bag" == "rqt_bag" ]; then
	bagoption="offline:=true use_sim_time:=true"
    else
	bagoption="use_bag:=true bag_filename:=$bag \
                   bag_start_time:=$bag_start_time \
                   bag_duration:=$bag_duration \
                   offline:=true use_sim_time:=true"
    fi
fi

eval "$command roslaunch cabot_navigation $launch \
      $bagoption \
      initial_pose_x:=$initx initial_pose_y:=$inity \
      odom_topic:=$odom_topic \
      robot:=$robot \
      $commandpost"
pids+=($!)

if [ $obstacles -eq 1 ]; then
    eval "$command roslaunch obstacle_bridge nodes.launch $commandpost"
    pids+=($!)
fi


if [ "$bag" == "rqt_bag" ]; then
    eval "$command rqt_bag --clock $commandpost"
    pids+=($!)
elif [ "$bag" == "" ]; then
    mkdir -p ~/bags/mapping
    read -r -d '' bagcommand <<- EOF
$command rosbag record -o ~/bags/mapping/${mapping}
/cabot/imu_fixed
/cabot/imu/data
/cabot/imu/data_raw
/cabot/motorStatus
/cabot/motorTarget
/cabot/odom
/cabot/odom_hector
/cabot/odom_raw
/cabot/odometry/filtered
/cabot/raw_cmd_vel
/cabot/cmd_vel
/scan
/scan1
/scan/parameter_descriptions
/scan/parameter_updates
/tf
/tf_static
/velodyne_points
$commandpost
EOF
    eval $bagcommand
    pids+=($!)
fi

## wait until it is terminated by the user
while [ 1 -eq 1 ];
do
    snore 1
done
